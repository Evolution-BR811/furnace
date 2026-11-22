/**
 * Furnace Tracker - multi-system chiptune tracker
 * Copyright (C) 2021-2026 tildearrow and contributors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

// importador de VGM (por Evolution BR811)
//
// em vez de tocar os registradores como uma gravacao, o VGM e "desmontado"
// em uma musica editavel:
// 1. os comandos do VGM sao lidos e um 2A03 e simulado (envelope, sweep,
//    contador de duracao, contador linear e DMC), tirando uma foto do estado
//    de cada canal a cada tick do driver do jogo (normalmente 60 Hz).
// 2. o inicio das notas e detectado e o espaco entre elas vira a grade de
//    linhas (velocidade ou groove de duas velocidades).
// 3. cada nota vira nota + instrumento. tudo o que muda dentro da nota
//    (envelope de volume, duty, arpejo, vibrato, slide) vai para as macros
//    do instrumento, tick a tick, entao a musica soa como o original.
//    instrumentos iguais sao reaproveitados.
// 4. DPCM vira um kit com mapa de samples; PCM ($4011) vira sample de 8 bits.
//
// a afinacao usa a propria tabela de periodos do Furnace ao contrario, entao
// cada periodo do jogo e reproduzido exatamente (nota + ajuste fino).
//
// YM2413 (OPLL) e VRC7: a nota e o key on (o envelope e do chip). o patch
// vai para o instrumento (e para a macro de wave quando muda dentro da nota),
// o patch custom vira os operadores do instrumento, mudancas nele durante a
// nota viram macros FM, e um key on no meio da nota vira macro de phase reset.
// o modo de ritmo usa o sistema "OPLL with drums" (18xx quando liga/desliga).
//
// YM2612 (OPN2): tambem e FM, entao a nota e o key on. o patch do canal (os 4
// operadores, algoritmo, feedback, AMS/FMS) vai para um instrumento FM; o TL
// das portadoras e relativo a mais alta, que vira o volume (0-127), como o
// Furnace faz. mudancas no patch durante a nota viram macros FM, o paneio vira
// macro de pan, o LFO vira 10xy. a afinacao usa a conta do proprio Furnace
// (calcFreq + escolha do bloco) ao contrario, com bloco fixo quando precisa.
// o DAC (comandos 0x8n, escritas no $2A e streams 0x90-0x95) vira samples de
// 8 bits tocadas no canal FM 6. o SN76489 junto vira o PSG do Genesis.

#include "fileOpsCommon.h"
#include "../bsr.h"
#include "../platform/sound/snes/SPC_DSP.h"
#include <algorithm>
#include <array>
#include <map>
#include <set>
#include <vector>

namespace {

const int vgmNESLengthTable[32]={
  10,254,20,2,40,4,80,6,160,8,60,10,14,12,26,14,
  12,16,24,18,48,20,96,22,192,24,72,26,16,28,32,30
};

const int vgmNESDMCRateNTSC[16]={
  428,380,340,320,286,254,226,214,190,160,142,128,106,84,72,54
};
const int vgmNESDMCRatePAL[16]={
  398,354,316,298,276,236,210,198,176,148,132,118,98,78,66,50
};

struct VGMInvalidException {};

// a register write or a RAM upload, in VGM sample time (44100 Hz)
struct VGMEvent {
  double time;
  int reg; // 0x00-0x17 = $4000-$4017, -1 = RAM block
  int val; // value, or RAM block index
};

struct VGMRAMBlock {
  unsigned short addr;
  std::vector<unsigned char> data;
};

struct VGMDACWrite {
  double time;
  int val;
};

// a write to the YM2612 DAC: its value, and where in the PCM data bank it
// came from (-1: written directly). rate: the stream's rate, if known
// a write to the 32X's PWM: channel (0 left, 1 right, 2 both), its 12-bit
// value, where in the PWM data it came from (-1: written directly), the
// stream's rate if known, and whether a stream starts there
struct VGMPWMWrite {
  double time;
  int ch;
  int val;
  int src;
  double rate;
  bool start=false;
};

// an SNES sample: its BRR blocks (as they are in the SPC's RAM) and the block
// its loop goes back to (-1: no loop)
struct VGMSNESSample {
  std::vector<unsigned char> brr;
  int loopBlock=-1;
  unsigned int addr=0;
};

struct VGMOPNDACWrite {
  double time;
  int val;
  int src;
  double rate;
  // the first write of a DAC stream started by the file (0x93/0x95): a
  // sample starts here, whatever the data around it
  bool start=false;
};

// state of a channel at one tick
struct VGMChanTick {
  bool aud=false;      // audible
  int period=0;        // timer period (pulse/triangle/5B) or noise index
  int vol=0;           // 0-15 (0 if not audible)
  int duty=0;          // pulse duty, noise mode, or 5B mixer (1 tone, 2 noise, 4 envelope)
  bool restart=false;  // length/phase register was written in this tick
  // Sunsoft 5B only
  int noise=0;         // noise period (shared by the 3 channels)
  int envShape=0, envPeriod=0;
  bool envRestart=false; // envelope shape written in this tick
  // YM2413/VRC7 only: the sound (patch, sustain, custom patch) of the channel,
  // an index into the converter's table of FM sounds
  int fm=-1;
  int block=0; // and the block the driver wrote
  // SID only: control register (without the gate), attack/decay, sustain/release
  int sidCtrl=0, sidAD=0, sidSR=0;
  // SNES only: the level of the envelope (ENVX, 0-127)
  int envx=0;
};

struct VGMDPCMTrigger {
  int tick;
  int addr, len, rate;
  bool loop;
  std::vector<unsigned char> data;
};

struct VGMDPCMStop {
  int tick;
};

// a minimal 2A03 sound model, only what affects pitch/volume per tick
struct VGMNESApu {
  bool pal;
  double clock;

  struct Pulse {
    int duty=0;
    bool halt=false, constVol=false;
    int volParam=0;
    bool envStart=false;
    int envDiv=0, envDecay=0;
    bool swEn=false, swNeg=false, swReload=false;
    int swPeriod=0, swShift=0, swDiv=0;
    int timer=0;
    int length=0;
    bool enabled=false;
  } pulse[2];

  struct Tri {
    bool control=false;
    int reloadVal=0;
    bool reloadFlag=false;
    int linear=0;
    int timer=0;
    int length=0;
    bool enabled=false;
  } tri;

  struct Noise {
    bool halt=false, constVol=false;
    int volParam=0;
    bool envStart=false;
    int envDiv=0, envDecay=0;
    bool mode=false;
    int index=0;
    int length=0;
    bool enabled=false;
  } noise;

  int dmcRate=0;
  bool dmcLoop=false;
  int dmcAddr=0xc000, dmcLen=1;
  bool dmcPlaying=false;
  double dmcEnd=0;

  bool fiveStep=false;
  double frameStart=0;
  int frameStep=0;
  double cyc=0;

  unsigned char ram[65536];

  // results the converter asks for
  std::vector<VGMDPCMTrigger>* triggers=NULL;
  std::vector<VGMDPCMStop>* stops=NULL;
  int curTick=0;

  VGMNESApu(): pal(false), clock(1789773.0) {
    memset(ram,0,65536);
  }

  double stepTime(int step) {
    // quarter frame clocks of the frame sequencer, in CPU cycles
    static const double four[4]={7457.5,14913.5,22371.5,29829.5};
    static const double five[4]={7457.5,14913.5,22371.5,37281.5};
    return frameStart+(fiveStep?five[step]:four[step]);
  }

  void clockEnv(bool& start, int& div, int& decay, int volParam, bool loop) {
    if (start) {
      start=false;
      decay=15;
      div=volParam;
    } else if (div==0) {
      div=volParam;
      if (decay>0) {
        decay--;
      } else if (loop) {
        decay=15;
      }
    } else {
      div--;
    }
  }

  int sweepTarget(int i) {
    int change=pulse[i].timer>>pulse[i].swShift;
    if (pulse[i].swNeg) {
      return pulse[i].timer-change-(i==0?1:0);
    }
    return pulse[i].timer+change;
  }

  void quarterFrame() {
    for (int i=0; i<2; i++) clockEnv(pulse[i].envStart,pulse[i].envDiv,pulse[i].envDecay,pulse[i].volParam,pulse[i].halt);
    clockEnv(noise.envStart,noise.envDiv,noise.envDecay,noise.volParam,noise.halt);
    if (tri.reloadFlag) {
      tri.linear=tri.reloadVal;
    } else if (tri.linear>0) {
      tri.linear--;
    }
    if (!tri.control) tri.reloadFlag=false;
  }

  void halfFrame() {
    for (int i=0; i<2; i++) {
      if (!pulse[i].halt && pulse[i].length>0) pulse[i].length--;
      int target=sweepTarget(i);
      if (pulse[i].swDiv==0 && pulse[i].swEn && pulse[i].swShift>0 && pulse[i].timer>=8 && target<=0x7ff) {
        pulse[i].timer=target;
      }
      if (pulse[i].swDiv==0 || pulse[i].swReload) {
        pulse[i].swDiv=pulse[i].swPeriod;
        pulse[i].swReload=false;
      } else {
        pulse[i].swDiv--;
      }
    }
    if (!tri.control && tri.length>0) tri.length--;
    if (!noise.halt && noise.length>0) noise.length--;
  }

  double dmcDuration() {
    const int* rt=pal?vgmNESDMCRatePAL:vgmNESDMCRateNTSC;
    return (double)dmcLen*8.0*(double)rt[dmcRate&15];
  }

  void advance(double to) {
    while (true) {
      double next=stepTime(frameStep);
      if (next>to) break;
      quarterFrame();
      if (frameStep==1 || frameStep==3) halfFrame();
      frameStep++;
      if (frameStep>=4) {
        frameStep=0;
        frameStart+=fiveStep?37282.0:29830.0;
      }
    }
    while (dmcPlaying && dmcEnd<=to) {
      if (dmcLoop) {
        dmcEnd+=dmcDuration();
      } else {
        dmcPlaying=false;
      }
    }
    cyc=to;
  }

  void dmcStart() {
    dmcPlaying=true;
    dmcEnd=cyc+dmcDuration();
    if (triggers!=NULL) {
      VGMDPCMTrigger t;
      t.tick=curTick;
      t.addr=dmcAddr;
      t.len=dmcLen;
      t.rate=dmcRate;
      t.loop=dmcLoop;
      t.data.resize(dmcLen);
      for (int i=0; i<dmcLen; i++) {
        int a=dmcAddr+i;
        // the DMC wraps to $8000 after $FFFF
        if (a>0xffff) a=0x8000+(a-0x10000);
        t.data[i]=ram[a&0xffff];
      }
      triggers->push_back(t);
    }
  }

  // returns true if this write (re)starts a note on the channel (length/phase register)
  int write(int reg, int val) {
    int restartChan=-1;
    switch (reg) {
      case 0x00: case 0x04: {
        Pulse& p=pulse[reg>>2];
        p.duty=val>>6;
        p.halt=val&0x20;
        p.constVol=val&0x10;
        p.volParam=val&15;
        break;
      }
      case 0x01: case 0x05: {
        Pulse& p=pulse[reg>>2];
        p.swEn=val&0x80;
        p.swPeriod=(val>>4)&7;
        p.swNeg=val&8;
        p.swShift=val&7;
        p.swReload=true;
        break;
      }
      case 0x02: case 0x06:
        pulse[reg>>2].timer=(pulse[reg>>2].timer&0x700)|val;
        break;
      case 0x03: case 0x07: {
        Pulse& p=pulse[reg>>2];
        p.timer=(p.timer&0xff)|((val&7)<<8);
        if (p.enabled) p.length=vgmNESLengthTable[val>>3];
        p.envStart=true;
        restartChan=reg>>2;
        break;
      }
      case 0x08:
        tri.control=val&0x80;
        tri.reloadVal=val&0x7f;
        break;
      case 0x0a:
        tri.timer=(tri.timer&0x700)|val;
        break;
      case 0x0b:
        tri.timer=(tri.timer&0xff)|((val&7)<<8);
        if (tri.enabled) tri.length=vgmNESLengthTable[val>>3];
        tri.reloadFlag=true;
        restartChan=2;
        break;
      case 0x0c:
        noise.halt=val&0x20;
        noise.constVol=val&0x10;
        noise.volParam=val&15;
        break;
      case 0x0e:
        noise.mode=val&0x80;
        noise.index=val&15;
        break;
      case 0x0f:
        if (noise.enabled) noise.length=vgmNESLengthTable[val>>3];
        noise.envStart=true;
        restartChan=3;
        break;
      case 0x10:
        dmcLoop=val&0x40;
        dmcRate=val&15;
        break;
      case 0x12:
        dmcAddr=0xc000+(val<<6);
        break;
      case 0x13:
        dmcLen=(val<<4)+1;
        break;
      case 0x15:
        pulse[0].enabled=val&1;
        pulse[1].enabled=val&2;
        tri.enabled=val&4;
        noise.enabled=val&8;
        if (!pulse[0].enabled) pulse[0].length=0;
        if (!pulse[1].enabled) pulse[1].length=0;
        if (!tri.enabled) tri.length=0;
        if (!noise.enabled) noise.length=0;
        if (val&16) {
          if (!dmcPlaying) dmcStart();
        } else {
          if (dmcPlaying && stops!=NULL) {
            VGMDPCMStop s;
            s.tick=curTick;
            stops->push_back(s);
          }
          dmcPlaying=false;
        }
        break;
      case 0x17:
        fiveStep=val&0x80;
        frameStart=cyc;
        frameStep=0;
        if (fiveStep) {
          quarterFrame();
          halfFrame();
        }
        break;
    }
    return restartChan;
  }

  void snapshot(VGMChanTick* out) {
    for (int i=0; i<2; i++) {
      Pulse& p=pulse[i];
      int v=p.constVol?p.volParam:p.envDecay;
      bool mute=(p.length==0 || p.timer<8 || (!p.swNeg && sweepTarget(i)>0x7ff) || !p.enabled);
      out[i].aud=(!mute && v>0);
      out[i].period=p.timer;
      out[i].vol=out[i].aud?v:0;
      out[i].duty=p.duty;
    }
    out[2].aud=(tri.enabled && tri.length>0 && tri.linear>0 && tri.timer>=2);
    out[2].period=tri.timer;
    out[2].vol=out[2].aud?15:0;
    out[2].duty=0;
    int nv=noise.constVol?noise.volParam:noise.envDecay;
    out[3].aud=(noise.enabled && noise.length>0 && nv>0);
    out[3].period=noise.index;
    out[3].vol=out[3].aud?nv:0;
    out[3].duty=noise.mode?1:0;
  }
};

unsigned int vgmRead32(const std::vector<unsigned char>& b, size_t pos) {
  if (pos+4>b.size()) return 0;
  return b[pos]|(b[pos+1]<<8)|(b[pos+2]<<16)|((unsigned int)b[pos+3]<<24);
}

String vgmUTF16ToUTF8(const std::vector<unsigned char>& b, size_t& pos, size_t end) {
  String ret;
  while (pos+1<end) {
    unsigned int c=b[pos]|(b[pos+1]<<8);
    pos+=2;
    if (c==0) break;
    if (c>=0xd800 && c<0xdc00 && pos+1<end) {
      unsigned int c2=b[pos]|(b[pos+1]<<8);
      if (c2>=0xdc00 && c2<0xe000) {
        pos+=2;
        c=0x10000+((c-0xd800)<<10)+(c2-0xdc00);
      }
    }
    if (c<0x80) {
      ret+=(char)c;
    } else if (c<0x800) {
      ret+=(char)(0xc0|(c>>6));
      ret+=(char)(0x80|(c&0x3f));
    } else if (c<0x10000) {
      ret+=(char)(0xe0|(c>>12));
      ret+=(char)(0x80|((c>>6)&0x3f));
      ret+=(char)(0x80|(c&0x3f));
    } else {
      ret+=(char)(0xf0|(c>>18));
      ret+=(char)(0x80|((c>>12)&0x3f));
      ret+=(char)(0x80|((c>>6)&0x3f));
      ret+=(char)(0x80|(c&0x3f));
    }
  }
  return ret;
}

// the Game Boy's sound (registers $FF10-$FF3F): what its four channels play
// as the writes come, with the frame sequencer (512 Hz: lengths at 256 Hz,
// the sweep at 128 Hz, envelopes at 64 Hz). times are in samples (44100 Hz)
struct VGMGBApu {
  unsigned char regs[0x30];
  unsigned char waveRam[16];
  struct Chan {
    bool on=false;
    int vol=0, envDir=0, envPer=0, envTimer=0;
    int len=0;
    bool lenEn=false;
    int freq=0;
    // (pulse 1's sweep)
    int sweepShadow=0, sweepPer=0, sweepShift=0, sweepTimer=0;
    bool sweepNeg=false, sweepOn=false;
  } ch[4];
  bool powered=true;
  double seqTime=0;
  int seqStep=0;
  bool restart[4]={false,false,false,false};
  VGMGBApu() {
    memset(regs,0,sizeof(regs));
    memset(waveRam,0,sizeof(waveRam));
    regs[0x26-0x10]=0x80;
    regs[0x25-0x10]=0xff;
  }
  bool dac(int c) const {
    if (c==2) return regs[0x1a-0x10]&0x80;
    return (regs[0x12-0x10+c*5]&0xf8)!=0;
  }
  int sweepCalc() {
    Chan& c=ch[0];
    int d=c.sweepShadow>>c.sweepShift;
    return c.sweepNeg?(c.sweepShadow-d):(c.sweepShadow+d);
  }
  void clockSeq() {
    // lengths: steps 0, 2, 4, 6
    if ((seqStep&1)==0) {
      for (int i=0; i<4; i++) {
        Chan& c=ch[i];
        if (c.lenEn && c.len>0) {
          if (--c.len==0) c.on=false;
        }
      }
    }
    // sweep: steps 2 and 6
    if (seqStep==2 || seqStep==6) {
      Chan& c=ch[0];
      if (c.sweepTimer>0) c.sweepTimer--;
      if (c.sweepTimer<=0) {
        c.sweepTimer=c.sweepPer?c.sweepPer:8;
        if (c.sweepOn && c.sweepPer) {
          int nf=sweepCalc();
          if (nf>2047) {
            c.on=false;
          } else if (c.sweepShift) {
            c.sweepShadow=nf;
            c.freq=nf;
            if (sweepCalc()>2047) c.on=false;
          }
        }
      }
    }
    // envelopes: step 7
    if (seqStep==7) {
      for (int i=0; i<4; i++) {
        if (i==2) continue;
        Chan& c=ch[i];
        if (!c.envPer) continue;
        if (--c.envTimer<=0) {
          c.envTimer=c.envPer;
          if (c.envDir && c.vol<15) c.vol++;
          if (!c.envDir && c.vol>0) c.vol--;
        }
      }
    }
    seqStep=(seqStep+1)&7;
  }
  void advance(double t) {
    const double step=44100.0/512.0;
    while (seqTime+step<=t) {
      seqTime+=step;
      if (powered) clockSeq();
    }
  }
  void write(int addr, unsigned char v) {
    if (addr>=0x30 && addr<0x40) {
      waveRam[addr-0x30]=v;
      return;
    }
    if (addr<0x10 || addr>=0x30) return;
    if (!powered && addr!=0x26) return;
    regs[addr-0x10]=v;
    if (addr==0x26) {
      bool p=v&0x80;
      if (!p && powered) {
        for (int i=0x10; i<0x26; i++) regs[i-0x10]=0;
        for (int i=0; i<4; i++) ch[i]=Chan();
      }
      if (p && !powered) seqStep=0;
      powered=p;
      return;
    }
    if (addr>=0x24) return;
    int c=(addr-0x10)/5;
    int r=(addr-0x10)%5;
    Chan& C=ch[c];
    switch (r) {
      case 0:
        if (c==0) {
          C.sweepPer=(v>>4)&7;
          C.sweepNeg=v&8;
          C.sweepShift=v&7;
        }
        break;
      case 1:
        C.len=(c==2)?(256-v):(64-(v&63));
        break;
      case 2:
        if (!dac(c)) C.on=false;
        break;
      case 3:
        if (c!=3) C.freq=(C.freq&0x700)|v;
        break;
      case 4:
        if (c!=3) C.freq=(C.freq&0xff)|((v&7)<<8);
        C.lenEn=v&0x40;
        if (v&0x80) {
          // trigger
          restart[c]=true;
          C.on=dac(c);
          if (C.len==0) C.len=(c==2)?256:64;
          if (c!=2) {
            unsigned char e=regs[0x12-0x10+c*5];
            C.vol=e>>4;
            C.envDir=(e>>3)&1;
            C.envPer=e&7;
            C.envTimer=C.envPer;
          }
          if (c==0) {
            C.sweepShadow=C.freq;
            C.sweepTimer=C.sweepPer?C.sweepPer:8;
            C.sweepOn=(C.sweepPer || C.sweepShift);
            if (C.sweepShift && sweepCalc()>2047) C.on=false;
          }
        }
        break;
    }
  }
  // the channels now: pulses and wave take the period Furnace uses
  // (2048 minus the register), the wave its volume as 15/10/5 and the wave
  // (an index, from waveIdx) in duty; the noise takes the NR43 value as its
  // period and the width bit as its duty. pan: 1 right, 2 left
  template<typename F> void snapshot(VGMChanTick* out, F waveIdx) {
    unsigned char nr51=regs[0x25-0x10];
    for (int i=0; i<4; i++) {
      VGMChanTick t;
      const Chan& C=ch[i];
      bool on=powered && C.on && dac(i);
      t.noise=((nr51>>i)&1)|(((nr51>>(i+4))&1)<<1);
      if (i==3) {
        unsigned char n=regs[0x22-0x10];
        t.period=n;
        t.duty=(n>>3)&1;
        t.vol=C.vol;
      } else {
        int p=2048-C.freq;
        if (p<1) p=1;
        if (p>2047) p=2047;
        t.period=p;
        if (i==2) {
          static const int wv[4]={0,15,10,5};
          t.vol=wv[(regs[0x1c-0x10]>>5)&3];
          t.duty=waveIdx(waveRam);
        } else {
          t.vol=C.vol;
          t.duty=regs[0x11-0x10+i*5]>>6;
        }
      }
      t.aud=on && t.vol>0 && t.noise!=0;
      if (!t.aud) t.vol=0;
      t.restart=restart[i];
      out[i]=t;
    }
    for (int i=0; i<4; i++) restart[i]=false;
  }
};

// a decompression table (data block 0x7F): for a compression type, sub-type
// and bit sizes, the values the compressed codes stand for
struct VGMDecompTable {
  int type=-1, subType=-1, bitsDec=0, bitsCmp=0;
  std::vector<unsigned int> values;
};

// decompresses a compressed data block (types 0x40-0x7E): bit packing (copy,
// shift left, table) or DPCM. the values come out little endian, in as many
// bytes as the decompressed bits need. returns false if it can't
bool vgmDecompressBlock(const unsigned char* d, size_t n, const std::vector<VGMDecompTable>& tables, std::vector<unsigned char>& out) {
  if (n<5) return false;
  const int ctype=d[0];
  const size_t outSize=d[1]|(d[2]<<8)|(d[3]<<16)|((size_t)d[4]<<24);
  if (n<10) return false;
  const int bitsDec=d[5];
  const int bitsCmp=d[6];
  const int subType=d[7];
  const unsigned int addVal=d[8]|(d[9]<<8);
  if (bitsDec<1 || bitsDec>16 || bitsCmp<1 || bitsCmp>16) return false;
  if (outSize>(64u<<20)) return false;
  const int outBytes=(bitsDec+7)/8;
  const unsigned int outMask=(bitsDec>=32)?0xffffffffu:((1u<<bitsDec)-1);
  const VGMDecompTable* table=NULL;
  if (ctype==1 || subType==2) {
    for (const VGMDecompTable& t: tables) {
      if (t.type==ctype && t.subType==subType && t.bitsDec==bitsDec && t.bitsCmp==bitsCmp) table=&t;
    }
    if (table==NULL) return false;
  }
  out.clear();
  out.reserve(outSize);
  const unsigned char* in=d+10;
  const size_t inLen=n-10;
  size_t bitPos=0;
  const size_t bitEnd=inLen*8;
  unsigned int acc=addVal; // (DPCM: the start value)
  while (out.size()+outBytes<=outSize && bitPos+bitsCmp<=bitEnd) {
    // (the codes are read in pieces of up to 8 bits, each one highest bit
    // first, and the first piece is the lowest bits of the code: like VGM
    // players do)
    unsigned int code=0;
    int outBit=0;
    int toRead=bitsCmp;
    while (toRead>0) {
      int nb=(toRead>=8)?8:toRead;
      toRead-=nb;
      unsigned int piece=0;
      for (int b=0; b<nb; b++, bitPos++) {
        piece=(piece<<1)|((in[bitPos>>3]>>(7-(bitPos&7)))&1);
      }
      code|=piece<<outBit;
      outBit+=nb;
    }
    unsigned int v=0;
    if (ctype==0) {
      switch (subType) {
        case 0: v=code+addVal; break;
        case 1: v=(code<<(bitsDec-bitsCmp))+addVal; break;
        case 2: v=(code<table->values.size())?table->values[code]:0; break;
        default: return false;
      }
    } else if (ctype==1) {
      acc+=(code<table->values.size())?table->values[code]:0;
      v=acc;
    } else {
      return false;
    }
    v&=outMask;
    for (int ob=0; ob<outBytes; ob++) out.push_back((v>>(8*ob))&0xff);
  }
  while (out.size()<outSize) out.push_back(0);
  return true;
}

bool vgmGunzip(const unsigned char* in, size_t len, std::vector<unsigned char>& out) {
  z_stream zs;
  memset(&zs,0,sizeof(zs));
  if (inflateInit2(&zs,16+MAX_WBITS)!=Z_OK) return false;
  zs.next_in=(Bytef*)in;
  zs.avail_in=len;
  unsigned char buf[65536];
  int ret;
  do {
    zs.next_out=buf;
    zs.avail_out=sizeof(buf);
    ret=inflate(&zs,Z_NO_FLUSH);
    if (ret!=Z_OK && ret!=Z_STREAM_END) {
      inflateEnd(&zs);
      return false;
    }
    out.insert(out.end(),buf,buf+(sizeof(buf)-zs.avail_out));
    if (out.size()>256*1024*1024) {
      inflateEnd(&zs);
      return false;
    }
  } while (ret!=Z_STREAM_END && zs.avail_in>0);
  inflateEnd(&zs);
  return !out.empty();
}

// place an effect in the first free effect column
void vgmPutFx(short* cell, int fx, int val, int& maxCol) {
  for (int i=0; i<DIV_MAX_EFFECTS; i++) {
    if (cell[DIV_PAT_FX(i)]==-1) {
      cell[DIV_PAT_FX(i)]=fx;
      cell[DIV_PAT_FXVAL(i)]=val;
      if (i+1>maxCol) maxCol=i+1;
      return;
    }
  }
}

// cut trailing repeats: a macro holds its last value after it ends
void vgmTrim(std::vector<int>& v) {
  while (v.size()>1 && v[v.size()-1]==v[v.size()-2]) v.pop_back();
}

// find a repeating tail (vibrato, tremolo, arpeggio cycles) and turn it into
// a macro loop, so that notes of any length share one instrument.
// returns the loop position, or -1.
int vgmLoopify(std::vector<int>& v) {
  int n=(int)v.size();
  int bestLen=n, bestS=-1, bestP=0;
  for (int p=2; p<=64 && p*2<=n; p++) {
    // walk back from the end while the values keep repeating every p
    int s=n-p;
    while (s>0 && v[s-1]==v[s-1+p]) s--;
    if (n-s<p*2) continue;
    if (s+p<bestLen) {
      bestLen=s+p;
      bestS=s;
      bestP=p;
    }
  }
  if (bestS<0) return -1;
  v.resize(bestS+bestP);
  return bestS;
}

// value of a macro at tick k, following its loop or holding the last value
int vgmMacroValueAt(const std::vector<int>& v, int loop, int k);

// like vgmLoopify, for a tail that repeats with its steps one tick early or
// late now and then (a driver slower than the tick rate, rounded by the VGM's
// timing). the loop must play every value of the original within a tick of
// where it was, and most of them exactly. returns the loop position, or -1
int vgmLoopifyLoose(std::vector<int>& v) {
  const int n=(int)v.size();
  int bestLen=n, bestS=-1, bestP=0;
  for (int p=2; p<=64 && p*2<=n; p++) {
    int s=n-p, loose=0;
    while (s>0) {
      const int i=s-1;
      if (v[i]==v[i+p]) {
        s--;
        continue;
      }
      if (v[i]==v[i+p-1] || (i+p+1<n && v[i]==v[i+p+1])) {
        loose++;
        s--;
        continue;
      }
      break;
    }
    if (n-s<p*2) continue;
    if (loose*4>(n-p-s)) continue;
    if (s+p>=bestLen) continue;
    // (check it: each value within a tick of the loop's)
    std::vector<int> lv(v.begin(),v.begin()+s+p);
    bool ok=true;
    for (int k=0; k<n && ok; k++) {
      if (v[k]==vgmMacroValueAt(lv,s,k)) continue;
      if (k>0 && v[k]==vgmMacroValueAt(lv,s,k-1)) continue;
      if (v[k]==vgmMacroValueAt(lv,s,k+1)) continue;
      ok=false;
    }
    if (!ok) continue;
    bestLen=s+p;
    bestS=s;
    bestP=p;
  }
  if (bestS<0) return -1;
  v.resize(bestS+bestP);
  return bestS;
}

// shorten a macro: either cut a held tail or loop a repeating one,
// whichever is shorter. this must see the whole note, held tail included,
// or it would invent loops. returns the loop position, or -1.
int vgmCompress(std::vector<int>& v) {
  std::vector<int> held=v;
  vgmTrim(held);
  std::vector<int> looped=v;
  int loop=(looped.size()>3)?vgmLoopify(looped):-1;
  if (loop>=0 && looped.size()<held.size()) {
    v=looped;
    return loop;
  }
  v=held;
  return -1;
}

// value of a macro at tick k, following its loop or holding the last value
int vgmMacroValueAt(const std::vector<int>& v, int loop, int k) {
  if (v.empty()) return 0;
  if (k<(int)v.size()) return v[k];
  if (loop>=0 && loop<(int)v.size()) {
    int ll=(int)v.size()-loop;
    return v[loop+((k-loop)%ll)];
  }
  return v.back();
}

// what Furnace plays from a sequence macro over n ticks, with its loop and
// release point, when the note is released at relTick (-1: never).
// (same rules as DivMacroStruct::doMacro, speed 1, no delay)
void vgmSimMacro(const std::vector<int>& v, int loop, int rel, int relTick, int n, std::vector<int>& out) {
  out.assign(n,0);
  int len=(int)v.size();
  if (len==0) return;
  if (loop<0) loop=len;
  if (rel<0) rel=255;
  int pos=0, val=v[0];
  bool released=false, has=true;
  for (int t=0; t<n; t++) {
    if (t==relTick) released=true;
    if (released && pos<rel && rel<len) pos=rel;
    if (has) {
      val=v[pos++];
      if (pos>rel && !released) {
        if (loop<len && loop<rel) {
          pos=loop;
        } else {
          pos--;
        }
      }
      if (pos>=len) {
        if (loop<len && (loop>=rel || rel>=len)) {
          pos=loop;
        } else {
          has=false;
        }
      }
    }
    out[t]=val;
  }
}

// an envelope with a release: attack, a steady sustain, then a fall that
// starts when the note is let go (the length of the sustain is the length
// of the note). finds the sustain (the longest steady run followed by a
// lower tail). returns false if the volume doesn't look like that.
bool vgmFindADSR(const std::vector<int>& v, int& aLen, int& sEnd) {
  int n=(int)v.size();
  int bestLen=0;
  aLen=-1;
  sEnd=-1;
  int i=0;
  while (i<n) {
    int j=i;
    while (j+1<n && v[j+1]==v[i]) j++;
    int runLen=j-i+1;
    // a tail must follow, and fall
    // the sustain has to be the plateau of the note (a quarter of it at
    // least), not a step of a decay that happens to repeat a value
    if (v[i]>0 && runLen>=3 && runLen*4>=n && j+1<n && runLen>=bestLen) {
      bool falls=true;
      for (int k=j+1; k<n; k++) if (v[k]>v[i]) falls=false;
      if (falls) {
        bestLen=runLen;
        aLen=i;
        sEnd=j;
      }
    }
    i=j+1;
  }
  return aLen>=0;
}

bool vgmAllZero(const std::vector<int>& v) {
  for (int i: v) if (i!=0) return false;
  return true;
}

String vgmMacroKey(const std::vector<int>& v) {
  String r;
  for (int i: v) r+=fmt::sprintf("%d,",i);
  return r+";";
}


// a log of APU activity, from a VGM or from running an NSF
struct VGMNESLog {
  std::vector<VGMEvent> events;
  std::vector<VGMRAMBlock> ramBlocks;
  std::vector<VGMDACWrite> dacWrites;
  double endTime=0;
  double loopTime=-1;
  double clock=1789773.0;
  bool pal=false;
  unsigned int headerRate=0;
  // known tick length and first tick (NSF: the play routine rate)
  double fixedPeriod=0;
  double fixedT0=0;
  // Sunsoft 5B writes (events with reg 0x100+register) and its channels
  bool has5B=false;
  // Konami VRC6: events with reg 0x500 + channel*4 + register ($9000-$9003,
  // $A000-$A002, $B000-$B002)
  bool hasVRC6=false;
  // 2A03 present (a VGM may have only the SN76489)
  bool hasNES=true;
  // SN76489 (Master System/Game Gear PSG): events with reg 0x200 (a byte
  // written to the chip) and 0x201 (Game Gear stereo)
  bool hasSN=false;
  bool snStereo=false;
  double snClock=3579545.0;
  double snNoiseDivider=64.0;
  // YM2413 (OPLL) or VRC7: events with reg 0x300+register
  bool hasOPLL=false;
  bool opllVRC7=false;
  bool opllDrums=false; // rhythm mode is used at some point
  double opllClock=3579545.0;
  // SID (C64): events with reg 0x400+register
  bool hasSID=false;
  bool sid8580=false;
  bool sidDigi=false; // samples through the volume register (left out)
  double sidClock=985248.0;
  // YM2612 (OPN2): events with reg 0x600 + port*0x100 + register (the DAC
  // register 0x2A is left out: its writes are in opnDac)
  bool hasOPN=false;
  // the file turns on channel 3's special mode (one frequency per operator):
  // the song uses Furnace's "Extended Channel 3", FM 3 as four channels
  bool opnExt=false;
  double opnClock=7670453.0;
  std::vector<VGMOPNDACWrite> opnDac;
  // 32X PWM: its writes, and its cycle register (the output goes from 0 to
  // the cycle: the middle is silence). it becomes a Generic PCM DAC
  bool hasPWM=false;
  // (stereo: its left and right outputs play different things, and go on
  // two channels. rate: the hardware's, from the cycle register, 0 if unknown)
  bool pwmStereo=false;
  double pwmRate=0;
  int pwmCycle=1047;
  std::vector<VGMPWMWrite> pwm;
  // SNES (from an SPC): events with reg 0x800+DSP register, 0x900+voice
  // (the sample the voice starts at its key on) and 0xA00+voice (the level
  // of its envelope, ENVX, read from the DSP as it changes). snesInit: the DSP's
  // registers when the song starts (for the echo and the main volume)
  bool hasSNES=false;
  unsigned char snesInit[128];
  // SPC: the largest value the DSP's sums reached before clipping, and the
  // factor the voices' volumes go up by (the main and echo volumes go down
  // by it: the same sound, with volumes that use more of their range)
  int snesPeak=0;
  int snesVolGain=1;
  std::vector<VGMSNESSample> snesSamples;
  // Ricoh RF5C68/RF5C164 (Sega CD PCM): events with reg 0x800|(channel<<4)|
  // register (0-6), 0x880 (register 8, the key offs), 0x881 (the chip on),
  // 0x900+channel (the sample of a key on) and 0xA00+channel (silent: only
  // end markers left). the samples go in snesSamples (raw bytes, loop
  // position). it uses the SNES's channels (44-51)
  bool hasRF=false;
  double rfClock=12500000.0;
  // Game Boy (DMG): events with reg 0xC00+address ($10-$3F: the sound
  // registers and the wave RAM). gbStereo: the song pans some channel
  bool hasGB=false;
  bool gbStereo=false;
};

// the macros of one note (an instrument candidate)
struct VGMMacroSet {
  int type; // 0: pulse, 2: triangle, 3: noise, 4: Sunsoft 5B, 5/6: SN76489, 7: OPLL, 8: OPLL drums, 9: SID, 10/11: VRC6, 12: YM2612
  int fm=-1; // OPLL: the FM sound (an instrument only plays one)
  // OPLL: custom patch fields that change during the note (FM macros):
  // (field, values) and the loop of each, and all of it as a key
  std::vector<std::pair<int,std::vector<int>>> fmMac;
  std::vector<int> fmMacLoop;
  String fmKey;
  // vol, duty (5B: mixer), arp, pitch, 5B noise frequency, 5B envelope shape
  std::vector<int> v[6];
  int loop[6];
  int rel[6]={-1,-1,-1,-1,-1,-1}; // release point (the volume of an ADSR shape)
  bool has[6];
  bool anyRelease=false; // some note of it triggers the release
  // (release tick, ticks heard) of every note using it
  std::vector<std::pair<int,int>> heard;
  int uses;
  int weight; // ticks played with this shape
  int maxObs; // longest stretch of it that a note actually hears
  bool reshaped=false; // extended by a longer note (its key no longer describes it)
  int firstUse;
  int ins;
  String key;
};

// what the subsongs of one import share (instruments, samples, kits)
struct VGMNESContext {
  std::map<String,int> insByKey;
  std::map<String,int> pcmInsMap;
  std::map<String,int> dpcmSampleMap;
  std::map<String,int> pcmSampleMap;
  std::map<String,std::pair<int,int>> kitSlots;
  std::vector<int> kits;
  int kitFill=0;
  int pcmCount=0;
  int instrumentsOver=0;
  std::map<String,int> nameCount;
  // instruments made by earlier subsongs, as merge targets
  std::vector<VGMMacroSet> prevSets;
  // most new instruments one subsong may make (0: no limit)
  int shareLimit=0;
  // the channels each instrument plays on (for its name)
  std::map<int,std::vector<String>> insChans;
  // pitch mode of the song (-1: not decided yet, 0: linear, 1: periods).
  // the first subsong decides, the others follow (it is one song setting)
  int pitchMode=-1;
  // row grid: 0 the usual one, 1 notes on even rows (if that grid is valid)
  int gridMode=0;
  // notes that had to use the closest instrument (instrument limit)
  int mergedNotes=0;
  // the song's tuning was already picked (SID)
  bool tuningSet=false;
  // SID hard restart: -1 not decided, 0 none, else ticks<<17|test<<16|AD<<8|SR
  int sidHR=-1;
  // SID pulse width in the pattern where it runs free (0: never)
  int sidFreePW=1;
  // Game Boy: the wavetables made so far (by their 16 bytes)
  std::map<String,int> gbWaveMap;
};

bool vgmConvertNES(VGMNESLog& log, VGMNESContext& ctx, DivSong& ds, DivSubSong* sub, std::vector<String>& warns, String& err);

// a field of a YM2612 channel patch (32 bytes, see opnPatch), as Furnace's
// instrument has it. 0-43: operator f/11, field f%11 (detune, multiplier, TL,
// rate scaling, attack, AM, decay, sustain rate, sustain level, release,
// SSG-EG); 44: algorithm, 45: feedback, 46: FMS, 47: AMS
static int opnField(const unsigned char* raw, int f) {
  // (Furnace's detune: 3 is none, 0-2 down, 4-6 up)
  static const int dtInv[8]={3,4,5,6,3,2,1,0};
  if (f<44) {
    const unsigned char* o=&raw[(f/11)*7];
    switch (f%11) {
      case 0: return dtInv[(o[0]>>4)&7];
      case 1: return o[0]&15;
      case 2: return o[1]&127;
      case 3: return o[2]>>6;
      case 4: return o[2]&31;
      case 5: return o[3]>>7;
      case 6: return o[3]&31;
      case 7: return o[4]&31;
      case 8: return o[5]>>4;
      case 9: return o[5]&15;
      case 10: return o[6]&15;
    }
    return 0;
  }
  switch (f) {
    case 44: return raw[28]&7;
    case 45: return (raw[28]>>3)&7;
    case 46: return raw[29]&7;
    case 47: return (raw[29]>>4)&3;
  }
  return 0;
}

// converts on a scratch copy of the song with both row grids, and returns the
// one to use: notes on even rows (1) if that needs no more instruments and
// approximates no more notes than the usual grid (0)
static int vgmPickGrid(const VGMNESLog& log, const VGMNESContext& ctx, const DivSong& ds, bool pw=false) {
  int score[2]={-1,-1};
  int ins[2]={0,0};
  int merged[2]={0,0};
  for (int m=0; m<2; m++) {
    DivSong t=ds;
    // deep copies, so instruments shared with earlier tracks can be reused
    // (and renamed) without touching the real ones
    for (DivInstrument*& i: t.ins) i=new DivInstrument(*i);
    size_t smpBefore=t.sample.size();
    t.wave.clear();
    size_t insBefore=t.ins.size();
    t.subsong.clear();
    DivSubSong* sub=new DivSubSong;
    t.subsong.push_back(sub);
    VGMNESLog l=log;
    VGMNESContext c=ctx;
    if (pw) {
      c.sidFreePW=m;
    } else {
      c.gridMode=m;
    }
    c.mergedNotes=0;
    std::vector<String> w;
    String e;
    if (vgmConvertNES(l,c,t,sub,w,e)) {
      ins[m]=(int)(t.ins.size()-insBefore);
      merged[m]=c.mergedNotes;
      score[m]=c.mergedNotes*1000+ins[m];
    }
    for (DivInstrument* i: t.ins) delete i;
    for (size_t i=smpBefore; i<t.sample.size(); i++) delete t.sample[i];
    // (the wavetables it made: only new ones, the list was emptied)
    for (DivWavetable* i: t.wave) delete i;
    t.wave.clear();
    for (DivSubSong* i: t.subsong) {
      i->clearData();
      delete i;
    }
  }
  if (pw) {
    // the pulse width in the pattern is not always exact (a sweep that
    // pauses now and then): only when the instruments don't fit otherwise,
    // or it saves a lot of them
    if (score[1]>=0 && score[0]>=0 && ((merged[0]>0 && score[1]<score[0]) || ins[1]*10<=ins[0]*6)) {
      logI("SID import: pulse width in the pattern (%d instruments, %d otherwise)",ins[1],ins[0]);
      return 1;
    }
    return 0;
  }
  if (score[1]>=0 && score[0]>=0 && score[1]<=score[0]) {
    logI("VGM import: notes on even rows (%d instruments, %d with the usual rows)",ins[1],ins[0]);
    return 1;
  }
  return 0;
}

// turns an APU log into notes, instruments and effects in one subsong
bool vgmConvertNES(VGMNESLog& log, VGMNESContext& ctx, DivSong& ds, DivSubSong* sub, std::vector<String>& warns, String& err) {
    std::vector<VGMEvent>& events=log.events;
    std::vector<VGMRAMBlock>& ramBlocks=log.ramBlocks;
    std::vector<VGMDACWrite>& dacWrites=log.dacWrites;
    double endTime=log.endTime;
    double loopTime=log.loopTime;
    double nesClock=log.clock;
    bool pal=log.pal;
    unsigned int headerRate=log.headerRate;
    // --- step 2: find the driver's tick rate ---
    // writes come in bursts, one per frame. $4011 is left out because PCM
    // streams write it thousands of times per second.
    std::vector<double> bursts;
    for (VGMEvent& e: events) {
      if (e.reg<0) continue;
      // (the SNES's envelope levels are read every 2 ms: not the driver's)
      if (e.reg>=0xa00) continue;
      if (bursts.empty() || e.time-bursts.back()>200.0) {
        bursts.push_back(e.time);
      }
    }
    if (bursts.size()<2 && log.fixedPeriod<=0) {
      err="no NES music was found";
      return false;
    }
    double period=log.fixedPeriod;
    double t0=bursts.empty()?0.0:bursts.front();
    // (a tick that wanders: its writes come around the tick, before or
    // after it, so the first one may be up to half a tick before it)
    bool wanders=false;
    if (log.fixedPeriod>0) {
      t0=log.fixedT0;
    } else {
      std::map<int,int> hist;
      int total=0;
      for (size_t i=1; i<bursts.size(); i++) {
        double d=bursts[i]-bursts[i-1];
        if (d<150.0 || d>4000.0) continue;
        hist[(int)round(d)]++;
        total++;
      }
      // the tick is the longest period that divides (almost) every gap between
      // bursts. drivers that only write on changes leave gaps of several
      // frames (4, 5, 9...), so the most common gap is not always one tick.
      // first look around the NES frame rate (50-60 Hz, plus some margin),
      // then at faster rates for drivers that tick more than once per frame.
      auto scoreP=[&](double P) -> double {
        int hit=0;
        for (auto& h: hist) {
          double g=h.first;
          double steps=round(g/P);
          if (steps<1) continue;
          if (fabs(g-steps*P)<=2.5+0.002*g) hit+=h.second;
        }
        return total?(double)hit/(double)total:0.0;
      };
      for (int range=0; range<2 && period<=0; range++) {
        double pMax=(range==0)?1000.0:600.0;
        double pMin=(range==0)?600.0:150.0;
        double best=0;
        for (double P=pMax; P>=pMin; P-=0.05) best=std::max(best,scoreP(P));
        if (best<0.9) continue;
        for (double P=pMax; P>=pMin; P-=0.05) {
          if (scoreP(P)>=best-0.02) {
            period=P;
            break;
          }
        }
      }
      // drivers whose writes wander a bit around their tick (a couple of
      // milliseconds: the Sega CD's sub CPU, for example): the period that
      // puts the bursts closest to the same phase (the length of the mean
      // vector of their phases), the shortest of the near-best ones
      if (period<=0 && bursts.size()>=32) {
        std::vector<std::pair<double,double>> cand;
        double bestR=0;
        for (double P=150.0; P<=1500.0; P+=0.25) {
          double cs=0, sn=0;
          for (double t: bursts) {
            double a=2.0*M_PI*fmod(t,P)/P;
            cs+=cos(a);
            sn+=sin(a);
          }
          double R=sqrt(cs*cs+sn*sn)/(double)bursts.size();
          cand.push_back(std::make_pair(P,R));
          bestR=std::max(bestR,R);
        }
        if (bestR>=0.5) {
          for (auto& c: cand) {
            if (c.second>=bestR*0.9) {
              // (the best one around the first that is near the best)
              double P=c.first, R=c.second;
              for (auto& d: cand) {
                if (fabs(d.first-c.first)<=c.first*0.01 && d.second>R) {
                  P=d.first;
                  R=d.second;
                }
              }
              period=P;
              double cs=0, sn=0;
              for (double t: bursts) {
                double a=2.0*M_PI*fmod(t,P)/P;
                cs+=cos(a);
                sn+=sin(a);
              }
              double ph=atan2(sn,cs);
              if (ph<0) ph+=2.0*M_PI;
              t0=ph/(2.0*M_PI)*P;
              // (the first tick: the one the first burst is closest to)
              while (t0>bursts.front()+0.5*P) t0-=P;
              wanders=true;
              logI("VGM import: a tick that wanders: %.2f samples, %.0f%% in phase",P,R*100.0);
              break;
            }
          }
        }
      }
      if (period<=0) period=(headerRate==50)?882.0:735.0;
      // fit a straight line through the bursts so the grid doesn't drift:
      // number the frames by counting whole frames between bursts, then
      // least squares gives the exact frame length and phase.
      if (bursts.size()>=8) {
        double n=0;
        double sx=0, sy=0, sxx=0, sxy=0;
        int cnt=0;
        for (size_t i=0; i<bursts.size(); i++) {
          if (i>0) {
            double steps=round((bursts[i]-bursts[i-1])/period);
            if (steps<1) steps=1;
            n+=steps;
          }
          sx+=n;
          sy+=bursts[i];
          sxx+=n*n;
          sxy+=n*bursts[i];
          cnt++;
        }
        double den=(double)cnt*sxx-sx*sx;
        if (den>0) {
          double slope=((double)cnt*sxy-sx*sy)/den;
          double icpt=(sy-slope*sx)/(double)cnt;
          if (fabs(slope-period)<period*0.02) {
            period=slope;
            // first frame start, never after the first burst (and not whole
            // frames before it either: those would only be silence)
            if (wanders) {
              // (around the bursts: the closest tick to the first one)
              t0=icpt;
              while (t0>bursts.front()+0.5*period) t0-=period;
              while (t0+period<=bursts.front()+0.5*period) t0+=period;
            } else {
              t0=std::min(icpt,bursts.front());
              while (t0+period<=bursts.front()) t0+=period;
            }
          }
        }
      }
    }
    logI("VGM import: tick %.2f samples (%.3f Hz) from %d bursts, first at %.1f",period,44100.0/period,(int)bursts.size(),t0);
    auto tickOf=[&](double t) -> int {
      int k=(int)ceil((t-t0)/period-0.5);
      return (k<0)?0:k;
    };
    int numTicks=tickOf(endTime)+1;
    if (numTicks<2) numTicks=2;
    // safety: about 30 minutes at 60 Hz
    if (numTicks>120000) {
      numTicks=120000;
      warns.push_back("song is very long and was cut");
    }
    int loopTick=(loopTime>=0)?tickOf(loopTime):-1;
    if (loopTick>=numTicks) loopTick=-1;

    // --- step 3: simulate the 2A03 tick by tick ---
    // channels: 0-4 2A03 (4 is the DMC), 5-7 Sunsoft 5B
    // internal channels: 0-3 2A03, 4 DMC, 5-7 Sunsoft 5B, 8-11 SN76489
    // (8-10 square, 11 noise), 12-20 YM2413/VRC7 FM, 21-25 YM2413 drums
    // (kick, snare, tom, top, hi-hat). patIdx maps them to the song's
    // channels (FM 7-9 and the first three drums share theirs, like in
    // Furnace's "OPLL with drums").
    // 26-28: the three SID voices, 29-31: VRC6 (two pulses and the saw)
    // 32-37: YM2612 FM 1-6, 38: the YM2612 DAC (it plays on FM 6's channel)
    // 39-42: FM 3's operators 1-4 in its special mode (Extended Channel 3);
    // FM 3 itself, outside of that mode, plays on OP1's channel
    // 43: the 32X's PWM, as a Generic PCM DAC (its left output, in stereo)
    // 44-51: the SNES's eight voices
    // 52-55: the Game Boy (pulse 1, pulse 2, wave, noise)
    // 56: the 32X's PWM's right output, in stereo (a second PCM DAC)
    const int NCH=57;
    std::vector<VGMChanTick> chanTicks[NCH];
    for (int c=0; c<NCH; c++) chanTicks[c].resize(numTicks);
    std::vector<int> tonal;
    int patIdx[NCH];
    for (int c=0; c<NCH; c++) patIdx[c]=-1;
    int numChans=0;
    // FM channels: a note is a key on (the chip's envelope)
    auto isFMChan=[](int c) -> bool {
      return (c>=12 && c<29) || (c>=32 && c<38) || (c>=39 && c<43) || (c>=44 && c<52);
    };
    // the YM2612 (its FM channels, and FM 3's operators)
    auto isOPNChan=[](int c) -> bool {
      return (c>=32 && c<38) || (c>=39 && c<43);
    };
    // (index into the YM2612's per channel tables: 0-5 FM, 6-9 operators)
    auto opnIdx=[](int c) -> int {
      return (c<38)?(c-32):(c-33);
    };
    // the YM2612 first (like Furnace's Genesis: FM, then the PSG)
    if (log.hasOPN) {
      for (int c=32; c<38; c++) {
        if (log.opnExt && c==34) {
          // FM 3: its four operators, in the order of Furnace's channels
          for (int o=39; o<43; o++) {
            patIdx[o]=numChans++;
            tonal.push_back(o);
          }
          patIdx[34]=patIdx[39];
          tonal.push_back(34);
          continue;
        }
        patIdx[c]=numChans++;
        tonal.push_back(c);
      }
      patIdx[38]=patIdx[37];
    }
    if (log.hasNES) {
      for (int c=0; c<5; c++) patIdx[c]=numChans++;
      tonal={0,1,2,3};
    }
    if (log.has5B) {
      for (int c=5; c<8; c++) {
        patIdx[c]=numChans++;
        tonal.push_back(c);
      }
    }
    if (log.hasSN) {
      for (int c=8; c<12; c++) {
        patIdx[c]=numChans++;
        tonal.push_back(c);
      }
    }
    const int opllFM=log.opllVRC7?6:9;
    if (log.hasOPLL) {
      int base=numChans;
      for (int c=0; c<opllFM; c++) {
        patIdx[12+c]=base+c;
        tonal.push_back(12+c);
      }
      numChans+=opllFM;
      if (log.opllDrums) {
        for (int d=0; d<5; d++) {
          patIdx[21+d]=base+6+d;
          tonal.push_back(21+d);
        }
        numChans=base+11;
      }
    }
    if (log.hasSID) {
      for (int c=26; c<29; c++) {
        patIdx[c]=numChans++;
        tonal.push_back(c);
      }
    }
    if (log.hasVRC6) {
      for (int c=29; c<32; c++) {
        patIdx[c]=numChans++;
        tonal.push_back(c);
      }
    }
    // (the PWM's channel is the last one, like its chip)
    if (log.hasPWM) {
      patIdx[43]=numChans++;
      if (log.pwmStereo) patIdx[56]=numChans++;
    }
    // (the RF5C68 uses the same internal channels as the SNES)
    if (log.hasSNES || log.hasRF) {
      for (int c=44; c<52; c++) {
        patIdx[c]=numChans++;
        tonal.push_back(c);
      }
    }
    if (log.hasGB) {
      for (int c=52; c<56; c++) {
        patIdx[c]=numChans++;
        tonal.push_back(c);
      }
    }
    // song commands (speed, grooves, loop) go in the first channel
    const int cmdChan=log.hasGB?52:log.hasOPN?32:(log.hasSNES || log.hasRF)?44:(log.hasNES?0:(log.has5B?5:(log.hasSN?8:(log.hasOPLL?12:(log.hasSID?26:29)))));
    unsigned char ayRegs[16];
    memset(ayRegs,0,16);
    ayRegs[7]=0x3f;
    int snTone[3]={0,0,0};
    int snAtt[4]={15,15,15,15};
    int snNoise=0, snLatch=0, snPan=0xff;
    bool snLatchVol=false;
    // YM2413/VRC7 registers, and the FM sounds the channels play: the preset
    // patches (1-15, the patch number itself goes to the wave macro) or the
    // custom patch (0, with its 8 registers), plus the channel's sustain
    // bit. the drums are one sound of their own.
    unsigned char opllRegs[64];
    memset(opllRegs,0,64);
    struct VGMFMSound {
      int inst;
      bool sus;
      bool drum;
      unsigned char cust[8];
      // YM2612: the channel's patch (see opnPatch below)
      bool opn=false;
      unsigned char opnRaw[30];
      // SNES: the sample and the envelope (ADSR1, ADSR2, GAIN)
      bool snes=false;
      int snesSample=-1;
      unsigned char snesEnv[3];
      // RF5C68: the sample (in snesSample)
      bool rf=false;
    };
    std::vector<VGMFMSound> fmSounds;
    std::vector<String> fmSoundKeys;
    std::map<String,int> fmSoundMap;
    auto fmSoundOf=[&](int inst, bool sus, bool drum) -> int {
      VGMFMSound fs;
      fs.inst=drum?16:((inst==0)?0:1);
      fs.sus=drum?false:sus;
      fs.drum=drum;
      memset(fs.cust,0,8);
      if (!drum && inst==0) memcpy(fs.cust,opllRegs,8);
      String k=fmt::sprintf("%d|%d|%d|",fs.inst,fs.sus?1:0,fs.drum?1:0);
      for (int i=0; i<8; i++) k+=fmt::sprintf("%.2x",fs.cust[i]);
      auto it=fmSoundMap.find(k);
      if (it!=fmSoundMap.end()) return it->second;
      fmSounds.push_back(fs);
      fmSoundKeys.push_back(k);
      fmSoundMap[k]=(int)fmSounds.size()-1;
      return (int)fmSounds.size()-1;
    };
    // fnum<<block, the pitch of an FM channel (the same for any block)
    auto opllPitch=[&](int ch) -> int {
      int fnum=opllRegs[0x10+ch]|((opllRegs[0x20+ch]&1)<<8);
      int block=(opllRegs[0x20+ch]>>1)&7;
      return fnum<<block;
    };
    // rhythm mode per tick (for the 18xx switches), and the custom patch
    std::vector<bool> opllRhythm;
    std::vector<unsigned char> opllCust;
    if (log.hasOPLL) {
      opllRhythm.resize(numTicks,false);
      opllCust.resize((size_t)numTicks*8,0);
    }
    // SID registers, and the global part (filter, volume) per tick
    unsigned char sidRegs[32];
    memset(sidRegs,0,32);
    unsigned char vrcRegs[12];
    memset(vrcRegs,0,12);
    std::vector<unsigned char> sidGlobal;
    if (log.hasSID) sidGlobal.resize((size_t)numTicks*4,0);
    // YM2612 registers (two ports), the frequencies the channels took (the
    // high byte is latched until the low byte is written), the keys (the
    // operators that are on), the DAC switch and the LFO.
    // the patch of each channel per tick (opnPatch, 32 bytes): per operator
    // (in register order: +0, +4, +8, +12) $30, $40 (TL), $50, $60, $70, $80,
    // $90; then $B0 (feedback/algorithm), $B4 (AMS/FMS, no panning).
    // the TL of the carriers is relative to the loudest one: that one is the
    // channel's volume (Furnace adds the volume to the carriers' TL)
    unsigned char opnRegs[2][256];
    memset(opnRegs,0,sizeof(opnRegs));
    for (int p=0; p<2; p++) for (int i=0; i<3; i++) opnRegs[p][0xb4+i]=0xc0;
    int opnFreq[6]={0,0,0,0,0,0};
    int opnLatch[2]={0,0};
    int opnKey[6]={0,0,0,0,0,0};
    // channel 3's special mode: the frequencies of operators 1-3 ($A9/$AD,
    // $AA/$AE, $A8/$AC; operator 4 takes the channel's), with their own latch
    int opnFreq3[3]={0,0,0};
    int opnLatch3=0;
    // (the mode, per tick, for the 18xx switches)
    std::vector<bool> opnSpecial;
    if (log.hasOPN && log.opnExt) opnSpecial.resize(numTicks,false);
    bool opnDacEn=false;
    int opnCh3Ticks=0;
    // the LFO: if the song never turns it on, the AM/FMS/AMS settings do
    // nothing (they are left out, so patches that only differ there are one)
    bool opnLfoEver=false;
    if (log.hasOPN) {
      for (VGMEvent& e: events) {
        if (e.reg==0x622 && (e.val&8)) {
          opnLfoEver=true;
          break;
        }
      }
    }
    std::vector<unsigned char> opnPatch[10];
    std::vector<int> opnLFO;
    if (log.hasOPN) {
      for (int i=0; i<(log.opnExt?10:6); i++) opnPatch[i].resize((size_t)numTicks*32,0);
      opnLFO.resize(numTicks,0);
    }
    // SNES: the DSP's registers, the voices' keys (KON starts a note, KOF
    // lets it go), the sample each one started, and their envelope
    // registers per tick (for the gain macro)
    unsigned char snesRegs[128];
    memset(snesRegs,0,128);
    bool snesKeyed[8]={false,false,false,false,false,false,false,false};
    int snesSmpNext[8]={-1,-1,-1,-1,-1,-1,-1,-1};
    int snesSmp[8]={-1,-1,-1,-1,-1,-1,-1,-1};
    std::vector<unsigned char> snesEnvTick[8];
    int snesEnvx[8]={0,0,0,0,0,0,0,0};
    if (log.hasSNES) for (int v=0; v<8; v++) snesEnvTick[v].resize((size_t)numTicks*3,0);
    int snesNoiseTicks=0, snesPmonTicks=0, snesInvTicks=0;
    // the global registers per tick: FLG, EDL, EVOLL, EVOLR, EFB, FIR 0-7,
    // MVOLL, MVOLR, EON
    std::vector<unsigned char> snesGlobal;
    if (log.hasSNES) snesGlobal.resize((size_t)numTicks*16,0);
    // (the main and echo volumes divided by the voices' gain: see snesVolGain)
    auto snesGainDiv=[&](unsigned char b) -> unsigned char {
      if (log.snesVolGain<=1) return b;
      int v=(int)round((double)(signed char)b/(double)log.snesVolGain);
      return (unsigned char)(std::max(-128,std::min(127,v))&0xff);
    };
    // each voice's volume signs per tick (bit 0: left negative, bit 1: right
    // negative; bit 7: the voice plays), for Furnace's invert (14xy)
    std::vector<unsigned char> snesInv;
    if (log.hasSNES) snesInv.resize((size_t)numTicks*8,0);
    std::map<String,int> snesSoundMap;
    // the RF5C68: its channel registers (16 per channel), which channels are
    // on, the chip on, the sample of each note, and the channels left silent
    unsigned char rfRegs[128];
    memset(rfRegs,0,128);
    bool rfOn[8]={false,false,false,false,false,false,false,false};
    bool rfEnable=false;
    bool rfSilent[8]={false,false,false,false,false,false,false,false};
    int rfSmpNext[8]={-1,-1,-1,-1,-1,-1,-1,-1};
    int rfSmp[8]={-1,-1,-1,-1,-1,-1,-1,-1};
    // the carriers of each algorithm (operators in register order)
    static const bool opnCarrier[8][4]={
      {false,false,false,true},
      {false,false,false,true},
      {false,false,false,true},
      {false,false,false,true},
      {false,false,true ,true},
      {false,true ,true ,true},
      {false,true ,true ,true},
      {true ,true ,true ,true},
    };
    auto opnSoundOf=[&](const unsigned char* raw) -> int {
      VGMFMSound fs;
      fs.inst=0;
      fs.sus=false;
      fs.drum=false;
      memset(fs.cust,0,8);
      fs.opn=true;
      memcpy(fs.opnRaw,raw,30);
      static const char hexd[]="0123456789abcdef";
      String k="N|";
      k.reserve(62);
      for (int i=0; i<30; i++) {
        k+=hexd[raw[i]>>4];
        k+=hexd[raw[i]&15];
      }
      auto it=fmSoundMap.find(k);
      if (it!=fmSoundMap.end()) return it->second;
      fmSounds.push_back(fs);
      fmSoundKeys.push_back(k);
      fmSoundMap[k]=(int)fmSounds.size()-1;
      return (int)fmSounds.size()-1;
    };
    // the Game Boy, and its wave RAM as the song's wavetables
    VGMGBApu gbApu;
    auto gbWaveIdx=[&](const unsigned char* w) -> int {
      static const char hexd[]="0123456789abcdef";
      String k;
      for (int i=0; i<16; i++) {
        k+=hexd[w[i]>>4];
        k+=hexd[w[i]&15];
      }
      auto it=ctx.gbWaveMap.find(k);
      if (it!=ctx.gbWaveMap.end()) return it->second;
      DivWavetable* wt=new DivWavetable;
      wt->len=32;
      wt->max=15;
      for (int i=0; i<16; i++) {
        wt->data[i*2]=w[i]>>4;
        wt->data[i*2+1]=w[i]&15;
      }
      int idx=(int)ds.wave.size();
      ds.wave.push_back(wt);
      ctx.gbWaveMap[k]=idx;
      return idx;
    };
    std::vector<VGMDPCMTrigger> triggers;
    std::vector<VGMDPCMStop> stops;
    VGMNESApu* apu=new VGMNESApu;
    apu->pal=pal;
    apu->clock=nesClock;
    apu->triggers=&triggers;
    apu->stops=&stops;
    double cycPerSample=nesClock/44100.0;
    size_t evPos=0;
    for (int k=0; k<numTicks; k++) {
      double snap=t0+((double)k+0.5)*period;
      apu->curTick=k;
      bool restart[4]={false,false,false,false};
      bool envRestart=false;
      bool snNoiseRestart=false;
      bool opllKeyOn[14];
      for (int i=0; i<14; i++) opllKeyOn[i]=false;
      bool sidKeyOn[3]={false,false,false};
      bool opnKeyOn[6]={false,false,false,false,false,false};
      bool snesKeyOn[8]={false,false,false,false,false,false,false,false};
      bool rfKeyOn[8]={false,false,false,false,false,false,false,false};
      // (channel 3's operators 1-4: key bits 4-7)
      bool opnKeyOnOp[4]={false,false,false,false};
      while (evPos<events.size() && events[evPos].time<=snap) {
        VGMEvent& e=events[evPos++];
        if (log.hasGB && e.reg>=0xc00) {
          gbApu.advance(e.time);
          gbApu.write(e.reg&0xff,e.val);
          continue;
        }
        if (log.hasRF && e.reg>=0x800) {
          if (e.reg>=0xa00) {
            rfSilent[e.reg&7]=(e.val!=0);
          } else if (e.reg>=0x900) {
            rfSmpNext[e.reg&7]=e.val;
          } else if (e.reg==0x880) {
            for (int v=0; v<8; v++) {
              bool on=!((e.val>>v)&1);
              if (on && !rfOn[v]) {
                rfKeyOn[v]=true;
                rfSmp[v]=rfSmpNext[v];
                rfSilent[v]=false;
              }
              rfOn[v]=on;
            }
          } else if (e.reg==0x881) {
            rfEnable=e.val;
          } else {
            rfRegs[e.reg&0x7f]=e.val;
          }
          continue;
        }
        if (e.reg>=0xa00) {
          snesEnvx[e.reg&7]=e.val;
          continue;
        }
        if (e.reg>=0x900) {
          // (the sample of the next key on of a voice)
          snesSmpNext[e.reg&7]=e.val;
          continue;
        }
        if (e.reg>=0x800) {
          int r=e.reg&0x7f;
          snesRegs[r]=e.val;
          if (r==0x4c) {
            for (int v=0; v<8; v++) {
              if (!(e.val&(1<<v))) continue;
              snesKeyed[v]=true;
              snesKeyOn[v]=true;
              snesSmp[v]=snesSmpNext[v];
            }
          } else if (r==0x5c) {
            for (int v=0; v<8; v++) if (e.val&(1<<v)) snesKeyed[v]=false;
          }
          continue;
        }
        if (e.reg>=0x600) {
          int port=((e.reg-0x600)>>8)&1;
          int a=e.reg&0xff;
          if (port==0 && a==0x28) {
            // key on/off: channel (0-2, 4-6) and the operators that are on.
            // an operator going from off to on starts a note
            int code=e.val&7;
            if ((code&3)==3) continue;
            int ch=(code&3)+((code&4)?3:0);
            int mask=(e.val>>4)&15;
            if (mask&(~opnKey[ch])) opnKeyOn[ch]=true;
            if (ch==2) {
              for (int o=0; o<4; o++) if ((mask&(1<<o)) && !(opnKey[ch]&(1<<o))) opnKeyOnOp[o]=true;
            }
            opnKey[ch]=mask;
          } else if (port==0 && a==0x2b) {
            opnDacEn=e.val&0x80;
          } else if (a>=0xa4 && a<=0xa6) {
            opnLatch[port]=e.val;
          } else if (a>=0xa0 && a<=0xa2) {
            opnFreq[(a&3)+3*port]=((opnLatch[port]&0x3f)<<8)|e.val;
          } else if (port==0 && a>=0xac && a<=0xae) {
            opnLatch3=e.val;
          } else if (port==0 && a>=0xa8 && a<=0xaa) {
            opnFreq3[a-0xa8]=((opnLatch3&0x3f)<<8)|e.val;
          }
          opnRegs[port][a]=e.val;
          continue;
        }
        if (e.reg>=0x500) {
          vrcRegs[e.reg&15]=e.val;
          continue;
        }
        if (e.reg>=0x400) {
          int a=e.reg&0x1f;
          // the gate going from off to on starts a note (and starts the
          // envelope over), even within one tick
          if ((a==4 || a==11 || a==18) && (e.val&1) && !(sidRegs[a]&1)) sidKeyOn[a/7]=true;
          sidRegs[a]=e.val;
          continue;
        }
        if (e.reg>=0x300) {
          int a=e.reg&0xff;
          if (a>=64) continue;
          // a key that goes from off to on in this tick starts a note, even
          // if it was already on at the last tick (key off + key on)
          if (a>=0x20 && a<=0x28 && (e.val&0x10) && !(opllRegs[a]&0x10)) opllKeyOn[a-0x20]=true;
          if (a==0x0e) {
            for (int d=0; d<5; d++) {
              int bit=0x10>>d;
              if ((e.val&bit) && !(opllRegs[a]&bit)) opllKeyOn[9+d]=true;
            }
          }
          opllRegs[a]=e.val;
          continue;
        }
        if (e.reg==0x200) {
          int b=e.val;
          if (b&0x80) {
            snLatch=(b>>5)&3;
            snLatchVol=b&0x10;
            if (snLatchVol) {
              snAtt[snLatch]=b&15;
            } else if (snLatch<3) {
              snTone[snLatch]=(snTone[snLatch]&0x3f0)|(b&15);
            } else {
              snNoise=b&7;
              snNoiseRestart=true;
            }
          } else {
            if (snLatchVol) {
              snAtt[snLatch]=b&15;
            } else if (snLatch<3) {
              snTone[snLatch]=(snTone[snLatch]&15)|((b&0x3f)<<4);
            } else {
              snNoise=b&7;
              snNoiseRestart=true;
            }
          }
          continue;
        }
        if (e.reg==0x201) {
          snPan=e.val;
          continue;
        }
        if (e.reg>=0x100) {
          ayRegs[e.reg&15]=e.val;
          if ((e.reg&15)==13) envRestart=true;
          continue;
        }
        apu->advance(e.time*cycPerSample);
        if (e.reg<0) {
          VGMRAMBlock& rb=ramBlocks[e.val];
          for (size_t i=0; i<rb.data.size(); i++) apu->ram[(rb.addr+i)&0xffff]=rb.data[i];
        } else {
          int rc=apu->write(e.reg,e.val);
          if (rc>=0) restart[rc]=true;
        }
      }
      apu->advance(snap*cycPerSample);
      VGMChanTick snapOut[4];
      apu->snapshot(snapOut);
      for (int c=0; c<4; c++) {
        snapOut[c].restart=restart[c];
        chanTicks[c][k]=snapOut[c];
      }
      if (log.hasGB) {
        gbApu.advance(snap);
        VGMChanTick gt[4];
        gbApu.snapshot(gt,gbWaveIdx);
        for (int i=0; i<4; i++) chanTicks[52+i][k]=gt[i];
      }
      if (log.hasSN) {
        // Game Gear stereo: bits 0-3 right, 4-7 left. Furnace's pan macro
        // value is 1 (right) + 2 (left)
        for (int i=0; i<4; i++) {
          VGMChanTick t;
          int vol=15-snAtt[i];
          t.noise=((snPan>>i)&1)|(((snPan>>(i+4))&1)<<1);
          if (i<3) {
            t.period=snTone[i];
            t.aud=(vol>0 && snTone[i]>=2);
            t.duty=0;
          } else {
            int rate=snNoise&3;
            t.duty=((snNoise>>2)&1)|((rate==3)?2:0);
            // rates 0-2: a marker the pitch lookup understands; rate 3 takes
            // the period of square 3
            t.period=(rate==3)?snTone[2]:(0x1000|rate);
            t.aud=(vol>0);
            t.restart=snNoiseRestart;
          }
          t.vol=t.aud?vol:0;
          chanTicks[8+i][k]=t;
        }
      }
      if (log.hasOPLL) {
        // rhythm mode. some drivers let go of the drums by writing 0 to
        // register $0E, which turns rhythm mode off for a frame or two: as
        // long as FM 7-9 aren't playing then, that is only a key off
        bool rhythm=(opllRegs[0x0e]&0x20) && !log.opllVRC7;
        if (!rhythm && log.opllDrums) {
          rhythm=true;
          for (int i=6; i<9; i++) if (opllRegs[0x20+i]&0x10) rhythm=false;
        }
        opllRhythm[k]=rhythm;
        memcpy(&opllCust[(size_t)k*8],opllRegs,8);
        for (int i=0; i<opllFM; i++) {
          VGMChanTick t;
          // in rhythm mode FM 7-9 play the drums
          bool key=(opllRegs[0x20+i]&0x10) && !(rhythm && i>=6);
          int inst=opllRegs[0x30+i]>>4;
          t.aud=key;
          t.period=opllPitch(i);
          t.vol=key?(15-(opllRegs[0x30+i]&15)):0;
          t.restart=opllKeyOn[i];
          t.duty=inst;
          t.block=(opllRegs[0x20+i]>>1)&7;
          t.fm=fmSoundOf(inst,opllRegs[0x20+i]&0x20,false);
          chanTicks[12+i][k]=t;
        }
        if (log.opllDrums) {
          // kick, snare, tom, top, hi-hat: their key bit, volume and the FM
          // channel whose pitch they use (Furnace writes the same ones)
          static const int slot[5]={6,7,8,8,7};
          int att[5]={
            opllRegs[0x36]&15,
            opllRegs[0x37]&15,
            opllRegs[0x38]>>4,
            opllRegs[0x38]&15,
            opllRegs[0x37]>>4
          };
          int drumSound=fmSoundOf(16,false,true);
          for (int d=0; d<5; d++) {
            VGMChanTick t;
            bool key=rhythm && (opllRegs[0x0e]&(0x10>>d));
            t.aud=key;
            t.period=opllPitch(slot[d]);
            t.vol=key?(15-att[d]):0;
            t.restart=opllKeyOn[9+d];
            t.block=(opllRegs[0x20+slot[d]]>>1)&7;
            t.fm=drumSound;
            chanTicks[21+d][k]=t;
          }
        }
      }
      if (log.has5B) {
        for (int i=0; i<3; i++) {
          VGMChanTick t;
          bool toneEn=!(ayRegs[7]&(1<<i));
          bool noiseEn=!(ayRegs[7]&(8<<i));
          bool env=ayRegs[8+i]&16;
          int vol=ayRegs[8+i]&15;
          t.aud=(toneEn || noiseEn) && (env || vol>0);
          t.period=ayRegs[i*2]|((ayRegs[i*2+1]&15)<<8);
          t.vol=t.aud?(env?15:vol):0;
          t.duty=(toneEn?1:0)|(noiseEn?2:0)|(env?4:0);
          t.noise=ayRegs[6]&31;
          t.envShape=ayRegs[13]&15;
          t.envPeriod=ayRegs[11]|(ayRegs[12]<<8);
          t.envRestart=envRestart;
          chanTicks[5+i][k]=t;
        }
      }
      if (log.hasSID) {
        for (int i=0; i<3; i++) {
          const unsigned char* r=&sidRegs[i*7];
          VGMChanTick t;
          t.aud=r[4]&1;
          t.period=r[0]|(r[1]<<8);
          t.vol=t.aud?15:0;
          t.duty=r[2]|((r[3]&15)<<8);
          t.restart=sidKeyOn[i];
          t.sidCtrl=r[4]&0xfe;
          t.sidAD=r[5];
          t.sidSR=r[6];
          chanTicks[26+i][k]=t;
        }
        memcpy(&sidGlobal[(size_t)k*4],&sidRegs[0x15],4);
      }
      if (log.hasVRC6) {
        // ($9003 bit 0 stops all three)
        bool halt=vrcRegs[3]&1;
        for (int i=0; i<3; i++) {
          const unsigned char* r=&vrcRegs[i*4];
          VGMChanTick t;
          t.period=r[1]|((r[2]&15)<<8);
          if (i<2) {
            // (bit 7: the output is the volume itself, no square wave)
            t.aud=(r[2]&0x80) && !halt && !(r[0]&0x80) && (r[0]&15);
            t.vol=t.aud?(r[0]&15):0;
            t.duty=(r[0]>>4)&7;
          } else {
            t.aud=(r[2]&0x80) && !halt && (r[0]&63);
            t.vol=t.aud?(r[0]&63):0;
          }
          chanTicks[29+i][k]=t;
        }
      }
      if (log.hasOPN) {
        opnLFO[k]=opnRegs[0][0x22]&15;
        const bool special=(opnRegs[0][0x27]&0xc0)!=0;
        if (special && !log.opnExt) opnCh3Ticks++;
        if (log.opnExt) opnSpecial[k]=special;
        for (int ch=0; ch<6; ch++) {
          const unsigned char* r=opnRegs[ch/3];
          const int cc=ch%3;
          unsigned char* raw=&opnPatch[ch][(size_t)k*32];
          for (int o=0; o<4; o++) {
            const int ro=cc+4*o;
            raw[o*7+0]=r[0x30+ro]&0x7f;
            raw[o*7+1]=r[0x40+ro]&0x7f;
            raw[o*7+2]=r[0x50+ro]&0xdf;
            raw[o*7+3]=r[0x60+ro]&0x9f;
            raw[o*7+4]=r[0x70+ro]&0x1f;
            raw[o*7+5]=r[0x80+ro];
            raw[o*7+6]=r[0x90+ro]&0x0f;
          }
          raw[28]=r[0xb0+cc]&0x3f;
          raw[29]=r[0xb4+cc]&0x37;
          if (!opnLfoEver) {
            raw[29]=0;
            for (int o=0; o<4; o++) raw[o*7+3]&=0x1f;
          }
          // the volume: the loudest carrier
          const int alg=raw[28]&7;
          int minTL=127;
          for (int o=0; o<4; o++) if (opnCarrier[alg][o]) minTL=std::min(minTL,(int)raw[o*7+1]);
          for (int o=0; o<4; o++) if (opnCarrier[alg][o]) raw[o*7+1]-=minTL;
          VGMChanTick t;
          // (FM 6 is silent while the DAC is on)
          bool key=(opnKey[ch]!=0) && !(ch==5 && opnDacEn);
          int fnum=opnFreq[ch]&0x7ff;
          int block=(opnFreq[ch]>>11)&7;
          t.aud=key;
          t.period=fnum<<block;
          t.vol=key?(127-minTL):0;
          t.restart=opnKeyOn[ch];
          t.block=block;
          t.noise=(r[0xb4+cc]>>6)&3;
          t.fm=opnSoundOf(raw);
          // (FM 3 in the special mode is its four operators)
          if (ch==2 && log.opnExt && special) {
            t.aud=false;
            t.vol=0;
            t.restart=false;
          }
          chanTicks[32+ch][k]=t;
        }
        if (log.opnExt) {
          // FM 3's operators, each a channel of its own (in Furnace's order:
          // OP1-OP4 are the register slots 0, 2, 1, 3). each one's instrument
          // is its operator (in all four places, so any operator channel can
          // play it), the algorithm and, on OP1 (the only one Furnace takes it
          // from), the feedback. its volume is its own TL
          static const int opSlot[4]={0,2,1,3};
          // (frequency: operators 1-3 their own registers, 4 the channel's)
          const int opFreqOf[4]={opnFreq3[1],opnFreq3[2],opnFreq3[0],opnFreq[2]};
          const unsigned char* r=opnRegs[0];
          const unsigned char* chRaw=&opnPatch[2][(size_t)k*32];
          for (int o=0; o<4; o++) {
            const int sl=opSlot[o];
            unsigned char* raw=&opnPatch[6+o][(size_t)k*32];
            const int ro=2+4*sl;
            unsigned char opb[7];
            opb[0]=r[0x30+ro]&0x7f;
            opb[1]=0;
            opb[2]=r[0x50+ro]&0xdf;
            opb[3]=r[0x60+ro]&0x9f;
            opb[4]=r[0x70+ro]&0x1f;
            opb[5]=r[0x80+ro];
            opb[6]=r[0x90+ro]&0x0f;
            if (!opnLfoEver) opb[3]&=0x1f;
            for (int q=0; q<4; q++) memcpy(&raw[q*7],opb,7);
            raw[28]=(o==0)?chRaw[28]:(chRaw[28]&7);
            raw[29]=chRaw[29];
            VGMChanTick t;
            bool key=special && (opnKey[2]&(1<<o));
            int fnum=opFreqOf[o]&0x7ff;
            int block=(opFreqOf[o]>>11)&7;
            t.aud=key;
            t.period=fnum<<block;
            t.vol=key?(127-(r[0x40+ro]&0x7f)):0;
            t.restart=special && opnKeyOnOp[o];
            t.block=block;
            t.noise=(r[0xb6]>>6)&3;
            t.fm=opnSoundOf(raw);
            chanTicks[39+o][k]=t;
          }
        }
      }
      if (log.hasSNES) {
        // (KOF held lets the voices go)
        for (int v=0; v<8; v++) if (snesRegs[0x5c]&(1<<v)) snesKeyed[v]=false;
        if (snesRegs[0x3d]) snesNoiseTicks++;
        {
          unsigned char* gl=&snesGlobal[(size_t)k*16];
          gl[0]=snesRegs[0x6c];
          gl[1]=snesRegs[0x7d];
          gl[2]=snesGainDiv(snesRegs[0x2c]);
          gl[3]=snesGainDiv(snesRegs[0x3c]);
          gl[4]=snesRegs[0x0d];
          for (int i=0; i<8; i++) gl[5+i]=snesRegs[(i<<4)|0x0f];
          gl[13]=snesGainDiv(snesRegs[0x0c]);
          gl[14]=snesGainDiv(snesRegs[0x1c]);
          gl[15]=snesRegs[0x4d];
        }
        if (snesRegs[0x2d]&0xfe) snesPmonTicks++;
        for (int v=0; v<8; v++) {
          const unsigned char* r=&snesRegs[v<<4];
          VGMChanTick t;
          int vl=std::max(-128,std::min(127,(int)(signed char)r[0]*log.snesVolGain));
          int vr=std::max(-128,std::min(127,(int)(signed char)r[1]*log.snesVolGain));
          if ((vl<0 || vr<0) && snesKeyed[v]) snesInvTicks++;
          snesInv[(size_t)k*8+v]=(vl<0?1:0)|(vr<0?2:0)|((snesKeyed[v] && snesSmp[v]>=0)?0x80:0);
          vl=std::min(127,abs(vl));
          vr=std::min(127,abs(vr));
          int vol=std::max(vl,vr);
          // (the panning that makes Furnace write these volumes:
          // VOL = vol*pan/127, rounded down)
          int pl=127, pr=127;
          if (vol>0) {
            pl=std::min(127,(vl*127+vol-1)/vol);
            pr=std::min(127,(vr*127+vol-1)/vol);
          }
          t.aud=snesKeyed[v] && snesSmp[v]>=0;
          t.period=r[2]|((r[3]&0x3f)<<8);
          t.vol=t.aud?vol:0;
          t.restart=snesKeyOn[v];
          t.noise=(pl<<8)|pr;
          t.envx=snesEnvx[v];
          // the sound: the sample, and the envelope (ADSR, or the GAIN it
          // starts with)
          const unsigned char a1=r[5], a2=r[6], g=r[7];
          String key=fmt::sprintf("Q|%d|%.2x|",snesSmp[v],a1);
          if (a1&0x80) {
            key+=fmt::sprintf("%.2x",a2);
          } else {
            key+=fmt::sprintf("g%.2x",g);
          }
          auto it=snesSoundMap.find(key);
          if (it!=snesSoundMap.end()) {
            t.fm=it->second;
          } else {
            VGMFMSound fs;
            fs.inst=0;
            fs.sus=false;
            fs.drum=false;
            memset(fs.cust,0,8);
            fs.snes=true;
            fs.snesSample=snesSmp[v];
            fs.snesEnv[0]=a1;
            fs.snesEnv[1]=a2;
            fs.snesEnv[2]=g;
            fmSounds.push_back(fs);
            fmSoundKeys.push_back(key);
            t.fm=(int)fmSounds.size()-1;
            snesSoundMap[key]=t.fm;
          }
          snesEnvTick[v][(size_t)k*3]=a1;
          snesEnvTick[v][(size_t)k*3+1]=a2;
          snesEnvTick[v][(size_t)k*3+2]=g;
          chanTicks[44+v][k]=t;
        }
      }
      if (log.hasRF) {
        for (int v=0; v<8; v++) {
          const unsigned char* r=&rfRegs[v<<4];
          VGMChanTick t;
          // (a note that ends in the tick it starts in still plays)
          t.aud=rfOn[v] && rfEnable && rfSmp[v]>=0 && (!rfSilent[v] || rfKeyOn[v]);
          t.period=r[2]|(r[3]<<8);
          t.vol=t.aud?r[0]:0;
          t.restart=rfKeyOn[v];
          t.noise=r[1];
          String key=fmt::sprintf("R|%d",rfSmp[v]);
          auto it=snesSoundMap.find(key);
          if (it!=snesSoundMap.end()) {
            t.fm=it->second;
          } else {
            VGMFMSound fs;
            fs.inst=0;
            fs.sus=false;
            fs.drum=false;
            memset(fs.cust,0,8);
            fs.rf=true;
            fs.snesSample=rfSmp[v];
            memset(fs.snesEnv,0,3);
            fmSounds.push_back(fs);
            fmSoundKeys.push_back(key);
            t.fm=(int)fmSounds.size()-1;
            snesSoundMap[key]=t.fm;
          }
          chanTicks[44+v][k]=t;
        }
      }
    }
    delete apu;

    // SID hard restart: many players turn the gate off and set the envelope
    // to fixed values a tick or two before every note, so it starts clean.
    // Furnace's C64 does the same by itself (envelope reset time and values):
    // if most notes have it, those ticks are left to Furnace and don't go
    // into the instruments
    int hrN=0;
    std::vector<char> sidHR[3];
    if (log.hasSID) {
      if (ctx.sidHR<0) {
        std::map<int,int> cnt;
        int eligible=0;
        for (int v=0; v<3; v++) {
          std::vector<VGMChanTick>& ct=chanTicks[26+v];
          int lastOn=-1;
          for (int b=1; b<numTicks; b++) {
            if (!(ct[b].aud && (ct[b].restart || !ct[b-1].aud))) continue;
            if (lastOn>=0 && b-lastOn>=5) {
              eligible++;
              for (int N=1; N<=3; N++) {
                bool ok=true;
                for (int t=b-N; t<b && ok; t++) {
                  if (ct[t].aud || ct[t].sidAD!=ct[b-N].sidAD || ct[t].sidSR!=ct[b-N].sidSR) ok=false;
                }
                if (!ok) break;
                const VGMChanTick& pt=ct[b-N-1];
                if (pt.aud || pt.sidAD!=ct[b-N].sidAD || pt.sidSR!=ct[b-N].sidSR) {
                  cnt[(N<<17)|((ct[b-N].sidCtrl&8)<<13)|(ct[b-N].sidAD<<8)|ct[b-N].sidSR]++;
                  break;
                }
              }
            }
            lastOn=b;
          }
        }
        int best=-1, bestCnt=0;
        for (auto& i: cnt) {
          if (i.second>bestCnt) {
            bestCnt=i.second;
            best=i.first;
          }
        }
        ctx.sidHR=0;
        if (best>=0 && eligible>=4 && bestCnt*2>=eligible) {
          ctx.sidHR=best;
          logI("SID import: hard restart %d ticks before the notes (AD %.2x SR %.2x%s) in %d of %d notes",best>>17,(best>>8)&0xff,best&0xff,(best&(1<<16))?", test bit":"",bestCnt,eligible);
        }
      }
      if (ctx.sidHR>0) {
        int hr=ctx.sidHR;
        hrN=hr>>17;
        ds.systemFlags[0].set("initResetTime",hrN);
        ds.systemFlags[0].set("testAttack",(hr>>12)&15);
        ds.systemFlags[0].set("testDecay",(hr>>8)&15);
        ds.systemFlags[0].set("testSustain",(hr>>4)&15);
        ds.systemFlags[0].set("testRelease",hr&15);
        // the onsets that have it
        for (int v=0; v<3; v++) {
          std::vector<VGMChanTick>& ct=chanTicks[26+v];
          sidHR[v].assign(numTicks,0);
          for (int b=hrN+1; b<numTicks; b++) {
            if (!(ct[b].aud && (ct[b].restart || !ct[b-1].aud))) continue;
            bool ok=true;
            for (int t=b-hrN; t<b && ok; t++) {
              if (ct[t].aud || ct[t].sidAD!=((hr>>8)&0xff) || ct[t].sidSR!=(hr&0xff)) ok=false;
            }
            if (ok) sidHR[v][b]=1;
          }
        }
      }
    }

    // --- step 4: PCM streams and lone $4011 writes ---
    struct PCMStream {
      int tick;
      double start, end;
      std::vector<VGMDACWrite> w;
    };
    std::vector<PCMStream> pcmStreams;
    std::vector<VGMDACWrite> loneDAC;
    {
      size_t i=0;
      while (i<dacWrites.size()) {
        size_t j=i+1;
        while (j<dacWrites.size() && dacWrites[j].time-dacWrites[j-1].time<=132.0) j++;
        size_t count=j-i;
        double dur=dacWrites[j-1].time-dacWrites[i].time;
        if (count>=32 && dur>0 && (double)(count-1)*44100.0/dur>=2000.0) {
          PCMStream s;
          s.start=dacWrites[i].time;
          s.end=dacWrites[j-1].time;
          s.tick=tickOf(s.start);
          s.w.assign(dacWrites.begin()+i,dacWrites.begin()+j);
          pcmStreams.push_back(s);
        } else {
          for (size_t k=i; k<j; k++) loneDAC.push_back(dacWrites[k]);
        }
        i=j;
      }
    }

    // YM2612 DAC: the writes become samples.
    // - writes while the DAC is off (register $2B) are not heard: left out.
    // - runs of writes close together are samples. a run ends at a pause, where
    //   the data jumps to another place in the PCM bank (a new sample without a
    //   pause), and where the value stays the same for a while (drivers that
    //   keep writing the center value between samples).
    // - the rate: the stream's, or the writes' own. if the writes are evenly
    //   spaced, each one is a sample point; if not (a driver that pauses now
    //   and then), the sample is taken from the writes' times at the usual rate
    struct OPNDACStream {
      int tick;
      double start, end;
      double rate;
      bool regular;
      std::vector<VGMOPNDACWrite> w;
    };
    std::vector<OPNDACStream> opnDacStreams;
    int opnDacOff=0, opnDacEmpty=0;
    if (log.hasOPN) {
      // the DAC switch over time (if the file never sets it, it is on)
      std::vector<std::pair<double,bool>> dacSw;
      for (VGMEvent& e: events) if (e.reg==0x62b) dacSw.push_back(std::make_pair(e.time,(e.val&0x80)!=0));
      auto dacOnAt=[&](double t) -> bool {
        if (dacSw.empty()) return true;
        auto it=std::upper_bound(dacSw.begin(),dacSw.end(),t,[](double x, const std::pair<double,bool>& y) {
          return x<y.first;
        });
        if (it==dacSw.begin()) return false;
        return (it-1)->second;
      };
      std::vector<VGMOPNDACWrite> dw;
      dw.reserve(log.opnDac.size());
      for (VGMOPNDACWrite& w: log.opnDac) {
        if (dacOnAt(w.time)) {
          dw.push_back(w);
        } else {
          opnDacOff++;
        }
      }
      // (a value held this long is silence: 30 ms)
      const double flatLimit=44100.0*0.03;
      auto addStream=[&](size_t i, size_t j) {
        // [i, j)
        if (j<=i) return;
        // the silence at its ends (the value holding still) is not part of
        // it: it starts where the sound starts (the last still value is kept
        // as its first one) and ends where it stops
        while (i+1<j && dw[i+1].val==dw[i].val) i++;
        while (j-1>i && dw[j-2].val==dw[j-1].val) j--;
        size_t count=j-i;
        // (a few writes are not a sample: a driver setting the DAC before it
        // starts one, or what is left of the last one where the rate
        // changes. they would only end up joined to the next one)
        if (count<64) {
          opnDacEmpty++;
          return;
        }
        // an empty sample (silence, or a change too small to hear) is left
        // out, with its note
        int vMin=255, vMax=0;
        for (size_t k=i; k<j; k++) {
          vMin=std::min(vMin,dw[k].val);
          vMax=std::max(vMax,dw[k].val);
        }
        if (vMax-vMin<=2) {
          opnDacEmpty++;
          return;
        }
        double dur=dw[j-1].time-dw[i].time;
        if (dur<=0) return;
        OPNDACStream s;
        s.start=dw[i].time;
        s.end=dw[j-1].time;
        s.tick=tickOf(s.start);
        s.w.assign(dw.begin()+i,dw.begin()+j);
        // its values one by one, at the driver's rate: pauses in the writes
        // are the driver stopped for a moment (the 68000 taking the bus),
        // not part of the sound, so they don't count (the sample keeps its
        // pitch, whatever pauses each hit had). a stream command knows its
        // rate
        s.regular=true;
        if (dw[i].rate>0) {
          s.rate=dw[i].rate;
        } else {
          std::vector<double> gaps;
          gaps.reserve(j-i);
          for (size_t k=i+1; k<j; k++) gaps.push_back(dw[k].time-dw[k-1].time);
          std::vector<double> sorted=gaps;
          std::nth_element(sorted.begin(),sorted.begin()+sorted.size()/2,sorted.end());
          double med=sorted[sorted.size()/2];
          double lim=med*2.5+1.0;
          double sum=0;
          int n=0;
          for (double g: gaps) {
            if (g>lim) continue;
            sum+=g;
            n++;
          }
          s.rate=(n>0 && sum>0)?(44100.0*(double)n/sum):((double)(count-1)*44100.0/dur);
        }
        if (s.rate<1000.0) s.rate=1000.0;
        if (s.rate>96000.0) s.rate=96000.0;
        opnDacStreams.push_back(s);
      };
      // where samples start in the PCM bank: the first place read after a
      // pause. a jump in the bank to one of those, without a pause, is a new
      // sample cutting the last one; a jump elsewhere is not (the file only
      // keeps a piece of data once, and the rest of a sample may be stored
      // somewhere else, like the quiet tail of a drum)
      std::set<int> startSrc, jumpSrc;
      {
        bool fresh=true;
        for (size_t k=0; k<dw.size(); k++) {
          // (after a pause, unless the driver only goes on where it was)
          if (k>0 && dw[k].time-dw[k-1].time>132.0) {
            bool resumes=(dw[k].src>=0 && dw[k-1].src>=0 && dw[k].src==dw[k-1].src+1 && dw[k].time-dw[k-1].time<=600.0);
            if (!resumes) fresh=true;
          }
          if (fresh && dw[k].src>=0) {
            startSrc.insert(dw[k].src);
            fresh=false;
          }
          // (and, apart, where the driver jumps to in the bank: a sample it
          // starts right after another one, or a piece of one the file
          // keeps elsewhere; the quiet ones are told apart below)
          if (k>0 && dw[k].src>=0 && dw[k-1].src>=0 && dw[k].src!=dw[k-1].src+1) jumpSrc.insert(dw[k].src);
        }
      }
      // a piece of a run, split where the value holds still for long
      auto addPiece=[&](size_t i, size_t j) {
        size_t segStart=i;
        size_t p=i;
        while (p<j) {
          size_t q=p;
          while (q+1<j && dw[q+1].val==dw[p].val) q++;
          if (q>p && dw[q].time-dw[p].time>=flatLimit) {
            addStream(segStart,p+1);
            segStart=q;
          }
          p=q+1;
        }
        addStream(segStart,j);
      };
      // the usual spacing of the writes around a place (median of W gaps)
      const size_t W=24;
      std::vector<double> win(W);
      auto medGap=[&](size_t from) -> double {
        // gaps from..from+W-1 (between writes from-1..from+W-1)
        for (size_t k=0; k<W; k++) win[k]=dw[from+k].time-dw[from+k-1].time;
        std::nth_element(win.begin(),win.begin()+W/2,win.end());
        return win[W/2];
      };
      // a jump in the bank to a place where samples start is a new sample if
      // it starts like a sample does: loud, or much louder than what played
      // right before it (an attack). a piece that goes on at the level of
      // the last one, or quieter, is a part of its fall the file keeps
      // somewhere else (a drum's tail): that stays with it
      auto quietFrom=[&](size_t j) -> bool {
        // (the next 64 values, through the driver's pauses)
        int lo=255, hi=0;
        int ls=dw[j].src-1;
        size_t n=0;
        for (size_t e=j; e<dw.size() && n<64; e++, n++) {
          if (e>j) {
            bool cont=(dw[e].src>=0 && dw[e].src==ls+1);
            if (dw[e].time-dw[e-1].time>132.0 && !(cont && dw[e].time-dw[e-1].time<=600.0)) break;
            if (dw[e].src>=0 && !cont && startSrc.count(dw[e].src)) break;
          }
          if (dw[e].src>=0) ls=dw[e].src;
          lo=std::min(lo,dw[e].val);
          hi=std::max(hi,dw[e].val);
        }
        int after=hi-lo;
        if (after>=64) return false;
        // (the 64 values before it)
        int blo=255, bhi=0;
        for (size_t e=(j>64)?(j-64):0; e<j; e++) {
          blo=std::min(blo,dw[e].val);
          bhi=std::max(bhi,dw[e].val);
        }
        int before=(bhi>=blo)?(bhi-blo):0;
        if (after>=16 && after*2>=before*3+16) return false;
        return true;
      };
      size_t i=0;
      while (i<dw.size()) {
        size_t j=i+1;
        int lastSrc=dw[i].src;
        while (j<dw.size()) {
          // (a stream the file starts: a sample, always)
          if (dw[j].start) break;
          if (dw[j].time-dw[j-1].time>132.0) {
            // (a driver stopped for a moment, that goes on with the same
            // sample right where it was, is still playing it)
            bool resumes=(dw[j].src>=0 && dw[j-1].src>=0 && dw[j].src==dw[j-1].src+1 && dw[j].time-dw[j-1].time<=600.0);
            // (unless that place is where a sample starts: the file keeps
            // the next sample right after this one, and the pause was the
            // end of this one)
            if (resumes && (startSrc.count(dw[j].src) || (jumpSrc.count(dw[j].src) && !quietFrom(j)))) resumes=false;
            if (!resumes) break;
          }
          if (dw[j].src>=0 && dw[j].src!=lastSrc+1) {
            if (startSrc.count(dw[j].src)) break;
            if (jumpSrc.count(dw[j].src) && !quietFrom(j)) break;
          }
          if (dw[j].src>=0) lastSrc=dw[j].src;
          j++;
        }
        // a change in the rate of the writes is a new sample too (a driver
        // that starts another sample at once, at its own rate)
        size_t pieceStart=i;
        size_t k=i+W+1;
        while (k+W<=j) {
          double before=medGap(k-W);
          double after=medGap(k);
          double ratio=(before>after)?(before/std::max(after,0.5)):(after/std::max(before,0.5));
          if (ratio<1.6) {
            k++;
            continue;
          }
          // the exact place: where the change is the biggest nearby
          size_t bestK=k;
          double bestR=ratio;
          for (size_t m=k+1; m<k+W && m+W<=j; m++) {
            double b2=medGap(m-W), a2=medGap(m);
            double r2=(b2>a2)?(b2/std::max(a2,0.5)):(a2/std::max(b2,0.5));
            if (r2>bestR) {
              bestR=r2;
              bestK=m;
            }
          }
          addPiece(pieceStart,bestK);
          pieceStart=bestK;
          k=bestK+W+1;
        }
        addPiece(pieceStart,j);
        i=j;
      }
    }

    // 32X PWM: the left and right outputs become one (the Generic PCM DAC
    // plays mono), and runs of it become samples, like the YM2612's DAC: a
    // stream the file starts is a new one, and so is a pause, or a long
    // silence (the value holding still). the values are 12 bits from 0 to the
    // cycle register, the middle being silence: they become 16-bit samples
    // (in stereo, data is the left output and dataR the right one; the hits
    // are found on both together, so they start and end in the same place)
    struct PWMHit {
      int tick;
      double start, end, rate;
      std::vector<short> data, dataR;
    };
    std::vector<PWMHit> pwmHits;
    int pwmEmpty=0;
    if (log.hasPWM) {
      const int cyc=std::max(3,log.pwmCycle);
      // (a pulse is cycle-1 clocks long, as Furnace's 32X PWM does it: the
      // middle of that is silence)
      const double mid=(double)(cyc-1)*0.5;
      // (val: both outputs together, what the hits are found on; l and r:
      // each one)
      struct PWMFrame {
        double time;
        int val, l, r;
        double rate;
        bool start;
      };
      std::vector<PWMFrame> fr;
      {
        std::vector<VGMPWMWrite>& pw=log.pwm;
        double curL=mid, curR=mid;
        size_t i=0;
        while (i<pw.size()) {
          PWMFrame f;
          f.time=pw[i].time;
          f.rate=0;
          f.start=false;
          size_t j=i;
          // the writes of one output: left and right together. they don't
          // always come at the same moment (two streams a fraction of a
          // sample apart, or a game writing one side and then the other):
          // a side written again starts the next output, and so does a gap
          // of most of an output's time. (otherwise each side would be an
          // output of its own: twice as many values, and the sample would
          // play an octave lower)
          const double outRate=(pw[i].rate>0)?pw[i].rate:((log.pwmRate>0)?log.pwmRate:22000.0);
          const double pairGap=0.75*44100.0/outRate;
          int seen=0;
          while (j<pw.size()) {
            const VGMPWMWrite& w=pw[j];
            const int bits=(w.ch==0)?1:((w.ch==1)?2:3);
            if (j>i && ((seen&bits) || pw[j].time-pw[i].time>=pairGap)) break;
            seen|=bits;
            if (w.ch==0) {
              curL=w.val;
            } else if (w.ch==1) {
              curR=w.val;
            } else {
              curL=w.val;
              curR=w.val;
            }
            if (w.rate>0) f.rate=w.rate;
            if (w.start) f.start=true;
            j++;
          }
          f.val=(int)round((curL+curR)*0.5);
          f.l=(int)round(curL);
          f.r=(int)round(curR);
          fr.push_back(f);
          i=j;
        }
      }
      const int quietRange=std::max(2,cyc/64);
      auto addHit=[&](size_t a, size_t e) {
        // (the silence at its ends is not part of it)
        while (a+1<e && fr[a+1].val==fr[a].val) a++;
        while (e-1>a && fr[e-2].val==fr[e-1].val) e--;
        if (e-a<64) {
          pwmEmpty++;
          return;
        }
        int lo=1<<20, hi=-1;
        for (size_t k=a; k<e; k++) {
          lo=std::min(lo,fr[k].val);
          hi=std::max(hi,fr[k].val);
        }
        if (hi-lo<=quietRange) {
          pwmEmpty++;
          return;
        }
        double dur=fr[e-1].time-fr[a].time;
        if (dur<=0) return;
        PWMHit h;
        h.start=fr[a].time;
        h.tick=tickOf(h.start);
        if (fr[a].rate>0) {
          h.rate=fr[a].rate;
        } else if (log.pwmRate>0 && fabs((double)(e-a-1)*44100.0/dur-log.pwmRate)<log.pwmRate*0.03) {
          // (written directly at about the hardware's rate: that one, exact)
          h.rate=log.pwmRate;
        } else {
          // (the usual spacing, leaving out pauses)
          std::vector<double> gaps;
          for (size_t k=a+1; k<e; k++) gaps.push_back(fr[k].time-fr[k-1].time);
          std::vector<double> srt=gaps;
          std::nth_element(srt.begin(),srt.begin()+srt.size()/2,srt.end());
          double lim=srt[srt.size()/2]*2.5+1.0;
          double sum=0;
          int n=0;
          for (double g: gaps) {
            if (g>lim) continue;
            sum+=g;
            n++;
          }
          h.rate=(n>0 && sum>0)?(44100.0*(double)n/sum):((double)(e-a-1)*44100.0/dur);
        }
        if (h.rate<1000.0) h.rate=1000.0;
        if (h.rate>96000.0) h.rate=96000.0;
        h.end=fr[e-1].time+44100.0/h.rate;
        auto toSample=[&](int val) -> short {
          // (a width past a whole pulse is a whole pulse)
          val=std::max(0,std::min(cyc-1,val));
          double v=((double)val-mid)*32767.0/mid;
          if (v<-32768.0) v=-32768.0;
          if (v>32767.0) v=32767.0;
          return (short)round(v);
        };
        h.data.resize(e-a);
        if (log.pwmStereo) h.dataR.resize(e-a);
        for (size_t k=a; k<e; k++) {
          if (log.pwmStereo) {
            h.data[k-a]=toSample(fr[k].l);
            h.dataR[k-a]=toSample(fr[k].r);
          } else {
            h.data[k-a]=toSample(fr[k].val);
          }
        }
        pwmHits.push_back(h);
      };
      const double flatLimitPWM=44100.0*0.03;
      size_t i=0;
      while (i<fr.size()) {
        size_t j=i+1;
        // (the loop point starts one too: after the loop the song plays it
        // from there)
        while (j<fr.size() && !fr[j].start && fr[j].time-fr[j-1].time<=132.0 && !(loopTime>=0 && fr[j-1].time<loopTime && fr[j].time>=loopTime)) j++;
        // the PWM usually plays what the 32X's processor mixes, without
        // pauses: a new sound (an attack) starts a new sample too, like a
        // note does. the loudness in windows of 10 ms: where it jumps up
        // (to 1.8 times the last two windows), a new sample starts (at most
        // one every 40 ms)
        std::vector<size_t> cuts;
        {
          std::vector<size_t> wStart;
          std::vector<double> wE;
          size_t k=i;
          while (k<j) {
            const double tw=fr[k].time;
            size_t e=k;
            double sum=0;
            int n=0;
            while (e<j && fr[e].time-tw<441.0) {
              double x=(double)fr[e].val-mid;
              sum+=x*x;
              n++;
              e++;
            }
            wStart.push_back(k);
            wE.push_back((n>0)?sqrt(sum/(double)n):0.0);
            k=e;
          }
          const double floorE=(double)cyc/256.0;
          size_t lastCut=0;
          for (size_t w=2; w<wE.size(); w++) {
            double prev=std::max(std::max(wE[w-1],wE[w-2]),floorE);
            if (wE[w]>1.8*prev && w-lastCut>=4) {
              // (on the start of the closest tick: the note plays there,
              // so the samples join without a gap)
              double tc=fr[wStart[w]].time;
              double kt=round((tc-t0)/period);
              double tt=t0+kt*period;
              size_t f=wStart[w];
              while (f>i+1 && fr[f-1].time>=tt) f--;
              while (f<j && fr[f].time<tt) f++;
              if (f>i && f<j && (cuts.empty() || f>cuts.back())) cuts.push_back(f);
              lastCut=w;
            }
          }
        }
        cuts.push_back(j);
        size_t pieceStart=i;
        for (size_t pe: cuts) {
          // (and where the value holds still for long)
          size_t segStart=pieceStart;
          size_t q0=pieceStart;
          while (q0<pe) {
            size_t q=q0;
            while (q+1<pe && fr[q+1].val==fr[q0].val) q++;
            if (q>q0 && fr[q].time-fr[q0].time>=flatLimitPWM) {
              addHit(segStart,q0+1);
              segStart=q;
            }
            q0=q+1;
          }
          addHit(segStart,pe);
          pieceStart=pe;
        }
        i=j;
      }
    }

    // --- step 5: exact pitch, by running Furnace's own period table backwards ---
    DivPitchTable pt;
    pt.init(ds.tuning,nesClock,16,0x800,true,true);
    // for each period, the pitch offset (8.7 fixed point, 128 = 1 semitone) that produces it
    std::vector<int> periodOffset(2048,-1);
    {
      // the table interpolates inside each note, so near note boundaries the
      // offsets of one period may not be contiguous: keep all of them
      std::vector<std::vector<int>> hits(2048);
      for (int off=0; off<180*128; off++) {
        int p=pt.get(off,0,0)-1;
        if (p<0) p=0;
        if (p>2047) p=2047;
        hits[p].push_back(off);
      }
      std::vector<int> lo(2048,-1);
      for (int p=0; p<2048; p++) {
        if (hits[p].empty()) continue;
        lo[p]=hits[p][0];
        // prefer a whole note if one lands on this period, else the middle one
        int best=hits[p][hits[p].size()/2];
        for (int off: hits[p]) {
          if ((off&127)==0) {
            best=off;
            break;
          }
        }
        periodOffset[p]=best;
      }
      // periods the table can't hit exactly: take the closest one that it can
      for (int p=0; p<2048; p++) {
        if (periodOffset[p]>=0) continue;
        for (int d=1; d<2048; d++) {
          if (p-d>=0 && lo[p-d]>=0) { periodOffset[p]=periodOffset[p-d]; break; }
          if (p+d<2048 && lo[p+d]>=0) { periodOffset[p]=periodOffset[p+d]; break; }
        }
        if (periodOffset[p]<0) periodOffset[p]=0;
      }
    }
    // the same for the 5B (CPU clock, like Furnace's "Famicom with Sunsoft 5B";
    // its period register is used as is, without the -1)
    std::vector<int> periodOffset5B;
    // (the tables forward too: what Furnace writes for a pitch offset)
    DivPitchTable fwd5B, fwdSN, fwdOPLL;
    int snThr=0;
    if (log.has5B) {
      DivPitchTable& pt5=fwd5B;
      pt5.init(ds.tuning,pal?((283.75*15625.0+25.0)*3.0/8.0):((315000000.0/88.0)/2.0),16,0xfff,true,true);
      periodOffset5B.assign(4096,-1);
      std::vector<std::vector<int>> hits(4096);
      for (int off=0; off<180*128; off++) {
        int p=pt5.get(off,0,0);
        if (p<0) p=0;
        if (p>4095) p=4095;
        hits[p].push_back(off);
      }
      for (int p=0; p<4096; p++) {
        if (hits[p].empty()) continue;
        int best=hits[p][hits[p].size()/2];
        for (int off: hits[p]) {
          if ((off&127)==0) {
            best=off;
            break;
          }
        }
        periodOffset5B[p]=best;
      }
      for (int p=0; p<4096; p++) {
        if (periodOffset5B[p]>=0) continue;
        for (int d=1; d<4096; d++) {
          if (p-d>=0 && !hits[p-d].empty()) { periodOffset5B[p]=periodOffset5B[p-d]; break; }
          if (p+d<4096 && !hits[p+d].empty()) { periodOffset5B[p]=periodOffset5B[p+d]; break; }
        }
        if (periodOffset5B[p]<0) periodOffset5B[p]=0;
      }
    }
    // and for the SN76489. Furnace uses its pitch table, except above a
    // threshold where each semitone is one period step down to 0 ("easy
    // noise"): that is what reaches the tiny periods (1-15) that drivers use
    // for high noise with square 3's period.
    std::vector<int> periodOffsetSN, periodOffsetSNNoise;
    auto buildSN=[&](double divider, std::vector<int>& out) {
      DivPitchTable ptSN;
      ptSN.init(ds.tuning,log.snClock,divider,0x3ff,true,true);
      int thr=(int)round(128.0*12.0*std::log((log.snClock/(16.0*divider))/(0.0625*ds.tuning))/std::log(2.0))-384+64+60*128;
      if (divider==64.0) {
        snThr=thr;
        fwdSN=ptSN;
      }
      out.assign(1024,-1);
      std::vector<std::vector<int>> hits(1024);
      for (int off=0; off<180*128; off++) {
        int p;
        if (off>thr) {
          p=((16<<7)-(off-thr))>>7;
          if (p<0) p=0;
        } else {
          p=ptSN.get(off,0,0);
        }
        if (p<0) p=0;
        if (p>1023) p=1023;
        hits[p].push_back(off);
      }
      for (int p=0; p<1024; p++) {
        if (hits[p].empty()) continue;
        int best=hits[p][hits[p].size()/2];
        for (int off: hits[p]) {
          if ((off&127)==0) {
            best=off;
            break;
          }
        }
        out[p]=best;
      }
      for (int p=0; p<1024; p++) {
        if (out[p]>=0) continue;
        for (int d=1; d<1024; d++) {
          if (p-d>=0 && !hits[p-d].empty()) { out[p]=out[p-d]; break; }
          if (p+d<1024 && !hits[p+d].empty()) { out[p]=out[p+d]; break; }
        }
        if (out[p]<0) out[p]=0;
      }
    };
    if (log.hasSN) {
      buildSN(64.0,periodOffsetSN);
      if (log.snNoiseDivider!=64.0) {
        buildSN(log.snNoiseDivider,periodOffsetSNNoise);
      } else {
        periodOffsetSNNoise=periodOffsetSN;
      }
    }
    // and for the YM2413/VRC7: Furnace computes a frequency from its table
    // and picks the block itself (frequency/343), so the pitch to match is
    // fnum<<block, whatever block the driver used
    std::vector<int> pitchOffsetOPLL;
    std::vector<bool> opllExact(log.hasOPLL?65536:0,false);
    // with the instrument's fixed block (drivers that use f-nums above 343
    // with a lower block): offsets for that block, made when first needed
    std::vector<int> pitchOffsetBlock[8];
    std::vector<bool> exactBlock[8];
    auto makeBlockTable=[&](int b) {
      if (!pitchOffsetBlock[b].empty()) return;
      DivPitchTable ptB;
      ptB.init(ds.tuning,log.opllClock,1180068,0xffff,false,true);
      std::vector<int>& out=pitchOffsetBlock[b];
      out.assign(512,-1);
      exactBlock[b].assign(512,false);
      for (int off=0; off<180*128; off++) {
        int f=ptB.get(off,0,0);
        if (f<0) f=0;
        if (f>65535) f=65535;
        int fnum=f>>b;
        if (fnum>0x1ff) fnum=0x1ff;
        if (out[fnum]<0 || ((off&127)==0 && (out[fnum]&127)!=0)) out[fnum]=off;
      }
      for (int fn=0; fn<512; fn++) exactBlock[b][fn]=(out[fn]>=0);
      for (int fn=0; fn<512; fn++) {
        if (out[fn]>=0) continue;
        for (int d=1; d<512; d++) {
          if (fn-d>=0 && exactBlock[b][fn-d]) { out[fn]=out[fn-d]; break; }
          if (fn+d<512 && exactBlock[b][fn+d]) { out[fn]=out[fn+d]; break; }
        }
        if (out[fn]<0) out[fn]=0;
      }
    };
    // the fixed block of the note being built (0: Furnace picks the block)
    int curFixedBlock=0;
    if (log.hasOPLL) {
      DivPitchTable& ptO=fwdOPLL;
      ptO.init(ds.tuning,log.opllClock,1180068,0xffff,false,true);
      const int span=1<<16;
      pitchOffsetOPLL.assign(span,-1);
      for (int off=0; off<180*128; off++) {
        int f=ptO.get(off,0,0);
        if (f<0) f=0;
        if (f>65535) f=65535;
        int block=f/343;
        if (block>0) block=bsr(block);
        if (block>7) block=7;
        int fnum=f>>block;
        if (fnum>0x1ff) fnum=0x1ff;
        int L=fnum<<block;
        // a whole note wins
        if ((off&127)==0 || pitchOffsetOPLL[L]<0) {
          if (pitchOffsetOPLL[L]<0 || (pitchOffsetOPLL[L]&127)!=0) pitchOffsetOPLL[L]=off;
        }
      }
      for (int L=0; L<span; L++) opllExact[L]=(pitchOffsetOPLL[L]>=0);
      // pitches the table can't hit: the closest one it can
      std::vector<int> lo(span,-1), hi(span,-1);
      int last=-1;
      for (int L=0; L<span; L++) {
        if (pitchOffsetOPLL[L]>=0) last=L;
        lo[L]=last;
      }
      last=-1;
      for (int L=span-1; L>=0; L--) {
        if (pitchOffsetOPLL[L]>=0) last=L;
        hi[L]=last;
      }
      for (int L=0; L<span; L++) {
        if (pitchOffsetOPLL[L]>=0) continue;
        int pick=-1;
        if (lo[L]<0) {
          pick=hi[L];
        } else if (hi[L]<0) {
          pick=lo[L];
        } else {
          pick=(L-lo[L]<=hi[L]-L)?lo[L]:hi[L];
        }
        pitchOffsetOPLL[L]=(pick>=0)?pitchOffsetOPLL[pick]:0;
      }
    }
    // and for the YM2612. Furnace computes its frequency (linear pitch) like
    // DivEngine::calcFreq does, and picks the block from the note
    // (CONVERT_FNUM_BLOCK) or uses the instrument's fixed block. the pitch to
    // match is fnum<<block, whatever block the driver used
    auto opnFwd=[&](int off, int fixedBlock) -> int {
      double tuning=ds.tuning;
      const double divider=9437184.0;
      const double clock=log.opnClock;
      double fbase=tuning*pow(2.0,(float)(off+384-7680)/(128.0*12.0));
      int bf=round(fbase*(divider/clock));
      int block;
      if (fixedBlock>0) {
        block=fixedBlock-1;
        bf>>=block;
        if (bf<0) bf=0;
        if (bf>2047) bf=2047;
        return bf<<block;
      }
      double tun=tuning;
      if (tun<400.0) tun=400.0;
      if (tun>500.0) tun=500.0;
      int boundaryBottom=tun*pow(2.0,0.25)*(divider/clock);
      int boundaryTop=2.0*tun*pow(2.0,0.25)*(divider/clock);
      while (boundaryTop>2047) {
        boundaryTop>>=1;
        boundaryBottom>>=1;
      }
      block=((off>>7)-60)/12;
      if (block<0) block=0;
      if (block>7) block=7;
      bf>>=block;
      if (bf<0) bf=0;
      while (bf>0 && bf<boundaryBottom && block>0) {
        bf<<=1;
        block--;
      }
      if (bf>boundaryTop) {
        while (block<7 && bf>boundaryTop) {
          bf>>=1;
          block++;
        }
        if (bf>2047) bf=2047;
      }
      return bf<<block;
    };
    const int opnSpan=1<<18;
    std::vector<int> pitchOffsetOPN;
    std::vector<bool> opnExact;
    // with the instrument's fixed block, made when first needed
    std::vector<int> pitchOffsetOPNBlock[8];
    std::vector<bool> opnExactBlock[8];
    auto makeOPNBlockTable=[&](int b) {
      if (!pitchOffsetOPNBlock[b].empty()) return;
      std::vector<int>& out=pitchOffsetOPNBlock[b];
      out.assign(2048,-1);
      opnExactBlock[b].assign(2048,false);
      for (int off=0; off<180*128; off++) {
        int fnum=opnFwd(off,b+1)>>b;
        if (out[fnum]<0 || ((off&127)==0 && (out[fnum]&127)!=0)) out[fnum]=off;
      }
      for (int fn=0; fn<2048; fn++) opnExactBlock[b][fn]=(out[fn]>=0);
      for (int fn=0; fn<2048; fn++) {
        if (out[fn]>=0) continue;
        for (int d=1; d<2048; d++) {
          if (fn-d>=0 && opnExactBlock[b][fn-d]) { out[fn]=out[fn-d]; break; }
          if (fn+d<2048 && opnExactBlock[b][fn+d]) { out[fn]=out[fn+d]; break; }
        }
        if (out[fn]<0) out[fn]=0;
      }
    };
    if (log.hasOPN) {
      pitchOffsetOPN.assign(opnSpan,-1);
      for (int off=0; off<180*128; off++) {
        int L=opnFwd(off,0);
        if (L<0 || L>=opnSpan) continue;
        // a whole note wins
        if (pitchOffsetOPN[L]<0 || ((off&127)==0 && (pitchOffsetOPN[L]&127)!=0)) pitchOffsetOPN[L]=off;
      }
      opnExact.assign(opnSpan,false);
      for (int L=0; L<opnSpan; L++) opnExact[L]=(pitchOffsetOPN[L]>=0);
      // pitches the table can't hit: the closest one it can
      int last=-1;
      std::vector<int> lo(opnSpan,-1);
      for (int L=0; L<opnSpan; L++) {
        if (opnExact[L]) last=L;
        lo[L]=last;
      }
      last=-1;
      std::vector<bool> close(opnSpan,false);
      for (int L=opnSpan-1; L>=0; L--) {
        if (opnExact[L]) last=L;
        if (opnExact[L]) continue;
        int h=last, l=lo[L];
        int pick=(l<0)?h:((h<0)?l:((L-l<=h-L)?l:h));
        pitchOffsetOPN[L]=(pick>=0)?pitchOffsetOPN[pick]:0;
        // drivers that use a lower block and a bigger f-num than Furnace
        // have a bit more precision; the closest pitch Furnace makes is a
        // few cents away at most. that's close enough: a fixed block would
        // need one instrument per octave. (0.3%: about 5 cents)
        if (pick>=0 && (long long)abs(pick-L)*1000<=3LL*(long long)L) close[L]=true;
      }
      for (int L=0; L<opnSpan; L++) if (close[L]) opnExact[L]=true;
    }
    // and for the SNES: Furnace's pitch table for a sample at 32000 Hz (C-4
    // plays it at its rate: P=0x1000), backwards
    std::vector<int> pitchOffsetSNES;
    DivPitchTable fwdSNES;
    if (log.hasSNES) {
      fwdSNES.init(ds.tuning,1024000.0,131072.0*32000.0/8372.0,0x3fff,false,true);
      pitchOffsetSNES.assign(16384,-1);
      for (int off=0; off<180*128; off++) {
        int f=fwdSNES.get(off,0,0);
        if (f<0) f=0;
        if (f>0x3fff) f=0x3fff;
        if (pitchOffsetSNES[f]<0 || ((off&127)==0 && (pitchOffsetSNES[f]&127)!=0)) pitchOffsetSNES[f]=off;
      }
      // (the ones it can't make: the closest one it can)
      for (int P=0; P<16384; P++) {
        if (pitchOffsetSNES[P]>=0) continue;
        int lo=P, hi=P;
        while (lo>=0 && pitchOffsetSNES[lo]<0) lo--;
        while (hi<16384 && pitchOffsetSNES[hi]<0) hi++;
        int pick=(lo<0)?hi:((hi>=16384)?lo:((P-lo<=hi-P)?lo:hi));
        pitchOffsetSNES[P]=(pick>=0 && pick<16384 && pitchOffsetSNES[pick]>=0)?pitchOffsetSNES[pick]:0;
      }
    }
    // and for the RF5C68: Furnace's pitch table for a sample at the chip's
    // rate (clock/384: C-4 steps one byte per output, FD=0x800), backwards
    std::vector<int> pitchOffsetRF;
    DivPitchTable fwdRF;
    const int rfRate=(int)round(log.rfClock/384.0);
    if (log.hasRF) {
      fwdRF.init(ds.tuning,log.rfClock,786432.0*(double)rfRate/8372.0,0xffff,false,true);
      pitchOffsetRF.assign(65536,-1);
      for (int off=0; off<180*128; off++) {
        int f=fwdRF.get(off,0,0);
        if (f<0) f=0;
        if (f>0xffff) f=0xffff;
        if (pitchOffsetRF[f]<0 || ((off&127)==0 && (pitchOffsetRF[f]&127)!=0)) pitchOffsetRF[f]=off;
      }
      // (the ones it can't make: the closest one it can)
      int lastF=-1;
      std::vector<int> nextF(65536,-1);
      for (int P=65535, nf=-1; P>=0; P--) {
        if (pitchOffsetRF[P]>=0) nf=P;
        nextF[P]=nf;
      }
      for (int P=0; P<65536; P++) {
        if (pitchOffsetRF[P]>=0) {
          lastF=P;
          continue;
        }
        int lo=lastF, hi=nextF[P];
        int pick=(lo<0)?hi:((hi<0)?lo:((P-lo<=hi-P)?lo:hi));
        pitchOffsetRF[P]=(pick>=0)?pitchOffsetRF[pick]:0;
      }
    }
    // and for the SID: a frequency register (16 bits, no blocks). the notes
    // are found with the linear table; the song then uses Furnace's
    // non-linear pitch, where the note's frequency plus the fine tune and
    // pitch macro (in register units) reach any frequency exactly
    // VRC6 (CPU clock like the 2A03; pulses divide by 16, the saw by 14; the
    // register is the period minus 1)
    std::vector<int> periodOffsetVRC6[2];
    DivPitchTable fwdVRC6[2];
    if (log.hasVRC6) {
      double vclk=pal?((283.75*15625.0+25.0)*3.0/8.0):((315000000.0/88.0)/2.0);
      for (int w=0; w<2; w++) {
        fwdVRC6[w].init(ds.tuning,vclk,w?14:16,0x1000,true,true);
        std::vector<std::vector<int>> hits(4096);
        for (int off=0; off<180*128; off++) {
          int p=fwdVRC6[w].get(off,0,0)-1;
          if (p<0) p=0;
          if (p>4095) p=4095;
          hits[p].push_back(off);
        }
        periodOffsetVRC6[w].assign(4096,-1);
        for (int p=0; p<4096; p++) {
          if (hits[p].empty()) continue;
          int best=hits[p][hits[p].size()/2];
          for (int off: hits[p]) {
            if ((off&127)==0) {
              best=off;
              break;
            }
          }
          periodOffsetVRC6[w][p]=best;
        }
        // (periods no note reaches: the closest one that does)
        for (int p=0; p<4096; p++) {
          if (periodOffsetVRC6[w][p]>=0) continue;
          int lo=p, hi=p;
          while (lo>=0 && periodOffsetVRC6[w][lo]<0) lo--;
          while (hi<4096 && periodOffsetVRC6[w][hi]<0) hi++;
          int pick=(lo<0)?hi:((hi>=4096)?lo:((p-lo<=hi-p)?lo:hi));
          if (pick>=0 && pick<4096 && periodOffsetVRC6[w][pick]>=0) periodOffsetVRC6[w][p]=periodOffsetVRC6[w][pick];
          else periodOffsetVRC6[w][p]=0;
        }
      }
    }
    std::vector<int> pitchOffsetSID;
    DivPitchTable fwdSID, ptSIDn;
    if (log.hasSID && !ctx.tuningSet) {
      // the tuning of the player's frequency table (A-4 in Hz), so its notes
      // land on Furnace's: the average distance of every played frequency
      // to the closest semitone
      double sx=0, sy=0;
      int cnt=0;
      for (int c=26; c<29; c++) {
        for (int k=0; k<numTicks; k++) {
          const VGMChanTick& t=chanTicks[c][k];
          if (!t.aud || !t.restart || t.period<64) continue;
          double hz=(double)t.period*log.sidClock/16777216.0;
          double semi=12.0*std::log(hz/440.0)/std::log(2.0);
          double frac=semi-floor(semi);
          sx+=cos(frac*2.0*M_PI);
          sy+=sin(frac*2.0*M_PI);
          cnt++;
        }
      }
      if (cnt>=8 && sx*sx+sy*sy>(double)cnt*(double)cnt*0.25) {
        double off=atan2(sy,sx)/(2.0*M_PI);
        double tun=440.0*pow(2.0,off/12.0);
        tun=round(tun*100.0)/100.0;
        if (fabs(tun-440.0)>=0.05) {
          ds.tuning=tun;
          logI("SID import: tuning A-4 = %.2f Hz",tun);
        }
      }
      ctx.tuningSet=true;
    }
    if (log.hasSID) {
      fwdSID.init(ds.tuning,log.sidClock,524288,0xffff,false,true);
      ptSIDn.init(ds.tuning,log.sidClock,524288,0xffff,false,false);
      const int span=1<<16;
      pitchOffsetSID.assign(span,-1);
      for (int off=0; off<180*128; off++) {
        int f=fwdSID.get(off,0,0);
        if (f<0) f=0;
        if (f>65535) f=65535;
        if (pitchOffsetSID[f]<0 || ((off&127)==0 && (pitchOffsetSID[f]&127)!=0)) pitchOffsetSID[f]=off;
      }
      std::vector<int> lo(span,-1), hi(span,-1);
      int last=-1;
      for (int L=0; L<span; L++) {
        if (pitchOffsetSID[L]>=0) last=L;
        lo[L]=last;
      }
      last=-1;
      for (int L=span-1; L>=0; L--) {
        if (pitchOffsetSID[L]>=0) last=L;
        hi[L]=last;
      }
      for (int L=0; L<span; L++) {
        if (pitchOffsetSID[L]>=0) continue;
        int pick=-1;
        if (lo[L]<0) {
          pick=hi[L];
        } else if (hi[L]<0) {
          pick=lo[L];
        } else {
          pick=(L-lo[L]<=hi[L]-L)?lo[L]:hi[L];
        }
        pitchOffsetSID[L]=(pick>=0)?pitchOffsetSID[pick]:0;
      }
    }
    // and for the Game Boy: Furnace's period (2048 minus the register) for
    // the pulses and the wave, backwards. the noise: Furnace's notes from
    // 60 up pick an NR43 value from its table; the closest one in frequency
    std::vector<int> pitchOffsetGB;
    DivPitchTable fwdGB;
    int gbNoiseNote[256];
    if (log.hasGB) {
      fwdGB.init(ds.tuning,4194304.0,16,0x7ff,true,true);
      pitchOffsetGB.assign(2048,-1);
      for (int off=0; off<180*128; off++) {
        int f=fwdGB.get(off,0,0);
        if (f<1) f=1;
        if (f>2047) f=2047;
        if (pitchOffsetGB[f]<0 || ((off&127)==0 && (pitchOffsetGB[f]&127)!=0)) pitchOffsetGB[f]=off;
      }
      for (int P=0; P<2048; P++) {
        if (pitchOffsetGB[P]>=0) continue;
        int lo=P, hi=P;
        while (lo>=0 && pitchOffsetGB[lo]<0) lo--;
        while (hi<2048 && pitchOffsetGB[hi]<0) hi++;
        int pick=(lo<0)?hi:((hi>=2048)?lo:((P-lo<=hi-P)?lo:hi));
        pitchOffsetGB[P]=(pick>=0 && pick<2048 && pitchOffsetGB[pick]>=0)?pitchOffsetGB[pick]:0;
      }
      // (Furnace's table: note 60+i plays noiseTable[i])
      static const unsigned char gbNoiseTable[61]={
        0,
        0xf7,0xf6,0xf5,0xf4,0xe7,0xe6,0xe5,0xe4,0xd7,0xd6,0xd5,0xd4,
        0xc7,0xc6,0xc5,0xc4,0xb7,0xb6,0xb5,0xb4,0xa7,0xa6,0xa5,0xa4,
        0x97,0x96,0x95,0x94,0x87,0x86,0x85,0x84,0x77,0x76,0x75,0x74,
        0x67,0x66,0x65,0x64,0x57,0x56,0x55,0x54,0x47,0x46,0x45,0x44,
        0x37,0x36,0x35,0x34,0x27,0x26,0x25,0x24,0x17,0x16,0x15,0x14,
      };
      // (the noise's frequency: 524288 / divisor / 2^(shift+1), divisor 0
      // counting as 0.5)
      auto noiseHz=[](int v) -> double {
        int r=v&7, sh=v>>4;
        double d=(r==0)?0.5:(double)r;
        return 524288.0/d/pow(2.0,(double)(sh+1));
      };
      static const unsigned char gbNoiseExtra[4]={0x07,0x06,0x05,0x04};
      for (int v=0; v<256; v++) {
        double hz=noiseHz(v&0xf7);
        int best=1;
        double bestD=1e30;
        for (int i=1; i<61; i++) {
          double d=fabs(log2(noiseHz(gbNoiseTable[i]))-log2(hz));
          if (d<bestD) {
            bestD=d;
            best=i;
          }
        }
        // (the table goes on with 0x07-0x04 at 61-64 and 0x03-0x00 at 65-68)
        for (int i=0; i<4; i++) {
          double d=fabs(log2(noiseHz(gbNoiseExtra[i]))-log2(hz));
          if (d<bestD) {
            bestD=d;
            best=61+i;
          }
        }
        for (int i=0; i<4; i++) {
          double d=fabs(log2(noiseHz(3-i))-log2(hz));
          if (d<bestD) {
            bestD=d;
            best=65+i;
          }
        }
        gbNoiseNote[v]=60+best;
      }
    }
    // offset of a tick (noise: note number*128)
    auto tickOffset=[&](int c, const VGMChanTick& t) -> int {
      if (c>=52) {
        if (c==55) return gbNoiseNote[t.period&0xff]<<7;
        return pitchOffsetGB[t.period&2047];
      }
      if (c>=44) return log.hasRF?pitchOffsetRF[t.period&0xffff]:pitchOffsetSNES[t.period&0x3fff];
      if (c>=32) {
        if (curFixedBlock>0) return pitchOffsetOPNBlock[curFixedBlock-1][(t.period>>(curFixedBlock-1))&2047];
        return pitchOffsetOPN[t.period&(opnSpan-1)];
      }
      if (c>=29) return periodOffsetVRC6[(c==31)?1:0][t.period&4095];
      if (c>=26) return pitchOffsetSID[t.period&0xffff];
      if (c>=12) {
        if (curFixedBlock>0) return pitchOffsetBlock[curFixedBlock-1][(t.period>>(curFixedBlock-1))&511];
        return pitchOffsetOPLL[t.period&0xffff];
      }
      if (c==11) {
        // SN noise: Furnace takes note%12 = 0, 1, 2 for the fixed rates 2, 1, 0
        // (C-4, C#4, D-4 here), or a pitch when it borrows square 3's period
        if (t.period&0x1000) return (108+(2-(t.period&3)))<<7;
        return periodOffsetSNNoise[t.period&1023];
      }
      if (c>=8) return periodOffsetSN[t.period&1023];
      if (c==3) return (60+(15-(t.period&15)))<<7;
      if (c>=5 && c<8) return periodOffset5B[t.period&4095];
      return periodOffset[t.period&2047];
    };
    // the period (or fnum<<block) Furnace writes for a pitch offset
    auto periodAt=[&](int c, int off) -> int {
      if (c>=52) {
        int f=fwdGB.get(off,0,0);
        if (f<1) f=1;
        if (f>2047) f=2047;
        return f;
      }
      if (c>=44 && log.hasRF) {
        int f=fwdRF.get(off,0,0);
        if (f<0) f=0;
        if (f>0xffff) f=0xffff;
        return f;
      }
      if (c>=44) {
        int f=fwdSNES.get(off,0,0);
        if (f<0) f=0;
        if (f>0x3fff) f=0x3fff;
        return f;
      }
      if (c>=32) return opnFwd(off,curFixedBlock);
      if (c>=29) {
        // (Furnace writes the table's value minus 1)
        int p=fwdVRC6[(c==31)?1:0].get(off,0,0)-1;
        if (p<0) p=0;
        if (p>4095) p=4095;
        return p;
      }
      if (c>=26) {
        int f=fwdSID.get(off,0,0);
        if (f<0) f=0;
        if (f>65535) f=65535;
        return f;
      }
      if (c>=12) {
        int f=fwdOPLL.get(off,0,0);
        if (f<0) f=0;
        if (f>65535) f=65535;
        int b;
        if (curFixedBlock>0) {
          b=curFixedBlock-1;
        } else {
          b=f/343;
          if (b>0) b=bsr(b);
          if (b>7) b=7;
        }
        int fn=f>>b;
        if (fn>0x1ff) fn=0x1ff;
        return fn<<b;
      }
      if (c>=8) {
        int p;
        if (off>snThr) {
          p=((16<<7)-(off-snThr))>>7;
        } else {
          p=fwdSN.get(off,0,0);
        }
        if (p<0) p=0;
        if (p>1023) p=1023;
        return p;
      }
      if (c>=5) {
        int p=fwd5B.get(off,0,0);
        if (p<0) p=0;
        if (p>4095) p=4095;
        return p;
      }
      int p=pt.get(off,0,0)-1;
      if (p<0) p=0;
      if (p>2047) p=2047;
      return p;
    };
    auto tickNote=[&](int c, const VGMChanTick& t) -> int {
      return (tickOffset(c,t)+64)>>7;
    };
    // period mode (Furnace's non-linear pitch): the fine tune, pitch macro and
    // vibrato count in periods, like the game's driver does. a slide of so
    // many periods per tick is then the same macro on any note, and a
    // driver's own tuning is just an E5xx. only for songs with only the
    // 2A03 (the pitch mode is one setting for the whole song); the pulses
    // and the triangle use it, the noise keeps its notes.
    const bool sidOnly=log.hasSID && !log.hasNES && !log.has5B && !log.hasSN && !log.hasOPLL && !log.hasOPN;
    const bool canPeriod=(log.hasNES && !log.has5B && !log.hasSN && !log.hasOPLL && !log.hasSID && !log.hasVRC6 && !log.hasOPN) || sidOnly;
    DivPitchTable ptPer;
    if (canPeriod) ptPer.init(ds.tuning,log.clock,16,0x800,true,false);
    bool curPeriod=false;
    auto usesPer=[&](int c) -> bool {
      return curPeriod && (c<3 || (c>=26 && c<32));
    };
    // the period Furnace writes for note n with pitch r (r: 1/128 semitone,
    // or periods in period mode)
    auto periodOf=[&](int c, int n, int r) -> int {
      if (usesPer(c) && c>=26) {
        int f=ptSIDn.getBase(n)+r;
        if (f<0) f=0;
        if (f>65535) f=65535;
        return f;
      }
      if (usesPer(c)) {
        int p=ptPer.getBase(n)-r-1;
        if (p<0) p=0;
        if (p>2047) p=2047;
        return p;
      }
      return periodAt(c,(n<<7)+r);
    };

    // --- step 6: note onsets (first pass) ---
    std::vector<int> onsets[NCH];
    // repeated notes (see below): only taken if they land on a row start
    std::vector<int> repeatCand[NCH];
    for (int c: tonal) {
      std::vector<VGMChanTick>& ct=chanTicks[c];
      for (int k=0; k<numTicks; k++) {
        if (!ct[k].aud) continue;
        if (k==0 || !ct[k-1].aud) {
          onsets[c].push_back(k);
          continue;
        }
        // FM: a note is a key on, nothing else (the chip's own envelope
        // starts there; volume and pitch changes go into the note)
        if (isFMChan(c)) {
          if (ct[k].restart) onsets[c].push_back(k);
          continue;
        }
        // pitch changes alone are not new notes here (arpeggios and vibrato
        // also rewrite the period); legato notes are found later on row starts.
        // the triangle has no volume, so only silence tells its notes apart.
        if (c==2) continue;
        bool volUp=(ct[k].vol>ct[k-1].vol);
        // (Game Boy: a trigger on another note, or another wave)
        if (c>=52 && ct[k].restart && (tickNote(c,ct[k])!=tickNote(c,ct[k-1]) || (c==54 && ct[k].duty!=ct[k-1].duty))) {
          onsets[c].push_back(k);
          continue;
        }
        if (ct[k].restart && volUp) {
          onsets[c].push_back(k);
          continue;
        }
        if (ct[k].vol-ct[k-1].vol>=3) {
          onsets[c].push_back(k);
          continue;
        }
        // the same note again: Furnace (and many drivers) don't rewrite the
        // period for a repeated note, only the envelope starts over. if the
        // volume goes up and the next ticks (volume, duty, noise) are the
        // start of the current note again, it may be a new note (step 8 keeps
        // it if it falls on a row start, like a note in a tracker would;
        // envelopes that loop by themselves rarely do)
        if (volUp && !onsets[c].empty()) {
          int o=onsets[c].back();
          int L=std::min(4,k-o);
          if (L>=2) {
            bool same=true;
            int checked=0;
            for (int i=0; i<L && same; i++) {
              if (k+i>=numTicks || !ct[k+i].aud) break;
              const VGMChanTick& x=ct[o+i];
              const VGMChanTick& y=ct[k+i];
              if (x.vol!=y.vol || x.duty!=y.duty || x.noise!=y.noise) same=false;
              checked++;
            }
            if (same && checked>=std::min(L,3)) {
              repeatCand[c].push_back(k);
              continue;
            }
          }
        }
      }
    }
    // a "note" one tick after another one on the same channel is almost always
    // the driver finishing the attack (volume set a frame later): merge it
    // (an FM key on is always a note)
    for (int c: tonal) {
      if (isFMChan(c)) continue;
      std::vector<int> kept;
      for (int k: onsets[c]) {
        if (!kept.empty() && k-kept.back()<=1) continue;
        kept.push_back(k);
      }
      onsets[c]=kept;
    }
    // a note that is still playing at the loop point is split there: the
    // second time around the channel has to play it from that point on
    // (and not whatever the end of the song left playing). not on FM,
    // where a new note would key on again
    if (loopTick>0 && loopTick<numTicks) {
      for (int c: tonal) {
        if (isFMChan(c)) continue;
        if (!chanTicks[c][loopTick].aud) continue;
        std::vector<int>& on=onsets[c];
        auto it=std::lower_bound(on.begin(),on.end(),loopTick);
        if (it!=on.end() && *it==loopTick) continue;
        on.insert(it,loopTick);
      }
    }
    // DPCM triggers and PCM streams are onsets on the DMC channel
    // (a trigger during a PCM stream is ignored)
    std::vector<int> dmcOnsetTicks;
    for (VGMDPCMTrigger& t: triggers) dmcOnsetTicks.push_back(t.tick);
    for (PCMStream& s: pcmStreams) dmcOnsetTicks.push_back(s.tick);
    std::sort(dmcOnsetTicks.begin(),dmcOnsetTicks.end());
    onsets[4]=dmcOnsetTicks;
    // the YM2612 DAC samples (they help find the rows, like the drums they are)
    for (OPNDACStream& s: opnDacStreams) onsets[38].push_back(s.tick);
    // (and the PWM's)
    for (PWMHit& h: pwmHits) onsets[43].push_back(h.tick);

    // --- step 7: find the row grid ---
    // game drivers often don't use a fixed number of ticks per row
    // (FamiTracker-style tempo gives rows of 4, 4, 5, 4, 4, 5...), so the rows
    // are anchored on the notes themselves. a row length R is chosen so that
    // the gaps between notes split into rows of floor(R) or ceil(R) ticks.
    // notes closer than half a row to the previous one go in the same row with
    // a delay (EDxx). R is chosen to get the fewest rows and speed changes.
    std::vector<int> rowLen;
    bool gridEven=false;
    {
      // anchor ticks and the channels that start a note there
      std::map<int,unsigned long long> anchorMask;
      for (int c=0; c<NCH; c++) for (int t: onsets[c]) anchorMask[t]|=(1ull<<c);
      if (loopTick>0) anchorMask[loopTick]|=0;
      std::vector<std::pair<int,unsigned long long>> anchors(anchorMask.begin(),anchorMask.end());

      // even: every gap between notes takes an even number of rows, so the
      // notes land on rows 0, 2, 4... (easier to edit and to move around)
      // (rows of up to 24 ticks at 60 Hz, 0.4 seconds: as long at faster
      // ticks, so a driver that ticks at 500 Hz doesn't get rows a few
      // milliseconds long, which look like a very fast song. at those rates
      // a row may also be a tick or two off: drivers with a fine timer and a
      // tempo of their own land their notes on it a little unevenly)
      const double rScale=std::max(1.0,std::min(255.0/24.0,(44100.0/period)/60.0));
      const int rTol=(int)floor(rScale/4.0);
      auto buildRows=[&](double R, bool even, std::vector<int>& out, int& invalid, int& absorbed, int& conflicts) {
        out.clear();
        invalid=0;
        absorbed=0;
        conflicts=0;
        int lo=(int)floor(R)-rTol, hi=(int)ceil(R)+rTol;
        auto split=[&](int delta, int k) {
          if (k<1) k=1;
          if (delta>255*k) k=(delta+254)/255;
          // spread the remainder evenly (Bresenham)
          int acc=0;
          for (int j=0; j<k; j++) {
            int len=(delta*(j+1))/k-acc;
            acc+=len;
            out.push_back(len);
            if (len<lo || len>hi) invalid++;
          }
        };
        int cur=0;
        unsigned long long curMask=0;
        for (auto& an: anchors) {
          int t=an.first;
          int delta=t-cur;
          if (delta<=0) {
            curMask|=an.second;
            continue;
          }
          int k=even?(2*(int)round((double)delta/(2.0*R))):(int)round((double)delta/R);
          // the loop point must start a row
          if (k==0 && t!=loopTick) {
            // same row as the current anchor, with a delay
            absorbed++;
            if (curMask&an.second) conflicts++;
            curMask|=an.second;
            continue;
          }
          split(delta,k);
          cur=t;
          curMask=an.second;
        }
        // the last row may be cut short by the end of the song: don't count it
        int tail=numTicks-cur;
        if (tail>0) {
          int before=invalid;
          split(tail,(int)round((double)tail/R));
          if (invalid>before) invalid--;
        }
      };

      // the best grid of each kind (normal, even)
      double bestCostK[2]={1e30,1e30};
      double bestRK[2]={1.0,1.0};
      int totalAnchors=(int)anchors.size();
      std::vector<int> tmp;
      const int riMax=(int)round(2400.0*rScale);
      const int riStep=std::max(1,(int)floor(rScale));
      for (int pass=0; pass<2; pass++) for (int ri=riMax; ri>=100; ri-=riStep) {
        const bool even=(pass==1);
        double R=(double)ri/100.0;
        int invalid, absorbed, conflicts;
        buildRows(R,even,tmp,invalid,absorbed,conflicts);
        if (tmp.empty()) continue;
        if (invalid*20>(int)tmp.size()) continue;
        // notes a tick or two after the others (drivers often start some
        // channels a frame late) are fine as delays, but not too many
        if (absorbed*2>totalAnchors) continue;
        // two notes of one channel in one row: the second becomes part of the
        // first one's macros (still exact, just less readable)
        if (conflicts*4>totalAnchors) continue;
        int changes=0;
        for (size_t j=1; j<tmp.size(); j++) if (tmp[j]!=tmp[j-1]) changes++;
        double cost=(double)tmp.size()+(double)changes+3.0*(double)absorbed+4.0*(double)invalid+3.0*(double)conflicts;
        if (even) cost=(double)tmp.size()*0.5+(double)changes*0.5+3.0*(double)absorbed+4.0*(double)invalid+3.0*(double)conflicts;
        if (cost<bestCostK[pass]) {
          bestCostK[pass]=cost;
          bestRK[pass]=R;
        }
      }
      // notes on even rows when the loader picked that (it tries both and
      // keeps it if it needs no more instruments), and the grid is valid
      bool bestEven=(ctx.gridMode==1 && bestCostK[1]<1e29);
      if (bestCostK[0]>=1e29) bestEven=(bestCostK[1]<1e29);
      double bestR=bestRK[bestEven?1:0];
      int invalid, absorbed, conflicts;
      buildRows(bestR,bestEven,rowLen,invalid,absorbed,conflicts);
      gridEven=bestEven;
      logI("VGM import: row length %.2f ticks (%d rows, %d delayed notes)%s",bestR,(int)rowLen.size(),absorbed,bestEven?", notes on even rows":"");
    }
    const int shift=0;
    std::vector<int> rowStart;
    {
      int t=0;
      for (int len: rowLen) {
        rowStart.push_back(t);
        t+=len;
      }
      rowStart.push_back(t);
      rowStart.push_back(t+1);
    }
    auto rowOf=[&](int tick) -> int {
      int r=(int)(std::upper_bound(rowStart.begin(),rowStart.end(),tick)-rowStart.begin())-1;
      if (r<0) r=0;
      if (r>=(int)rowLen.size()) r=(int)rowLen.size()-1;
      return r;
    };
    int numRows=(int)rowLen.size();

    // --- step 8: pitch changes on row starts are new notes too (legato) ---
    // (onsetsRep also has the repeated notes: the notes are built with and
    // without them, and the song keeps whichever needs fewer instruments)
    std::vector<int> onsetsRep[NCH];
    for (int c: tonal) {
      std::vector<VGMChanTick>& ct=chanTicks[c];
      std::vector<int> extra;
      size_t oi=0;
      // on FM a new note restarts the chip's envelope: pitch changes without
      // a key on stay in the note's macros
      for (int r=1; r<numRows && !isFMChan(c); r++) {
        // (notes on even rows only, if that is the grid)
        if (gridEven && (r&1)) continue;
        int k=rowStart[r]-shift;
        if (k<=0 || k>=numTicks) continue;
        while (oi<onsets[c].size() && onsets[c][oi]<k) oi++;
        if (oi<onsets[c].size() && onsets[c][oi]==k) continue;
        if (!ct[k].aud || !ct[k-1].aud) continue;
        int n=tickNote(c,ct[k]);
        if (n==tickNote(c,ct[k-1])) continue;
        int rowLen=rowStart[r+1]-rowStart[r];
        int need=std::min(3,rowLen);
        bool stable=true;
        for (int j=1; j<need && k+j<numTicks; j++) {
          if (!ct[k+j].aud || tickNote(c,ct[k+j])!=n) {
            stable=false;
            break;
          }
        }
        if (stable) extra.push_back(k);
      }
      // (and only when it starts like the note before it, or holds its
      // first note for a moment: a driver that plays its attack on another
      // pitch is one long note with its own shape)
      std::vector<int> extraRep;
      for (int k: repeatCand[c]) {
        int r=rowOf(k+shift);
        if (r<0 || r>=numRows || rowStart[r]!=k+shift) continue;
        if (gridEven && (r&1)) continue;
        auto po=std::upper_bound(onsets[c].begin(),onsets[c].end(),k);
        int o=(po==onsets[c].begin())?-1:*(po-1);
        // the pitch moves like it did at the start of the note before
        // (the same note, or an arpeggio played again from another root)
        bool sameShape=(o>=0);
        for (int i=1; i<3 && sameShape; i++) {
          if (k+i>=numTicks || o+i>=k || !ct[k+i].aud) break;
          if (tickNote(c,ct[k+i])-tickNote(c,ct[k])!=tickNote(c,ct[o+i])-tickNote(c,ct[o])) sameShape=false;
        }
        bool steady=(k+1>=numTicks || !ct[k+1].aud || tickNote(c,ct[k+1])==tickNote(c,ct[k]));
        if (!sameShape && !steady) continue;
        extraRep.push_back(k);
      }
      // one note per row: later notes in the same row melt into the first one
      auto finish=[&](std::vector<int> v) -> std::vector<int> {
        std::sort(v.begin(),v.end());
        v.erase(std::unique(v.begin(),v.end()),v.end());
        std::vector<int> kept;
        int lastRow=-1;
        for (int k: v) {
          int r=rowOf(k+shift);
          if (r==lastRow) continue;
          kept.push_back(k);
          lastRow=r;
        }
        return kept;
      };
      std::vector<int> base=onsets[c];
      base.insert(base.end(),extra.begin(),extra.end());
      std::vector<int> withRep=base;
      withRep.insert(withRep.end(),extraRep.begin(),extraRep.end());
      onsets[c]=finish(base);
      onsetsRep[c]=finish(withRep);
    }

    // --- step 9: build the song ---
    sub->hz=(float)(44100.0/period);
    int patLen=64;
    while (patLen<256 && (numRows+patLen-1)/patLen>DIV_MAX_PATTERNS) patLen*=2;
    sub->patLen=patLen;
    int numOrders=(numRows+patLen-1)/patLen;
    if (numOrders>DIV_MAX_PATTERNS) {
      numOrders=DIV_MAX_PATTERNS;
      numRows=numOrders*patLen;
      warns.push_back("song is too long for the pattern limit and was cut");
    }
    if (numOrders<1) numOrders=1;
    sub->ordersLen=numOrders;
    for (int c=0; c<numChans; c++) {
      for (int o=0; o<numOrders; o++) sub->orders.ord[c][o]=o;
    }
    int maxFxCol[NCH];
    for (int c=0; c<NCH; c++) maxFxCol[c]=1;
    auto cellAt=[&](int c, int row) -> short* {
      if (row<0) row=0;
      if (row>=numRows) row=numRows-1;
      int pc=(patIdx[c]>=0)?patIdx[c]:0;
      return sub->pat[pc].getPattern(row/patLen,true)->newData[row%patLen];
    };

    // row lengths: a groove if they repeat (up to 16 steps), otherwise speed
    // changes (0Fxx) where the length changes
    int loopRow=-1;
    if (loopTick>=0) {
      loopRow=rowOf(loopTick);
      if (loopRow>=numRows) loopRow=-1;
    }
    {
      int grooveP=0;
      for (int p=1; p<=16 && !grooveP; p++) {
        bool ok=(numRows>p);
        for (int j=p; j<numRows-1 && ok; j++) {
          if (rowLen[j]!=rowLen[j-p]) ok=false;
        }
        // after looping back the groove must be in the same step
        if (ok && loopRow>=0 && (loopRow%p)!=(numRows%p)) ok=false;
        if (ok) grooveP=p;
      }
      if (grooveP>0) {
        sub->speeds.len=grooveP;
        for (int j=0; j<grooveP; j++) sub->speeds.val[j]=rowLen[j];
        logI("VGM import: groove of %d step(s)",grooveP);
      } else {
        // not exactly periodic. drivers with a fractional tempo give rows like
        // 7 6 7 6 7 7 6 7 6...: a groove that slips now and then. play a
        // short groove and switch to the right rotation of it (09xx, which
        // also restarts the groove) only where the rows slip. compare that
        // with plain speed changes and keep whichever needs fewer commands.
        int plainChanges=0;
        for (int j=1; j<numRows; j++) if (rowLen[j]!=rowLen[j-1] || j==loopRow) plainChanges++;

        // simulate a groove plan: returns the number of 09xx commands
        auto simulate=[&](const std::vector<int>& P, bool emit) -> int {
          int p=(int)P.size();
          auto findGroove=[&](const std::vector<int>& g) -> int {
            for (size_t k=0; k<ds.grooves.size(); k++) {
              DivGroovePattern& gp=ds.grooves[k];
              if (gp.len!=(int)g.size()) continue;
              bool same=true;
              for (int x=0; x<gp.len; x++) if (gp.val[x]!=g[x]) same=false;
              if (same) return (int)k;
            }
            if (ds.grooves.size()>=255) return -1;
            DivGroovePattern gp;
            gp.len=g.size();
            for (size_t x=0; x<g.size(); x++) gp.val[x]=g[x];
            ds.grooves.push_back(gp);
            return (int)ds.grooves.size()-1;
          };
          auto rotation=[&](int r) -> std::vector<int> {
            std::vector<int> g(p);
            for (int x=0; x<p; x++) g[x]=P[(r+x)%p];
            return g;
          };
          int cmds=0;
          int rot=-1, idx=0; // rot -1: a one-row groove of an odd length
          int single=0;
          for (int j=0; j<numRows; j++) {
            int want=rowLen[j];
            int expected=(rot>=0)?P[(rot+idx)%p]:single;
            bool mustSet=(j==0) || (j==loopRow) || (want!=expected);
            if (!mustSet) {
              idx++;
              continue;
            }
            // the rotation that starts with this row and follows the next rows
            // the longest
            int bestR=-1, bestRun=-1;
            for (int r=0; r<p; r++) {
              if (P[r]!=want) continue;
              int run=0;
              while (j+run<numRows && run<64 && rowLen[j+run]==P[(r+run)%p]) run++;
              if (run>bestRun) {
                bestRun=run;
                bestR=r;
              }
            }
            if (j==0) {
              if (emit) {
                std::vector<int> g=(bestR>=0)?rotation(bestR):std::vector<int>(1,want);
                sub->speeds.len=g.size();
                for (size_t x=0; x<g.size(); x++) sub->speeds.val[x]=g[x];
              }
            } else {
              cmds++;
              if (emit) {
                int gi=findGroove((bestR>=0)?rotation(bestR):std::vector<int>(1,want));
                if (gi>=0) vgmPutFx(cellAt(cmdChan,j),0x09,gi,maxFxCol[cmdChan]);
              }
            }
            rot=bestR;
            single=want;
            idx=1;
          }
          return cmds;
        };

        std::vector<int> bestP;
        int bestCmds=plainChanges;
        for (int p=2; p<=4; p++) {
          // candidate patterns: the row lengths at a few places in the song
          for (int st=0; st+p<=numRows && st<64; st++) {
            std::vector<int> P(rowLen.begin()+st,rowLen.begin()+st+p);
            int cmds=simulate(P,false);
            if (cmds<bestCmds) {
              bestCmds=cmds;
              bestP=P;
            }
          }
        }
        // the groove table is shared with the other subsongs and needs one
        // entry per rotation; only worth it if it saves a lot
        if (!bestP.empty() && bestCmds*2<plainChanges) {
          size_t groovesBefore=ds.grooves.size();
          int cmds=simulate(bestP,true);
          logI("VGM import: groove of %d with %d switches (instead of %d speed changes), %d groove patterns",(int)bestP.size(),cmds,plainChanges,(int)ds.grooves.size());
          (void)groovesBefore;
        } else {
          sub->speeds.len=1;
          sub->speeds.val[0]=rowLen[0];
          int speedChanges=0;
          for (int j=1; j<numRows; j++) {
            if (rowLen[j]!=rowLen[j-1] || j==loopRow) {
              // song commands go in the first channel, so they survive when the
              // song is moved to a chip with fewer channels
              vgmPutFx(cellAt(cmdChan,j),0x0f,rowLen[j],maxFxCol[cmdChan]);
              speedChanges++;
            }
          }
          logI("VGM import: %d speed changes",speedChanges);
        }
      }
    }

    // instruments (shared with other subsongs through the context)
    std::map<String,int>& insMap=ctx.pcmInsMap;
    int& instrumentsOver=ctx.instrumentsOver;

    // --- step 10: DMC channel (first, so the drum kit always gets an instrument) ---
    if (log.hasNES) {
      const int c=4;
      // DPCM samples, one per unique data/loop
      std::map<String,int>& dpcmSampleMap=ctx.dpcmSampleMap;
      std::map<String,int>& pcmSampleMap=ctx.pcmSampleMap;
      // kit slots: (sample, rate, delta) -> (kit, note)
      std::map<String,std::pair<int,int>>& kitSlots=ctx.kitSlots;
      std::vector<int>& kits=ctx.kits; // instrument indices
      int& kitFill=ctx.kitFill;
      int& pcmCount=ctx.pcmCount;

      struct DMCNote {
        int tick;
        int kind; // 0: DPCM, 1: PCM
        int index;
      };
      std::vector<DMCNote> notes;
      for (size_t i=0; i<triggers.size(); i++) {
        bool inPCM=false;
        for (PCMStream& s: pcmStreams) {
          if (triggers[i].tick>=s.tick && triggers[i].tick<=tickOf(s.end)) {
            inPCM=true;
            break;
          }
        }
        if (inPCM) continue;
        DMCNote n;
        n.tick=triggers[i].tick;
        n.kind=0;
        n.index=(int)i;
        notes.push_back(n);
      }
      for (size_t i=0; i<pcmStreams.size(); i++) {
        DMCNote n;
        n.tick=pcmStreams[i].tick;
        n.kind=1;
        n.index=(int)i;
        notes.push_back(n);
      }
      std::stable_sort(notes.begin(),notes.end(),[](const DMCNote& x, const DMCNote& y) {
        return x.tick<y.tick;
      });

      // lone $4011 writes: they set the delta counter of the next sample,
      // or change the DMC level (which also softens triangle and noise)
      std::vector<std::pair<int,int>> deltaWrites; // tick, value
      for (VGMDACWrite& w: loneDAC) deltaWrites.push_back(std::make_pair(tickOf(w.time),w.val));

      int lastRow=-1;
      size_t dwPos=0;
      bool pcmMode=false;
      int dropped=0;
      for (DMCNote& n: notes) {
        int row=rowOf(n.tick+shift);
        int delay=(n.tick+shift)-rowStart[row];
        // delta writes before this note
        int delta=-1;
        while (dwPos<deltaWrites.size() && deltaWrites[dwPos].first<=n.tick) {
          if (deltaWrites[dwPos].first==n.tick) {
            delta=deltaWrites[dwPos].second;
          } else {
            int r=rowOf(deltaWrites[dwPos].first+shift);
            short* dc=cellAt(c,r);
            vgmPutFx(dc,0x11,deltaWrites[dwPos].second,maxFxCol[c]);
          }
          dwPos++;
        }
        if (row==lastRow) {
          dropped++;
          continue;
        }
        lastRow=row;
        short* cell=cellAt(c,row);
        if (n.kind==0) {
          VGMDPCMTrigger& t=triggers[n.index];
          String dataKey=fmt::sprintf("%d:",t.loop?1:0);
          dataKey.append((const char*)t.data.data(),t.data.size());
          int sampleIndex;
          auto si=dpcmSampleMap.find(dataKey);
          if (si!=dpcmSampleMap.end()) {
            sampleIndex=si->second;
          } else {
            DivSample* smp=new DivSample;
            smp->name=fmt::sprintf("DPCM $%.4X%s",t.addr,t.loop?" loop":"");
            smp->depth=DIV_SAMPLE_DEPTH_1BIT_DPCM;
            smp->centerRate=33144;
            smp->init(t.len*8);
            memcpy(smp->dataDPCM,t.data.data(),t.len);
            if (t.loop) {
              smp->loop=true;
              smp->loopStart=0;
              smp->loopEnd=t.len*8;
            }
            sampleIndex=(int)ds.sample.size();
            ds.sample.push_back(smp);
            dpcmSampleMap[dataKey]=sampleIndex;
          }
          String slotKey=fmt::sprintf("%d:%d:%d",sampleIndex,t.rate,delta);
          std::pair<int,int> slot;
          auto ki=kitSlots.find(slotKey);
          if (ki!=kitSlots.end()) {
            slot=ki->second;
          } else {
            if (kits.empty() || kitFill>=120) {
              if (ds.ins.size()>=256) {
                instrumentsOver++;
                continue;
              }
              DivInstrument* ins=new DivInstrument;
              ins->type=DIV_INS_NES;
              ins->name=kits.empty()?String("DPCM Kit"):fmt::sprintf("DPCM Kit %d",(int)kits.size()+1);
              ins->amiga.useSample=true;
              ins->amiga.useNoteMap=true;
              for (int k=0; k<180; k++) ins->amiga.noteMap[k].map=-1;
              kits.push_back((int)ds.ins.size());
              ds.ins.push_back(ins);
              kitFill=0;
            }
            slot=std::make_pair(kits.back(),60+kitFill);
            DivInstrument* kit=ds.ins[kits.back()];
            kit->amiga.noteMap[slot.second].map=sampleIndex;
            kit->amiga.noteMap[slot.second].dpcmFreq=t.rate;
            kit->amiga.noteMap[slot.second].dpcmDelta=delta;
            kitFill++;
            kitSlots[slotKey]=slot;
          }
          cell[DIV_PAT_NOTE]=slot.second;
          cell[DIV_PAT_INS]=slot.first;
          pcmMode=false;
        } else {
          PCMStream& s=pcmStreams[n.index];
          double dur=s.end-s.start;
          double rate=(double)(s.w.size()-1)*44100.0/std::max(dur,1.0);
          if (rate<1000.0) rate=1000.0;
          if (rate>44100.0) rate=44100.0;
          int count=(int)round(dur*rate/44100.0)+1;
          std::vector<signed char> data(count);
          size_t wi=0;
          for (int k=0; k<count; k++) {
            double tt=s.start+(double)k*44100.0/rate;
            while (wi+1<s.w.size() && s.w[wi+1].time<=tt) wi++;
            data[k]=(signed char)(s.w[wi].val*2-128);
          }
          String dataKey=fmt::sprintf("%d:",(int)round(rate));
          dataKey.append((const char*)data.data(),data.size());
          int sampleIndex;
          auto si=pcmSampleMap.find(dataKey);
          if (si!=pcmSampleMap.end()) {
            sampleIndex=si->second;
          } else {
            DivSample* smp=new DivSample;
            smp->name=fmt::sprintf("PCM %d",++pcmCount);
            smp->depth=DIV_SAMPLE_DEPTH_8BIT;
            smp->centerRate=(int)round(rate);
            smp->init(count);
            memcpy(smp->data8,data.data(),count);
            sampleIndex=(int)ds.sample.size();
            ds.sample.push_back(smp);
            pcmSampleMap[dataKey]=sampleIndex;
          }
          String insKey=fmt::sprintf("pcm|%d",sampleIndex);
          int insIndex;
          auto ii=insMap.find(insKey);
          if (ii!=insMap.end()) {
            insIndex=ii->second;
          } else {
            if (ds.ins.size()>=256) {
              instrumentsOver++;
              continue;
            }
            DivInstrument* ins=new DivInstrument;
            ins->type=DIV_INS_AMIGA;
            ins->name=ds.sample[sampleIndex]->name;
            ins->amiga.initSample=sampleIndex;
            insIndex=(int)ds.ins.size();
            ds.ins.push_back(ins);
            insMap[insKey]=insIndex;
          }
          cell[DIV_PAT_NOTE]=108; // C-4 plays at the sample's rate
          cell[DIV_PAT_INS]=insIndex;
          if (!pcmMode) vgmPutFx(cell,0x18,0x00,maxFxCol[c]);
          pcmMode=true;
        }
        if (delay>0) vgmPutFx(cell,0xed,delay,maxFxCol[c]);
      }
      // remaining lone writes
      for (; dwPos<deltaWrites.size(); dwPos++) {
        int r=rowOf(deltaWrites[dwPos].first+shift);
        if (r>=numRows) continue;
        vgmPutFx(cellAt(c,r),0x11,deltaWrites[dwPos].second,maxFxCol[c]);
      }
      // stops
      for (VGMDPCMStop& st: stops) {
        int r=rowOf(st.tick+shift);
        if (r>=numRows) continue;
        int d=(st.tick+shift)-rowStart[r];
        short* cell=cellAt(c,r);
        if (cell[DIV_PAT_NOTE]!=-1) continue;
        if (d==0) {
          cell[DIV_PAT_NOTE]=DIV_NOTE_OFF;
        } else {
          vgmPutFx(cell,0xec,d,maxFxCol[c]);
        }
      }
      if (dropped>0) warns.push_back(fmt::sprintf("%d DPCM/PCM hits landed on the same row as another one and were left out",dropped));
    }

    // --- step 11: pulse, triangle and noise notes ---
    // first every note is measured, then instruments are made. if there are
    // more distinct ones than fit, the rarest are merged into the closest
    // common one (same channel type), so only rare variations change.
    struct VGMNoteRec {
      int c, rowA, delayA, base, fine, noteVol;
      bool useOff;
      int rowS, delayS;
      int set;
      int envPeriod; // 5B envelope period this note needs, -1: none
      bool cutInNoteRow; // the cut is an ECxx on the note's row
      int cutTicks;
      int relTick; // macro release (E7xx) this many ticks after the start, -1: none
      // pitch steps inside the note, written as legato notes: (row, delay,
      // note, fine tune)
      std::vector<std::array<int,4>> legato;
      // arpeggio effect (00xy) changes: (row, xy); the first one is on the note's row
      std::vector<std::pair<int,int>> arpFx;
      // vibrato effect (04xy): (row, xy), -1 row: none. vibDelay: an EDxx on
      // that row, so it starts on the tick it should (a row without a note)
      int vibRow, vibVal;
      int vibDelay=0;
      int obs; // ticks the note hears its instrument
      int sidOnset=-1; // SID: the note's first tick
      bool sidDutyCut=false; // SID: its pulse width didn't fit in the macro
      // YM2612: the panning of the note (08xx), -1: none, -2: a pan macro
      int pan=-1;
      // YM2612: a note longer than its macros (255 ticks): its volume after
      // that, (row, volume), for the volume column
      std::vector<std::pair<int,int>> volTail;
    };
    std::vector<VGMMacroSet> sets;
    std::map<String,int> setMap;
    std::vector<VGMNoteRec> recs;
    std::map<String,std::vector<int>> nonVolSets;
    int scaledMatches=0, prefixMatches=0;
    // (long notes whose macros loop within a tick: see vgmLoopifyLoose)
    int loopedLoose=0;
    // (YM2612 notes whose pitch went to a close 04xy or just a fine tune,
    // instead of a pitch macro: see below)
    int looseVib=0;
    // first a look at every note's volume: an attack + sustain that shows up
    // with sustains of different lengths is one instrument with a release.
    // (a volume that only decays is left alone: the release would just make
    // more instruments there)
    std::map<String,std::set<int>> adsrFamilies;
    for (int c: tonal) {
      if (c==2 || isFMChan(c)) continue;
      std::vector<VGMChanTick>& ct=chanTicks[c];
      std::vector<int>& on=onsets[c];
      for (size_t i=0; i<on.size(); i++) {
        int a=on[i];
        int next=(i+1<on.size())?on[i+1]:numTicks;
        int s=next;
        while (s>a && !ct[s-1].aud) s--;
        if (s<=a) s=a+1;
        std::vector<int> rv;
        for (int k=a; k<std::min(next,s+1); k++) rv.push_back(ct[k].vol);
        int fa, fe;
        if (!vgmFindADSR(rv,fa,fe)) continue;
        String fk=fmt::sprintf("%d|%d|",c,rv[fa]);
        for (int k=0; k<fa; k++) fk+=fmt::sprintf("%d,",rv[k]);
        adsrFamilies[fk].insert(fe-fa);
      }
    }
    // YM2413 custom patch: there is only one, shared by the channels. a change
    // to it belongs to the latest note that started with the custom patch
    // (its instrument gets FM macros), until another one starts.
    std::vector<int> customOnsets;
    if (log.hasOPLL) {
      for (int c=12; c<12+opllFM; c++) {
        for (int k: onsets[c]) if (chanTicks[c][k].duty==0) customOnsets.push_back(k);
      }
      std::sort(customOnsets.begin(),customOnsets.end());
    }
    // the custom patch fields: register, shift, mask
    static const int fmField[24][3]={
      {0,7,1},{0,6,1},{0,5,1},{0,4,1},{0,0,15},
      {1,7,1},{1,6,1},{1,5,1},{1,4,1},{1,0,15},
      {2,0,63},{2,6,3},{3,6,3},{3,4,1},{3,3,1},{3,0,7},
      {4,4,15},{4,0,15},{5,4,15},{5,0,15},
      {6,4,15},{6,0,15},{7,4,15},{7,0,15}
    };
    // build the notes twice, with and without envelopes with a release, and
    // keep whichever needs fewer instruments (the release helps game drivers
    // a lot, but can split music whose volume only decays)
    std::vector<VGMMacroSet> setsAlt;
    std::vector<VGMNoteRec> recsAlt;
    int scaledAlt=0, prefixAlt=0, passAlt=-1;
    bool anyRepeat=false;
    for (int c: tonal) if (onsetsRep[c]!=onsets[c]) anyRepeat=true;
    int periodInexact=0;
    // SID filter and volume: one set for the three voices. each tick they
    // belong to a note: the latest one started on a voice that goes through
    // the filter (or the latest one at all, if none does). that note's
    // instrument gets the changes as macros.
    std::vector<int> sidOwner;
    int sidFirstOwner=-1;
    // what Furnace starts with: cutoff 2047, no resonance, all filter modes
    // (written with the first note), volume 15
    static const int sidInit[4]={2047,0,7,15};
    if (log.hasSID) {
      sidOwner.assign(numTicks,-1);
      std::vector<std::pair<int,int>> allOn;
      for (int c=26; c<29; c++) for (int k: onsets[c]) allOn.push_back(std::make_pair(k,c));
      std::sort(allOn.begin(),allOn.end());
      // (the current note of each voice, and whether it went through the
      // filter when it started)
      int cur[3]={-1,-1,-1};
      bool routed[3]={false,false,false};
      size_t oi=0;
      for (int k=0; k<numTicks; k++) {
        while (oi<allOn.size() && allOn[oi].first==k) {
          int v=allOn[oi].second-26;
          cur[v]=k;
          routed[v]=(sidGlobal[(size_t)k*4+2]>>v)&1;
          oi++;
        }
        int best=-1, bestR=-1;
        for (int v=0; v<3; v++) {
          if (cur[v]<0) continue;
          if (best<0 || cur[v]>=cur[best]) best=v;
          if (routed[v] && (bestR<0 || cur[v]>=cur[bestR])) bestR=v;
        }
        if (bestR>=0) best=bestR;
        sidOwner[k]=(best>=0)?(((26+best)<<24)|cur[best]):-1;
      }
      if (!allOn.empty()) sidFirstOwner=sidOwner[allOn[0].first];
    }
    // SID pulse width that runs on its own across the notes (a sweep the
    // player never starts over): it goes to the pattern (3xxx, 22xx/23xx),
    // not into the instruments, which would otherwise differ by where in the
    // sweep each note starts
    bool sidFreePW[3]={false,false,false};
    if (log.hasSID) {
      for (int v=0; v<3; v++) {
        std::vector<VGMChanTick>& ct=chanTicks[26+v];
        std::vector<int> d;
        for (int k=1; k<numTicks; k++) if (ct[k].duty!=ct[k-1].duty) d.push_back(abs(ct[k].duty-ct[k-1].duty));
        if (d.size()<16) continue;
        std::sort(d.begin(),d.end());
        int md=d[d.size()/2];
        int cnt=0, smooth=0;
        std::set<int> starts;
        for (int b: onsets[26+v]) {
          if (b<1) continue;
          bool pulse=false;
          for (int k=b; k<std::min(numTicks,b+3); k++) if (ct[k].sidCtrl&0x40) pulse=true;
          if (!pulse) continue;
          cnt++;
          starts.insert(ct[b].duty);
          if (abs(ct[b].duty-ct[b-1].duty)<=2*md+1) smooth++;
        }
        if (ctx.sidFreePW && cnt>=4 && smooth*10>=cnt*8 && starts.size()>=4) {
          sidFreePW[v]=true;
          logI("SID import: voice %d: free-running pulse width",v+1);
        }
      }
    }
    // YM2612 channels that are ever panned (they get a pan macro)
    bool opnPanUsed[10]={false,false,false,false,false,false,false,false,false,false};
    if (log.hasOPN) {
      for (int c: tonal) {
        if (!isOPNChan(c)) continue;
        for (int k=0; k<numTicks; k++) {
          const VGMChanTick& t=chanTicks[c][k];
          if (t.aud && t.noise!=3) {
            opnPanUsed[opnIdx(c)]=true;
            break;
          }
        }
      }
    }
    // (pass bit 3: YM2612 vibratos from a close 04xy, see below. it is kept
    // only when it needs fewer instruments than the exact ones)
    int looseVibAlt=0;
    for (int pass=0; pass<16; pass++) {
      const bool allowADSR=!(pass&1);
      const bool useRep=(pass&2);
      curPeriod=(pass&4);
      const bool looseVibPass=(pass&8);
      if (looseVibPass && !log.hasOPN) continue;
      if (useRep && !anyRepeat) continue;
      if (curPeriod && !canPeriod) continue;
      // (the SID always uses periods: its frequencies need it to come out exact)
      if (sidOnly && !curPeriod) continue;
      // (a later subsong follows the pitch mode of the first)
      if (ctx.pitchMode>=0 && curPeriod!=(ctx.pitchMode==1)) continue;
      periodInexact=0;
      sets.clear();
      setMap.clear();
      recs.clear();
      nonVolSets.clear();
      scaledMatches=0;
      loopedLoose=0;
      looseVib=0;
      prefixMatches=0;
      for (int c: tonal) {
        std::vector<VGMChanTick>& ct=chanTicks[c];
        std::vector<int>& on=useRep?onsetsRep[c]:onsets[c];
        const bool is5B=(c>=5 && c<8);
        const bool isSN=(c>=8 && c<12);
        const bool isOPLL=(c>=12 && c<26);
        const bool isSID=(c>=26 && c<29);
        const bool isVRC6=(c>=29 && c<32);
        const bool isOPN=isOPNChan(c);
        const bool isSNES=(c>=44 && c<52) && !log.hasRF;
        const bool isRF=(c>=44 && c<52) && log.hasRF;
        const bool isGB=(c>=52 && c<56);
        size_t i=0;
        while (i<on.size()) {
          int a=on[i];
          int next=(i+1<on.size())?on[i+1]:numTicks;
          // silence at the end of the note
          int s=next;
          while (s>a && !ct[s-1].aud) s--;
          if (s<=a) s=a+1;
          // SNES: drivers that end a note by switching its envelope from
          // ADSR to a decreasing GAIN (instead of a key off): that is the
          // release of Furnace's SNES instruments (ADSR with "sustain" and a
          // release, which plays at the note off). the note ends there
          int snesRelG=-1;
          if (isSNES) {
            const std::vector<unsigned char>& et=snesEnvTick[c-44];
            const unsigned char* q0=&et[(size_t)a*3];
            if (q0[0]&0x80) {
              for (int k=a+1; k<s; k++) {
                const unsigned char* q=&et[(size_t)k*3];
                if (q[0]==q0[0] && q[1]==q0[1]) continue;
                if (!(q[0]&0x80) && (q[2]&0xc0)==0x80) {
                  snesRelG=q[2];
                  s=k;
                }
                break;
              }
            }
          }
          // (SID: a gate off that is only the hard restart before the next
          // note is not the end of this one: Furnace does it)
          if (isSID && hrN>0 && next<numTicks && sidHR[c-26][next] && s>=next-hrN) s=next;
          int rowA=rowOf(a+shift);
          int delayA=(a+shift)-rowStart[rowA];
          // how the note ends when it goes silent before the next one:
          // an OFF if the silence starts on a row of its own, otherwise a note
          // cut (ECxx) on the note's own row, counting from its start. that way
          // the silence never goes into the instrument, and notes that only
          // differ in where they were cut share one instrument.
          // an envelope with a release keeps its fall (and the silence after
          // it) in the instrument; the release itself becomes an E7xx
          int adsrA=-1, adsrEnd=-1;
          bool adsr=false;
          // (not on FM: the end of an FM note is a key off, never a volume)
          if (c!=2 && !isOPLL && !isSID && !isOPN && !isSNES) {
            std::vector<int> rv;
            for (int k=a; k<std::min(next,s+1); k++) rv.push_back(ct[k].vol);
            adsr=vgmFindADSR(rv,adsrA,adsrEnd);
            // only for families that really vary their sustain
            if (adsr) {
              String fk=fmt::sprintf("%d|%d|",c,rv[adsrA]);
              for (int k=0; k<adsrA; k++) fk+=fmt::sprintf("%d,",rv[k]);
              auto fi=adsrFamilies.find(fk);
              if (fi==adsrFamilies.end() || fi->second.size()<2 || !allowADSR) adsr=false;
            }
          }
          bool useOff=false;
          bool cutInNoteRow=false;
          int rowS=0;
          if (s<next && !adsr) {
            rowS=rowOf(s+shift);
            int nextRow=(next<numTicks)?rowOf(next+shift):numRows+1;
            if (rowS>rowA && rowS<nextRow && (s+shift)==rowStart[rowS]) {
              useOff=true;
            } else if (s-a<=255) {
              useOff=true;
              cutInNoteRow=true;
            } else if (rowS<nextRow && !(rowS==rowA && delayA>0)) {
              useOff=true;
            }
          }
          int macroEnd=useOff?s:std::min(next,s+1);
          // SID: after the release (gate off) the note fades out on its own
          // and the player often keeps changing it (vibrato, pulse width):
          // the macros go on until it has faded
          int sidFadeEnd=next;
          if (isSID) {
            static const int relMs[16]={6,24,48,72,114,168,204,240,300,750,1500,2400,3000,9000,15000,24000};
            if (s<next) {
              int rel=ct[std::max(a,s-1)].sidSR&15;
              sidFadeEnd=std::min(next,s+1+(int)ceil((double)relMs[rel]*sub->hz/1000.0));
            }
            // (pitch and pulse width go on during it; Furnace's hard restart
            // only writes the envelope and the control register)
            macroEnd=std::max(macroEnd,std::min(sidFadeEnd,a+255));
            // the hard restart before the next note is Furnace's: the voice's
            // other settings stop changing there
            if (hrN>0 && next<numTicks && sidHR[c-26][next]) sidFadeEnd=std::max(a+1,std::min(sidFadeEnd,next-hrN));
          }
          // (ticks that count: heard, or fading out on the SID)
          auto live=[&](int k) -> bool {
            return ct[k].aud || (isSID && k<macroEnd);
          };

          // FM: if Furnace's own choice of block can't make one of the note's
          // pitches, the instrument takes the driver's block (fixed block)
          curFixedBlock=0;
          if (isOPLL) {
            bool allExact=true;
            bool oneBlock=true;
            for (int k=a; k<macroEnd; k++) {
              if (!ct[k].aud) continue;
              if (!opllExact[ct[k].period&0xffff]) allExact=false;
              if (ct[k].block!=ct[a].block) oneBlock=false;
            }
            if (!allExact && oneBlock) {
              makeBlockTable(ct[a].block);
              curFixedBlock=ct[a].block+1;
            }
          }
          if (isOPN) {
            bool allExact=true;
            bool oneBlock=true;
            for (int k=a; k<macroEnd; k++) {
              if (!ct[k].aud) continue;
              if (!opnExact[ct[k].period&(opnSpan-1)]) allExact=false;
              if (ct[k].block!=ct[a].block) oneBlock=false;
            }
            if (!allExact && oneBlock) {
              makeOPNBlockTable(ct[a].block);
              curFixedBlock=ct[a].block+1;
            }
          }
          // base note: the most common note of the note
          std::map<int,int> noteCount;
          for (int k=a; k<macroEnd; k++) if (live(k)) noteCount[tickNote(c,ct[k])]++;
          int base=tickNote(c,ct[a]);
          int bestCnt=-1;
          for (auto& nc: noteCount) {
            if (nc.second>bestCnt) {
              bestCnt=nc.second;
              base=nc.first;
            }
          }
          if (base<0) base=0;
          if (base>179) base=179;

          // pitch steps inside the note go to the pattern as legato notes
          // (EA01 + note: the pitch changes, the instrument keeps going), so
          // the instrument doesn't need an arpeggio for them. a step is a jump
          // of almost a semitone or more in one tick that then holds (for 2
          // ticks, or until the note ends), in a row of its own.
          // (not on noise channels or drums, whose pitch is a mode; the 5B
          // only takes them when nothing else is left in the arpeggio)
          std::vector<int> segBase(macroEnd-a,base);
          std::vector<int> segIdx(macroEnd-a,0);
          std::vector<int> arpAdd(macroEnd-a,0);
          std::vector<std::pair<int,int>> arpFx;
          std::vector<std::array<int,4>> legato;
          if (c!=3 && c!=11 && c!=55 && (c<21 || c>=26) && c!=4) {
            std::vector<int> offs(macroEnd-a), ns(macroEnd-a);
            int lo=tickOffset(c,ct[a]);
            for (int k=a; k<macroEnd; k++) {
              if (live(k)) lo=tickOffset(c,ct[k]);
              offs[k-a]=lo;
              ns[k-a]=(lo+64)>>7;
            }
            int nextRowL=(next<numTicks)?rowOf(next+shift):numRows;
            int offRow=(s<next && s<numTicks)?rowOf(s+shift):numRows;
            int len=macroEnd-a;
            // a chord arpeggio (note, note+x, note+y, one tick each) goes to
            // the pattern as 00xy, with legato notes where the chord's root
            // moves on a row start
            bool patArp=false;
            if (len>=3) {
              struct RowArp {
                int r, b, x, y;
                bool hasB, hasX, hasY;
              };
              std::vector<RowArp> ra;
              bool ok=true;
              for (int j=0; j<len && ok; j++) {
                int r=rowOf(a+j+shift);
                if (ra.empty() || ra.back().r!=r) {
                  RowArp nr;
                  nr.r=r;
                  nr.b=0;
                  nr.x=0;
                  nr.y=0;
                  nr.hasB=nr.hasX=nr.hasY=false;
                  ra.push_back(nr);
                }
                RowArp& cr=ra.back();
                int st=j%3;
                if (st==0) {
                  if (cr.hasB && cr.b!=ns[j]) ok=false;
                  cr.b=ns[j];
                  cr.hasB=true;
                }
              }
              // the steps, relative to the root of each row
              for (size_t ri=0; ri<ra.size() && ok; ri++) {
                if (!ra[ri].hasB) {
                  if (ri==0) {
                    ok=false;
                  } else {
                    ra[ri].b=ra[ri-1].b;
                    ra[ri].hasB=true;
                  }
                }
              }
              if (ok) {
                size_t ri=0;
                for (int j=0; j<len && ok; j++) {
                  int r=rowOf(a+j+shift);
                  while (ra[ri].r!=r) ri++;
                  RowArp& cr=ra[ri];
                  int st=j%3;
                  if (st==0) continue;
                  int d=ns[j]-cr.b;
                  if (st==1) {
                    if (cr.hasX && cr.x!=d) ok=false;
                    cr.x=d;
                    cr.hasX=true;
                  } else {
                    if (cr.hasY && cr.y!=d) ok=false;
                    cr.y=d;
                    cr.hasY=true;
                  }
                }
              }
              bool any=false;
              for (size_t ri=0; ri<ra.size() && ok; ri++) {
                RowArp& cr=ra[ri];
                if (!cr.hasX) cr.x=(ri>0)?ra[ri-1].x:cr.y;
                if (!cr.hasY) cr.y=(ri>0)?ra[ri-1].y:cr.x;
                if (cr.x<0 || cr.x>15 || cr.y<0 || cr.y>15) ok=false;
                // an empty arpeggio would stop the steps (and their phase)
                if (cr.x==0 && cr.y==0) ok=false;
                if (cr.x!=0 || cr.y!=0) any=true;
                // later rows need their own cell (before the next note or the end)
                if (ri>0 && (cr.r>=nextRowL || cr.r>=offRow)) {
                  bool change=(cr.b!=ra[ri-1].b) || (cr.x!=ra[ri-1].x) || (cr.y!=ra[ri-1].y);
                  if (change) ok=false;
                }
              }
              if (ok && any && ra[0].b>=0 && ra[0].b<=179) {
                patArp=true;
                base=ra[0].b;
                int seg=0;
                size_t ri=0;
                for (int j=0; j<len; j++) {
                  int r=rowOf(a+j+shift);
                  while (ra[ri].r!=r) ri++;
                  if (ri>0 && ra[ri].r!=ra[ri-1].r && j>0 && rowOf(a+j-1+shift)!=r) {
                    // first tick of a row: a new root, a new arpeggio
                    if (ra[ri].b!=ra[ri-1].b) {
                      seg++;
                      int bb=std::max(0,std::min(179,ra[ri].b));
                      legato.push_back({r,0,bb,0});
                    }
                    if (ra[ri].x!=ra[ri-1].x || ra[ri].y!=ra[ri-1].y) arpFx.push_back(std::make_pair(r,(ra[ri].x<<4)|ra[ri].y));
                  }
                  segBase[j]=ra[ri].b;
                  segIdx[j]=seg;
                  int st=j%3;
                  arpAdd[j]=(st==0)?0:((st==1)?ra[ri].x:ra[ri].y);
                }
                arpFx.insert(arpFx.begin(),std::make_pair(rowA,(ra[0].x<<4)|ra[0].y));
              }
            }
            std::vector<int> bounds;
            int lastRowUsed=rowA;
            for (int j=1; j<len && !patArp; j++) {
              if (ns[j]==ns[j-1]) continue;
              if (abs(offs[j]-offs[j-1])<96) continue;
              int hold=1;
              while (j+hold<len && ns[j+hold]==ns[j]) hold++;
              if (hold<2 && j+hold<len) continue;
              int r=rowOf(a+j+shift);
              if (r<=lastRowUsed || r>=nextRowL || r>=offRow) continue;
              if (gridEven && (r&1)) continue;
              int d=(a+j+shift)-rowStart[r];
              if (d>255) continue;
              bounds.push_back(j);
              lastRowUsed=r;
            }
            if (!bounds.empty()) {
              // each piece plays around its most common note
              std::vector<int> starts(1,0);
              for (int j: bounds) starts.push_back(j);
              starts.push_back(len);
              for (size_t sgi=0; sgi+1<starts.size(); sgi++) {
                std::map<int,int> cnt;
                for (int j=starts[sgi]; j<starts[sgi+1]; j++) cnt[ns[j]]++;
                int sb=ns[starts[sgi]], bc=-1;
                for (auto& x: cnt) {
                  if (x.second>bc) {
                    bc=x.second;
                    sb=x.first;
                  }
                }
                if (sgi==0) {
                  sb=base;
                } else {
                  if (sb<0) sb=0;
                  if (sb>179) sb=179;
                  int k=a+starts[sgi];
                  int r=rowOf(k+shift);
                  legato.push_back({r,(k+shift)-rowStart[r],sb,0});
                }
                for (int j=starts[sgi]; j<starts[sgi+1]; j++) {
                  segBase[j]=sb;
                  segIdx[j]=(int)sgi;
                }
              }
              // the first piece keeps the note's own base only if that is
              // still its most common note
              std::map<int,int> cnt0;
              for (int j=0; j<starts[1]; j++) cnt0[ns[j]]++;
              int b0=base, bc0=-1;
              for (auto& x: cnt0) {
                if (x.second>bc0) {
                  bc0=x.second;
                  b0=x.first;
                }
              }
              if (b0<0) b0=0;
              if (b0>179) b0=179;
              base=b0;
              for (int j=0; j<starts[1]; j++) segBase[j]=base;
            }
          }

          std::vector<int> vVol, vDuty, vArp, vPitch, vNoise, vEnv;
          // (on the SN76489, vNoise holds the Game Gear panning)
          int lastOff=tickOffset(c,ct[a]);
          int lastPer=ct[a].period;
          int lastDuty=ct[a].duty;
          int lastNoise=31-ct[a].noise;
          bool usesNoise=false, usesEnv=false;
          int envPeriod=-1;
          // YM2612: after the key off the chip's release plays (Furnace's note
          // off does the same): the volume and panning hold there
          int lastVolO=ct[a].vol, lastPanO=ct[a].noise;
          std::vector<int> vPanO;
          for (int k=a; k<macroEnd; k++) {
            const VGMChanTick& t=ct[k];
            const bool tl=live(k);
            int off=tl?tickOffset(c,t):lastOff;
            int per=tl?t.period:lastPer;
            int duty=tl?t.duty:lastDuty;
            lastOff=off;
            lastPer=per;
            lastDuty=duty;
            int n=(off+64)>>7;
            if (isOPN || isSNES || isRF) {
              if (tl) {
                lastVolO=t.vol;
                lastPanO=t.noise;
              }
              vVol.push_back(lastVolO);
              vPanO.push_back(lastPanO);
            } else {
              vVol.push_back(t.vol);
            }
            vDuty.push_back(duty);
            vArp.push_back(n-segBase[k-a]-arpAdd[k-a]);
            if (usesPer(c) && isSID) {
              // SID: the frequency minus the note's own frequency
              vPitch.push_back(per-ptSIDn.getBase(std::max(0,std::min(179,n))));
            } else if (usesPer(c)) {
              // (a period at the limit of the register is reached by any pitch
              // past it: take the one closest to none)
              int bse=ptPer.getBase(std::max(0,std::min(179,n)));
              int r=bse-(per+1);
              if (per>=2047 && bse-1>=2047) r=0;
              if (per<=0 && bse-1<=0) r=0;
              vPitch.push_back(r);
            } else {
              vPitch.push_back(off-(n<<7));
            }
            if ((isSN && log.snStereo) || (isGB && log.gbStereo)) {
              vNoise.push_back(t.noise);
              usesNoise=true;
            }
            if (is5B) {
              // Furnace's 5B noise macro is 31 minus the noise period
              if (t.aud && (duty&2)) {
                usesNoise=true;
                lastNoise=31-t.noise;
              }
              vNoise.push_back(lastNoise);
              if (t.aud && (duty&4) && !usesEnv) {
                usesEnv=true;
                envPeriod=t.envPeriod;
                vEnv.push_back(t.envShape);
              }
            }
          }
          // the 5B forgets the note of a legato for its arpeggio macro: only
          // when the arpeggio is gone completely
          if (is5B && !legato.empty() && !vgmAllZero(vArp)) {
            legato.clear();
            arpFx.clear();
            for (int k=a; k<macroEnd; k++) {
              vArp[k-a]+=segBase[k-a]-base+arpAdd[k-a];
              segIdx[k-a]=0;
            }
            // (base was the first piece's note)
          }
          // a 5B note without noise doesn't need the noise macro (it is shared,
          // and would change the noise of the other channels)
          if (!usesNoise) vNoise.clear();
          // YM2612 panning: a pan that holds for the whole note goes to the
          // pattern (08xx), so notes on the left and on the right share their
          // instrument; one that moves during the note takes a pan macro
          int notePan=-1;
          // SNES: the same, with 81xx/82xx (left and right, 0-127 each); a
          // pan that moves goes to the pan macros (left and right)
          int notePanLR=-1;
          if (isSNES && !vPanO.empty()) {
            bool same=true;
            for (int v: vPanO) if (v!=vPanO[0]) same=false;
            if (same) {
              notePanLR=vPanO[0];
            } else {
              vNoise.clear();
              vEnv.clear();
              for (int v: vPanO) {
                vNoise.push_back(v>>8);
                vEnv.push_back(v&0xff);
              }
            }
          }
          // RF5C68: a pan that holds goes to 08xy (x: the register's low
          // nibble, the left side), one that moves to the pan macros
          if (isRF && !vPanO.empty()) {
            bool same=true;
            for (int v: vPanO) if (v!=vPanO[0]) same=false;
            if (same) {
              notePanLR=vPanO[0];
            } else {
              vNoise.clear();
              vEnv.clear();
              for (int v: vPanO) {
                vNoise.push_back(v&15);
                vEnv.push_back((v>>4)&15);
              }
            }
          }
          if (isOPN && opnPanUsed[opnIdx(c)] && !vPanO.empty()) {
            bool same=true;
            for (int v: vPanO) if (v!=vPanO[0]) same=false;
            // (0800 is both sides in Furnace: no sides at all takes the macro)
            if (same && vPanO[0]!=0) {
              notePan=vPanO[0];
            } else {
              vNoise=vPanO;
            }
          }
          // FM: a key on inside the note (two notes in one row) is a phase
          // reset macro, which keys the channel on again (slot 5)
          if (isOPLL || isOPN) {
            int lastR=0;
            for (int k=a+1; k<macroEnd; k++) if (ct[k].restart && ct[k].aud) lastR=k-a;
            if (lastR>0) {
              vEnv.assign(lastR+2,0);
              for (int k=a+1; k<=a+lastR; k++) if (ct[k].restart && ct[k].aud) vEnv[k-a]=1;
            }
          }
          // a constant fine tune goes to E5xx in the pattern, the rest to the pitch macro
          // (each legato piece has its own E5xx)
          int fine=0;
          if (c!=3 && c!=55) {
            for (int sg=0; sg<=(int)legato.size(); sg++) {
              std::map<int,int> fc;
              for (size_t k=0; k<vPitch.size(); k++) if (vArp[k]==0 && segIdx[k]==sg) fc[vPitch[k]]++;
              int bc=-1, f0=0;
              for (auto& f: fc) {
                if (f.second>bc) {
                  bc=f.second;
                  f0=f.first;
                }
              }
              // (SID: E5xx reaches 127 register units; the rest stays in the macro)
              if (isSID) f0=std::max(-127,std::min(127,f0));
              for (size_t k=0; k<vPitch.size(); k++) if (segIdx[k]==sg) vPitch[k]-=f0;
              if (sg==0) {
                fine=f0;
              } else {
                legato[sg-1][3]=f0;
              }
            }
          } else {
            vPitch.assign(vPitch.size(),0);
          }
          // SNES: a pitch that only goes a bit past half a semitone (vibrato,
          // a bend) stays in the pitch macro instead of moving the arpeggio
          // by a semitone (notes with the same vibrato then share it)
          if ((isSNES || isRF) && legato.empty() && arpFx.empty()) {
            bool small=true;
            for (size_t j=0; j<vArp.size() && small; j++) {
              if (arpAdd[j]!=0 || vArp[j]<-1 || vArp[j]>1) small=false;
            }
            if (small) {
              for (size_t j=0; j<vArp.size(); j++) {
                vPitch[j]+=vArp[j]*128;
                vArp[j]=0;
              }
            }
          }
          // a pitch offset that writes the same period as none at all is no
          // pitch offset: the period register can't tell them apart, so the
          // instrument doesn't need it (each note would get its own otherwise)
          if (c!=3 && c!=11 && c!=55 && c!=4 && (c<21 || c>=26)) {
            for (size_t j=0; j<vPitch.size(); j++) {
              if (vPitch[j]==0 || !live(a+j)) continue;
              int n=vArp[j]+segBase[j]+arpAdd[j];
              int f=(segIdx[j]==0)?fine:legato[segIdx[j]-1][3];
              if (periodOf(c,n,f)==ct[a+j].period) vPitch[j]=0;
            }
          }
          // period mode: every tick must come out exactly (very low notes or
          // big fine tunes may not fit; the song then stays linear)
          if (curPeriod && (c<3 || c>=26)) {
            for (size_t j=0; j<vPitch.size(); j++) {
              if (!live(a+j)) continue;
              int n=vArp[j]+segBase[j]+arpAdd[j];
              int f=(segIdx[j]==0)?fine:legato[segIdx[j]-1][3];
              bool bad=(n<0 || n>179 || f<-127 || f>127 || periodOf(c,n,f+vPitch[j])!=ct[a+j].period);
              if (bad) {
                periodInexact++;
                break;
              }
            }
          }
          // a vibrato that Furnace's own 04xy makes exactly (songs made in
          // Furnace) goes to the pattern instead of a pitch macro: try every
          // speed and depth, from the note's start or from a row start
          int vibRow=-1, vibVal=0, vibDelay=0;
          // (also on top of a chord arpeggio, 00xy + 04xy)
          if (c!=3 && c!=11 && c!=55 && c!=4 && (c<21 || isVRC6 || isOPN || isSNES || isRF || isGB) && !isSID && legato.empty() && vgmAllZero(vArp) && !vgmAllZero(vPitch) && macroEnd-a>=4) {
            static short vibT[64];
            static bool vibInit=false;
            if (!vibInit) {
              for (int i=0; i<64; i++) vibT[i]=127*sin(((double)i/64.0)*(2*M_PI));
              vibInit=true;
            }
            int len=macroEnd-a;
            int firstOff=0;
            while (firstOff<len && vPitch[firstOff]==0) firstOff++;
            std::vector<int> kvs;
            for (int j=std::min(firstOff,len-1); j>=0; j--) {
              int r=rowOf(a+j+shift);
              if (j==0 || rowStart[r]==a+j+shift) {
                kvs.push_back(j);
                if (kvs.size()>=3) break;
              }
            }
            int nextRowV=(next<numTicks)?rowOf(next+shift):numRows;
            int offRowV=(s<next && s<numTicks)?rowOf(s+shift):numRows;
            int fines[2]={fine,0};
            bool found=false;
            for (int kv: kvs) {
              if (found) break;
              int r=rowOf(a+kv+shift);
              if (kv>0 && (r>=nextRowV || r>=offRowV)) continue;
              for (int fi=0; fi<2 && !found; fi++) {
                if (fi==1 && fines[1]==fines[0]) continue;
                for (int rate=1; rate<16 && !found; rate++) {
                  for (int depth=1; depth<16 && !found; depth++) {
                    bool ok=true;
                    for (int j=0; j<len && ok; j++) {
                      if (!ct[a+j].aud) continue;
                      int vib=0;
                      if (j>=kv) {
                        // (Furnace moves the vibrato on in the same tick it starts)
                        int pos=((j-kv+1)*rate)&63;
                        vib=((depth*vibT[pos]*15)>>4)/15;
                      }
                      if (periodOf(c,base+arpAdd[j],fines[fi]+vib)!=ct[a+j].period) ok=false;
                    }
                    if (ok) {
                      found=true;
                      vibRow=r;
                      vibVal=(rate<<4)|depth;
                      fine=fines[fi];
                    }
                  }
                }
              }
            }
            if (found) vPitch.assign(vPitch.size(),0);
          }
          // YM2612: a driver's vibrato is rarely exactly Furnace's (it moves
          // in steps, as often as the driver runs, in units of the frequency
          // register), so the exact search above misses it and the note keeps
          // a pitch macro of its own (each note's depth is different in
          // Furnace's units, so each one ends up as another instrument). here
          // an 04xy that comes close is good enough: sine or triangle (E303),
          // from the note's start or a row start, around the note's pitch or
          // the middle of the vibrato. close is: the error stays small next to
          // the vibrato's own size (and under a few cents where it is small),
          // so it sounds like the same vibrato. otherwise the macro stays
          if (isOPN && vibRow<0 && legato.empty() && arpFx.empty() && macroEnd-a>=8 && looseVibPass) {
            const int len=macroEnd-a;
            bool smallArp=true;
            bool anyPitch=false;
            for (int j=0; j<len && smallArp; j++) {
              if (arpAdd[j]!=0 || vArp[j]<-1 || vArp[j]>1) smallArp=false;
              if (vArp[j]!=0 || vPitch[j]!=0) anyPitch=true;
            }
            if (smallArp && anyPitch) {
              // the pitch the note plays, tick by tick (-1: not heard)
              std::vector<int> tgt(len,0);
              std::vector<bool> heard(len,false);
              int nHeard=0;
              for (int j=0; j<len; j++) {
                if (!ct[a+j].aud) continue;
                tgt[j]=tickOffset(c,ct[a+j]);
                heard[j]=true;
                nHeard++;
              }
              static short vibS[64];
              static bool vibSInit=false;
              if (!vibSInit) {
                for (int i=0; i<64; i++) vibS[i]=127*sin(((double)i/64.0)*(2*M_PI));
                vibSInit=true;
              }
              auto vibOut=[&](int shape, int pos) -> int {
                if (shape==3) {
                  int v=(pos&31);
                  if (pos&16) v=32-(pos&31);
                  if (pos&32) v=-v;
                  return v<<3;
                }
                return vibS[pos&63];
              };
              const int b7=base<<7;
              // a pitch that hardly moves (a few cents at most, a rounding
              // of the driver's) is no vibrato: the note's own pitch
              int lo=1<<30, hi=-(1<<30);
              for (int j=0; j<len; j++) {
                if (!heard[j]) continue;
                lo=std::min(lo,tgt[j]-b7);
                hi=std::max(hi,tgt[j]-b7);
              }
              if (nHeard>0 && hi-lo<=6 && lo>=-127 && hi<=127) {
                fine=(lo+hi)/2;
                vPitch.assign(vPitch.size(),0);
                vArp.assign(vArp.size(),0);
                looseVib++;
              } else if (nHeard>0) {
                int firstOff=0;
                while (firstOff<len && (!heard[firstOff] || abs(tgt[firstOff]-b7-fine)<=3)) firstOff++;
                std::vector<int> kvs;
                for (int j=std::min(firstOff,len-1); j>=0; j--) {
                  int r=rowOf(a+j+shift);
                  if (j==0 || rowStart[r]==a+j+shift) {
                    kvs.push_back(j);
                    if (kvs.size()>=2) break;
                  }
                }
                // (or on the very tick it starts, with an EDxx on its row: a
                // driver's vibrato delay rarely ends on a row start. only on a
                // row after the note's own, which the delay would move too)
                for (int j=firstOff-1; j<=firstOff+1; j++) {
                  if (j<=0 || j>=len) continue;
                  int r=rowOf(a+j+shift);
                  if (r<=rowA || (a+j+shift)-rowStart[r]>255) continue;
                  if (std::find(kvs.begin(),kvs.end(),j)==kvs.end()) kvs.push_back(j);
                }
                int nextRowV=(next<numTicks)?rowOf(next+shift):numRows;
                int offRowV=(s<next && s<numTicks)?rowOf(s+shift):numRows;
                // the pitches to center it on: the note's (before the vibrato)
                // and the middle of the vibrato
                int vlo=1<<30, vhi=-(1<<30);
                for (int j=firstOff; j<len; j++) {
                  if (!heard[j]) continue;
                  vlo=std::min(vlo,tgt[j]-b7);
                  vhi=std::max(vhi,tgt[j]-b7);
                }
                std::vector<int> fines;
                fines.push_back(fine);
                if (vlo<=vhi && (vlo+vhi)/2!=fine) fines.push_back((vlo+vhi)/2);
                double bestRms=1e30;
                int bestKv=-1, bestF=0, bestVal=0;
                for (int kv: kvs) {
                  int r=rowOf(a+kv+shift);
                  if (kv>0 && (r>=nextRowV || r>=offRowV)) continue;
                  for (int f: fines) {
                    if (f<-127 || f>127) continue;
                    for (int shape=0; shape<=3; shape+=3) {
                      for (int rate=1; rate<16; rate++) {
                        for (int depth=1; depth<16; depth++) {
                          // (the errors, and the vibrato's size, leaving out a
                          // few odd ticks: a driver often writes the next
                          // note's pitch a tick before its key on)
                          std::vector<int> errs;
                          errs.reserve(len);
                          std::vector<int> amps;
                          amps.reserve(len);
                          for (int j=0; j<len; j++) {
                            if (!heard[j]) continue;
                            int vib=0;
                            if (j>=kv) {
                              int pos=((j-kv+1)*rate)&63;
                              vib=((depth*vibOut(shape,pos)*15)>>4)/15;
                            }
                            errs.push_back(abs(tgt[j]-(b7+f+vib)));
                            amps.push_back(abs(tgt[j]-b7-f));
                          }
                          const int odd=std::max(1,(int)errs.size()/50);
                          std::sort(errs.begin(),errs.end());
                          std::sort(amps.begin(),amps.end());
                          const int keepN=std::max(1,(int)errs.size()-odd);
                          int maxE=errs[keepN-1];
                          int amp=amps[std::max(0,(int)amps.size()-1-odd)];
                          if (maxE>64) continue;
                          double se=0;
                          for (int q=0; q<keepN; q++) se+=(double)errs[q]*errs[q];
                          double rms=sqrt(se/(double)keepN);
                          // (close next to the vibrato's size, or a few cents)
                          if (rms>std::max(4.0,0.15*amp)) continue;
                          if (maxE>std::max(10,(int)(0.35*amp))) continue;
                          if (rms<bestRms) {
                            bestRms=rms;
                            bestKv=kv;
                            bestF=f;
                            bestVal=(shape<<8)|(rate<<4)|depth;
                          }
                        }
                      }
                    }
                  }
                }
                if (bestKv>=0) {
                  vibRow=rowOf(a+bestKv+shift);
                  vibDelay=(a+bestKv+shift)-rowStart[vibRow];
                  vibVal=bestVal;
                  fine=bestF;
                  vPitch.assign(vPitch.size(),0);
                  vArp.assign(vArp.size(),0);
                  looseVib++;
                }
              }
            }
          }
          // what this note plays, tick by tick (to compare with instruments)
          std::vector<int> rawV[6]={vVol,vDuty,vArp,vPitch,vNoise,vEnv};
          VGMMacroSet ms;
          ms.type=isGB?((c==54)?17:(c==55)?18:16):isRF?15:isSNES?14:isOPN?((c>=39)?13:12):isVRC6?((c==31)?11:10):isSID?9:(isOPLL?((c>=21)?8:7):(isSN?((c==11)?6:5):(is5B?4:((c<2)?0:c))));
          ms.fm=(isOPLL || isOPN || isSNES || isRF)?ct[a].fm:-1;
          if (isSNES) {
            // the envelope registers changing during the note in a way an
            // instrument can't do (another mode, other ADSR rates): the gain
            // macro does it. where the driver uses GAIN, its own GAIN byte
            // (exact, and the same in every note); where it uses ADSR, the
            // envelope's level as a direct gain
            const VGMFMSound& fs=fmSounds[ct[a].fm];
            const std::vector<unsigned char>& et=snesEnvTick[c-44];
            bool envMoves=false;
            {
              int lenE=std::min(macroEnd-a,255);
              while (lenE>1 && !ct[a+lenE-1].aud) lenE--;
              for (int k=a; k<a+lenE && !envMoves; k++) {
                const unsigned char* q=&et[(size_t)k*3];
                if (q[0]!=fs.snesEnv[0]) envMoves=true;
                else if ((q[0]&0x80) && q[1]!=fs.snesEnv[1]) envMoves=true;
              }
            }
            if (!envMoves && snesRelG>=0 && useOff) {
              ms.fmKey+=fmt::sprintf("R%d;",snesRelG);
            }
            if (envMoves) {
              int lenM=std::min(macroEnd-a,255);
              while (lenM>1 && !ct[a+lenM-1].aud) lenM--;
              std::vector<int> seq;
              for (int k=a; k<a+lenM; k++) {
                const unsigned char* q=&et[(size_t)k*3];
                int gv=(q[0]&0x80)?(ct[k].envx&127):q[2];
                // (the first tick: the note starts at that level. an
                // instant attack is at its top there, not where it decayed
                // to by the middle of the tick)
                if (k==a && (q[0]&0x80) && (q[0]&15)==15) gv=127;
                seq.push_back(gv);
              }
              int lp=vgmCompress(seq);
              ms.fmMac.push_back(std::make_pair(0,seq));
              ms.fmMacLoop.push_back(lp);
              ms.fmKey+=fmt::sprintf("E;g:%sL%d;",vgmMacroKey(seq),lp);
            } else if (!(fs.snesEnv[0]&0x80)) {
              const std::vector<unsigned char>& et=snesEnvTick[c-44];
              int lenM=std::min(macroEnd-a,255);
              while (lenM>1 && !ct[a+lenM-1].aud) lenM--;
              std::vector<int> seq;
              bool changes=false;
              bool adsr=false;
              for (int k=a; k<a+lenM; k++) {
                int gv=et[(size_t)k*3+2];
                if (et[(size_t)k*3]&0x80) adsr=true;
                if (!seq.empty() && gv!=seq[0]) changes=true;
                seq.push_back(gv);
              }
              if (changes && !adsr) {
                int lp=vgmCompress(seq);
                ms.fmMac.push_back(std::make_pair(0,seq));
                ms.fmMacLoop.push_back(lp);
                ms.fmKey+=fmt::sprintf("g:%sL%d;",vgmMacroKey(seq),lp);
              }
            }
          }
          if (curFixedBlock>0) ms.fmKey=fmt::sprintf("B%d;",curFixedBlock);
          if (isOPN) {
            // changes to the channel's patch during the note: FM macros
            // (fields: see opnField)
            // (after the key off the patch holds: drivers often load the
            // next note's patch there, which isn't part of this note)
            const unsigned char* pp=opnPatch[opnIdx(c)].data();
            int lenM=std::min(macroEnd-a,255);
            while (lenM>1 && !ct[a+lenM-1].aud) lenM--;
            for (int f=0; f<48 && lenM>1; f++) {
              std::vector<int> seq;
              bool changes=false;
              for (int k=a; k<a+lenM; k++) {
                int v=opnField(&pp[(size_t)k*32],f);
                if (!seq.empty() && v!=seq[0]) changes=true;
                seq.push_back(v);
              }
              if (!changes) continue;
              int lp=vgmCompress(seq);
              ms.fmMac.push_back(std::make_pair(f,seq));
              ms.fmMacLoop.push_back(lp);
              ms.fmKey+=fmt::sprintf("f%d:%sL%d;",f,vgmMacroKey(seq),lp);
            }
          }
          if (isOPLL && c<21) {
            // a note that switches to the custom patch takes its registers
            for (int k=a; k<macroEnd; k++) {
              if (ct[k].aud && ct[k].duty==0) {
                ms.fm=ct[k].fm;
                break;
              }
            }
            // changes to the custom patch while this note owns it
            if (ct[a].duty==0) {
              int wEnd=numTicks;
              auto wi=std::upper_bound(customOnsets.begin(),customOnsets.end(),a);
              if (wi!=customOnsets.end()) wEnd=*wi;
              int lenM=std::min(std::min(macroEnd,wEnd)-a,255);
              for (int f=0; f<24 && lenM>1; f++) {
                std::vector<int> seq;
                bool changes=false;
                for (int k=a; k<a+lenM; k++) {
                  int v=(opllCust[(size_t)k*8+fmField[f][0]]>>fmField[f][1])&fmField[f][2];
                  if (!seq.empty() && v!=seq[0]) changes=true;
                  seq.push_back(v);
                }
                if (!changes) continue;
                int lp=vgmCompress(seq);
                ms.fmMac.push_back(std::make_pair(f,seq));
                ms.fmMacLoop.push_back(lp);
                ms.fmKey+=fmt::sprintf("f%d:%sL%d;",f,vgmMacroKey(seq),lp);
              }
            }
          }
          if (isSID && sidFreePW[c-26]) {
            vDuty.assign(vDuty.size(),0);
            ms.fmKey+="P;";
          }
          if (isSID) {
            // the voice's own settings: what they are when the note starts
            // (the instrument's), and changes after that (macros), also
            // while the note fades out after its release
            const int sv=c-26;
            auto fieldOf=[&](int f, int k) -> int {
              const VGMChanTick& t=ct[k];
              switch (f) {
                case 0: return t.sidCtrl>>4;
                case 1: return (t.sidCtrl&0x0e)|1;
                case 2: return t.sidAD>>4;
                case 3: return t.sidAD&15;
                case 4: return t.sidSR>>4;
                case 5: return t.sidSR&15;
                case 6: return (sidGlobal[(size_t)k*4+2]>>sv)&1;
              }
              return 0;
            };
            int lenV=std::min(sidFadeEnd-a,255);
            if (lenV<1) lenV=1;
            int base[7];
            for (int f=0; f<7; f++) base[f]=fieldOf(f,a);
            ms.fmKey+=fmt::sprintf("S%d,%d,%d,%d,%d,%d,%d;",base[0],base[1],base[2],base[3],base[4],base[5],base[6]);
            for (int f=0; f<7; f++) {
              std::vector<int> seq;
              bool changes=false;
              for (int k=a; k<a+lenV; k++) {
                int v=fieldOf(f,k);
                if (v!=base[f]) changes=true;
                seq.push_back(v);
              }
              // Furnace starts with the test bit off and, with a sustain of
              // 15, a decay of 0: those take a macro to come out right
              bool need=changes;
              if (f==1 && (base[1]&8)) need=true;
              if (f==3 && base[4]==15 && base[3]!=0) need=true;
              if (!need) continue;
              int lp=vgmCompress(seq);
              ms.fmMac.push_back(std::make_pair(f,seq));
              ms.fmMacLoop.push_back(lp);
              ms.fmKey+=fmt::sprintf("f%d:%sL%d;",f,vgmMacroKey(seq),lp);
            }
            // the filter and the volume are shared by the voices: their changes
            // belong to the note that owns them (see sidOwner)
            int ownEnd=a;
            while (ownEnd<std::min(next,a+255) && sidOwner[ownEnd]==((c<<24)|a)) ownEnd++;
            if (ownEnd>a) {
              for (int f=10; f<14; f++) {
                std::vector<int> seq;
                bool changes=false;
                for (int k=a; k<ownEnd; k++) {
                  const unsigned char* g=&sidGlobal[(size_t)k*4];
                  int v=0;
                  switch (f) {
                    case 10: v=(g[0]&7)|(g[1]<<3); break;
                    case 11: v=g[2]>>4; break;
                    case 12: v=g[3]>>4; break;
                    case 13: v=g[3]&15; break;
                  }
                  if (!seq.empty() && v!=seq[0]) changes=true;
                  seq.push_back(v);
                }
                if (f==13 && log.sidDigi) continue;
                // (or if it is not what it was before the note: a player that
                // sets it as the note starts. before the first note it is
                // what Furnace starts with)
                int before=sidInit[f-10];
                if (a>0 && sidFirstOwner!=((c<<24)|a)) {
                  const unsigned char* g=&sidGlobal[(size_t)(a-1)*4];
                  switch (f) {
                    case 10: before=(g[0]&7)|(g[1]<<3); break;
                    case 11: before=g[2]>>4; break;
                    case 12: before=g[3]>>4; break;
                    case 13: before=g[3]&15; break;
                  }
                }
                if (!changes && seq[0]==before) continue;
                // (no loop: the macro must stop writing once the note no
                // longer owns these, so it only holds its last value)
                vgmTrim(seq);
                int lp=-1;
                ms.fmMac.push_back(std::make_pair(f,seq));
                ms.fmMacLoop.push_back(lp);
                ms.fmKey+=fmt::sprintf("f%d:%sL%d;",f,vgmMacroKey(seq),lp);
              }
            }
          }
          int relTick=-1;
          if (adsr && adsrEnd<(int)vVol.size()) {
            // attack | sustain sustain / fall: the loop (on the sustain) comes
            // before the release point, so after the fall the macro just ends
            // and holds (with the loop on the release point itself, Furnace
            // would jump back to the sustain once the fall is over)
            std::vector<int> av(vVol.begin(),vVol.begin()+adsrA);
            av.push_back(vVol[adsrA]);
            av.push_back(vVol[adsrA]);
            std::vector<int> tail(vVol.begin()+adsrEnd+1,vVol.end());
            vgmTrim(tail);
            av.insert(av.end(),tail.begin(),tail.end());
            std::vector<int> chk;
            int chkLen=(int)vVol.size()+64;
            vgmSimMacro(av,adsrA,adsrA+1,adsrEnd,chkLen,chk);
            bool same=true;
            for (int k=0; k<chkLen; k++) {
              if (chk[k]!=vVol[std::min(k,(int)vVol.size()-1)]) {
                same=false;
                break;
              }
            }
            if (same && av.size()<=255 && adsrEnd>=1) {
              ms.v[0]=av;
              ms.loop[0]=adsrA;
              ms.rel[0]=adsrA+1;
              relTick=adsrEnd;
            } else {
              adsr=false;
            }
          } else {
            adsr=false;
          }
          if (!adsr) ms.loop[0]=vgmCompress(vVol);
          else vVol=ms.v[0];
          ms.loop[1]=vgmCompress(vDuty);
          ms.loop[2]=vgmCompress(vArp);
          ms.loop[3]=vgmCompress(vPitch);
          ms.loop[4]=vNoise.empty()?-1:vgmCompress(vNoise);
          ms.loop[5]=-1;
          // a macro holds at most 255 ticks: if something still changes after
          // that, split the note at a row start and go on from there.
          // (not on the SID, where a new note would start the envelope over:
          // what doesn't fit is cut)
          bool dutyCut=false;
          if (isSID) {
            if (vDuty.size()>255 && ms.loop[1]<0) dutyCut=true;
            std::vector<int>* vv[4]={&vVol,&vDuty,&vArp,&vPitch};
            for (int m=0; m<4; m++) {
              if (vv[m]->size()>255) vv[m]->resize(255);
              if (ms.loop[m]>=(int)vv[m]->size()) ms.loop[m]=-1;
            }
          }
          // (nor on the YM2612, where a new note keys on again: an attack the
          // song doesn't have. the macros stop at 255 ticks, and the volume
          // after that goes on in the volume column, on the row starts)
          // first, a macro too long for that may still repeat, but not
          // exactly: a driver slower than the tick rate (60 Hz on 240 Hz
          // ticks) whose steps the VGM's timing rounds to one tick more or
          // less. that one loops too (a tick of difference isn't heard)
          {
            std::vector<int>* vv[5]={&vVol,&vDuty,&vArp,&vPitch,&vNoise};
            for (int m=0; m<5; m++) {
              if (vv[m]->size()<=255) continue;
              std::vector<int> lv=*vv[m];
              int lp=vgmLoopifyLoose(lv);
              if (lp>=0 && lv.size()<=255) {
                *vv[m]=lv;
                ms.loop[m]=lp;
                loopedLoose++;
              }
            }
          }
          std::vector<std::pair<int,int>> volTail;
          if (isOPN && (vVol.size()>255 || vDuty.size()>255 || vArp.size()>255 || vPitch.size()>255 || vNoise.size()>255 || vEnv.size()>255)) {
            std::vector<int>* vv[6]={&vVol,&vDuty,&vArp,&vPitch,&vNoise,&vEnv};
            for (int m=0; m<6; m++) {
              if (vv[m]->size()>255) {
                vv[m]->resize(255);
                if (m<5 && ms.loop[m]>=255) ms.loop[m]=-1;
              }
            }
            if (ms.loop[0]<0 && macroEnd-a>255) {
              int lastV=vVol.back();
              const int endT=useOff?s:macroEnd;
              for (int r=rowOf(a+255+shift); r<numRows; r++) {
                int k=rowStart[r]-shift;
                if (k<a+255) continue;
                if (k>=endT) break;
                if (!ct[k].aud) break;
                if (ct[k].vol==lastV) continue;
                volTail.push_back(std::make_pair(r,ct[k].vol));
                lastV=ct[k].vol;
              }
            }
          }
          if (vVol.size()>255 || vDuty.size()>255 || vArp.size()>255 || vPitch.size()>255 || vNoise.size()>255 || vEnv.size()>255) {
            int limit=a+255;
            int splitRow=rowOf(limit+shift);
            int split=rowStart[splitRow]-shift;
            if (split<=a) split=limit;
            on.insert(on.begin()+i+1,split);
            continue;
          }
          // (the OPLL drums always get one: Furnace doesn't set a drum's
          // volume again on a new note, only when its volume macro runs)
          bool volMacro=((vVol.size()>1) || adsr || (c>=21 && c<26)) && !isSID;
          // (the VRC6 saw's volume goes to 63)
          const int fullVol=isRF?255:(isOPN || isSNES)?127:((c==31)?63:15);
          int noteVol=volMacro?fullVol:vVol[0];
          if (c==2 && !volMacro) noteVol=15;
          ms.has[0]=volMacro;
          // the triangle, the SN squares and FM have no duty/mode macro
          // (FM: the patch number, the wave macro if it changes)
          ms.has[1]=(c!=2 && c!=31 && !(isSN && c!=11) && !(isOPLL && c>=21) && !isOPN && !isSNES && !isRF);
          // (SID: no volume of its own, the pulse width is the duty macro)
          if (isSID) {
            noteVol=15;
            ms.has[1]=true;
          }
          ms.has[2]=!vgmAllZero(vArp);
          ms.has[3]=!vgmAllZero(vPitch);
          ms.has[4]=!vNoise.empty();
          ms.has[5]=!vEnv.empty();
          ms.v[0]=vVol;
          ms.v[1]=vDuty;
          ms.v[2]=vArp;
          ms.v[3]=vPitch;
          ms.v[4]=vNoise;
          ms.v[5]=vEnv;
          String key=fmt::sprintf("%d|%s|%s|",ms.type,(ms.fm>=0)?fmSoundKeys[ms.fm]:String("-"),ms.fmKey);
          for (int m=0; m<6; m++) {
            key+=ms.has[m]?(vgmMacroKey(ms.v[m])+fmt::sprintf("L%dR%d;",ms.loop[m],ms.rel[m])):String("-;");
          }
          int setIndex=-1;
          auto si=setMap.find(key);
          String nonVolKey=fmt::sprintf("%d|%d|%s|",ms.type,ms.fm,ms.fmKey);
          for (int m=1; m<6; m++) {
            nonVolKey+=ms.has[m]?(vgmMacroKey(ms.v[m])+fmt::sprintf("L%d;",ms.loop[m])):String("-;");
          }
          // how long this note hears its instrument: until the note off, or
          // until the next note (a held value counts too)
          int obs=std::min(512,(useOff?s:next)-a);
          // (the SID goes on sounding while it fades out)
          if (isSID) obs=std::min(512,std::max(obs,macroEnd-a));
          // what instrument x plays for macro m over n ticks (released at relT),
          // with an absent macro playing 0, or the volume column for the volume
          auto simOut=[&](const VGMMacroSet& x, int m, int relT, int n, int colVol, std::vector<int>& out) {
            if (!x.has[m]) {
              out.assign(n,(m==0)?colVol:0);
              return;
            }
            vgmSimMacro(x.v[m],x.loop[m],x.rel[m],relT,n,out);
          };
          // this note's own values over n ticks (holding the last one)
          auto rawAt=[&](int m, int k) -> int {
            if (m==0 && !ms.has[0]) return noteVol;
            if (!ms.has[m]) return 0;
            const std::vector<int>& r=rawV[m];
            if (r.empty()) return 0;
            return r[std::min(k,(int)r.size()-1)];
          };
          // can this note play instrument x? returns the release tick it needs
          // (-1: none), or -2 if it can't. the note only has to sound the same
          // for as long as it is heard.
          // SNES: how far two pitches may be and still count as the same:
          // 8/128 of a semitone (about 6 cents, around the smallest change
          // of pitch the ear tells apart), or a step of the pitch register
          // (2216/P at pitch P) plus one if that is more (at most 12).
          // drivers' vibratos round differently on each note, and their
          // depth goes in steps of the register
          int snesPitchTol=8;
          if (isSNES) {
            int P=std::max(1,ct[a].period);
            snesPitchTol=std::max(8,std::min(12,(int)ceil(2216.0/(double)P)+1));
          }
          auto playsAs=[&](const VGMMacroSet& x) -> int {
            if (x.type!=ms.type || x.fm!=ms.fm || x.fmKey!=ms.fmKey) return -2;
            // the noise frequency is shared by the 3 channels: never add or drop it
            if (x.has[4]!=ms.has[4] || x.has[5]!=ms.has[5] || x.has[1]!=ms.has[1]) return -2;
            // a constant volume can't stand in for an envelope
            if (ms.has[0] && !x.has[0]) return -2;
            std::vector<int> out;
            int rel=-1;
            // the volume decides if (and when) the note is released
            if (ms.has[0] || x.has[0]) {
              simOut(x,0,-1,obs,15,out);
              int k0=-1;
              for (int k=0; k<obs; k++) {
                if (out[k]!=rawAt(0,k)) {
                  k0=k;
                  break;
                }
              }
              if (k0>=0) {
                if (x.rel[0]<0 || k0<2) return -2;
                rel=k0-1;
                simOut(x,0,rel,obs,15,out);
                for (int k=0; k<obs; k++) if (out[k]!=rawAt(0,k)) return -2;
              }
            }
            for (int m=1; m<6; m++) {
              if (!x.has[m] && !ms.has[m]) continue;
              simOut(x,m,rel,obs,15,out);
              // (SNES: a pitch within snesPitchTol is the same pitch)
              if ((isSNES || isRF) && m==3) {
                for (int k=0; k<obs; k++) if (abs(out[k]-rawAt(m,k))>snesPitchTol) return -2;
                continue;
              }
              for (int k=0; k<obs; k++) if (out[k]!=rawAt(m,k)) return -2;
            }
            return rel;
          };
          // would every note of instrument y still sound the same (each with
          // its own release, for as long as it is heard) with instrument x?
          auto coversNotes=[&](const VGMMacroSet& x, const VGMMacroSet& y) -> bool {
            if (x.type!=y.type || x.fm!=y.fm || x.fmKey!=y.fmKey) return false;
            if (x.has[4]!=y.has[4] || x.has[5]!=y.has[5] || x.has[1]!=y.has[1] || x.has[0]!=y.has[0]) return false;
            // a released note needs x to have a release point too
            std::vector<int> ox, oy;
            for (auto& h: y.heard) {
              if (h.first>=0 && x.rel[0]<0) return false;
              for (int m=0; m<6; m++) {
                if (!x.has[m] && !y.has[m]) continue;
                simOut(x,m,h.first,h.second,15,ox);
                simOut(y,m,h.first,h.second,15,oy);
                if (ox!=oy) return false;
              }
            }
            return true;
          };
          if (si!=setMap.end() && sets[si->second].maxObs>=0) {
            setIndex=si->second;
            sets[setIndex].uses++;
            if (obs>sets[setIndex].maxObs) sets[setIndex].maxObs=obs;
          }
          if (setIndex<0) {
            // a note cut short by the next one, or let go earlier, only hears
            // part of its instrument: any instrument that plays the same way
            // will do (with the release where this note needs it). and a longer
            // note may extend an instrument whose notes never got that far.
            for (size_t j=0; j<sets.size() && setIndex<0; j++) {
              VGMMacroSet& cs=sets[j];
              if (cs.type!=ms.type) continue;
              int r=playsAs(cs);
              if (r>=-1) {
                setIndex=(int)j;
                relTick=r;
                if (!ms.has[0] && cs.has[0]) noteVol=fullVol;
              } else if (cs.ins<0 && coversNotes(ms,cs)) {
                // take the longer shape (or the one with a release) for this
                // instrument: its notes so far never get past where they agree
                for (auto it=setMap.begin(); it!=setMap.end();) {
                  if (it->second==(int)j) {
                    it=setMap.erase(it);
                  } else {
                    ++it;
                  }
                }
                for (int m=0; m<6; m++) {
                  cs.v[m]=ms.v[m];
                  cs.loop[m]=ms.loop[m];
                  cs.rel[m]=ms.rel[m];
                  cs.has[m]=ms.has[m];
                }
                cs.key=key;
                cs.reshaped=true;
                setMap[key]=(int)j;
                setIndex=(int)j;
              }
            }
            if (setIndex>=0) {
              sets[setIndex].uses++;
              if (obs>sets[setIndex].maxObs) sets[setIndex].maxObs=obs;
              prefixMatches++;
            }
          }
          if (setIndex>=0) {
            // found
          } else if (volMacro && c!=2 && c!=31) {
            // the same envelope, only quieter: reuse it with a lower volume column.
            // the 2A03 plays macro*column/15 (rounded down), the 5B plays
            // macro-(15-column): check that it comes out exact.
            auto cands=nonVolSets.find(nonVolKey);
            if (cands!=nonVolSets.end()) {
              for (int j: cands->second) {
                VGMMacroSet& cs=sets[j];
                if (!cs.has[0] || cs.reshaped || cs.rel[0]>=0 || ms.rel[0]>=0) continue;
                int span=(int)std::max(cs.v[0].size(),ms.v[0].size())+64;
                for (int V=fullVol-1; V>=1 && setIndex<0; V--) {
                  bool ok=true;
                  for (int k=0; k<span && ok; k++) {
                    int mk=vgmMacroValueAt(cs.v[0],cs.loop[0],k);
                    int ak=vgmMacroValueAt(ms.v[0],ms.loop[0],k);
                    int out=(is5B || isSN || isOPLL || isOPN)?std::max(0,mk-(fullVol-V)):((V*mk)/(isRF?255:isSNES?127:15));
                    if (out!=ak) ok=false;
                  }
                  if (ok) {
                    setIndex=j;
                    noteVol=V;
                    cs.uses++;
                    if (obs>cs.maxObs) cs.maxObs=obs;
                    scaledMatches++;
                  }
                }
                if (setIndex>=0) break;
              }
            }
          }
          if (setIndex>=0) {
            // found
          } else {
            ms.uses=1;
            ms.weight=0;
            ms.maxObs=obs;
            ms.firstUse=(int)recs.size();
            ms.key=key;
            // the same shape in a subsong imported before: reuse its instrument
            auto prev=ctx.insByKey.find(key);
            ms.ins=(prev!=ctx.insByKey.end())?prev->second:-1;
            setIndex=(int)sets.size();
            sets.push_back(ms);
            setMap[key]=setIndex;
            nonVolSets[nonVolKey].push_back(setIndex);
          }

          sets[setIndex].weight+=macroEnd-a;
          if (relTick>=0) sets[setIndex].anyRelease=true;
          sets[setIndex].heard.push_back(std::make_pair(relTick,obs));
          VGMNoteRec nr;
          nr.c=c;
          nr.rowA=rowA;
          nr.delayA=delayA;
          nr.base=base;
          nr.fine=fine;
          nr.noteVol=noteVol;
          nr.useOff=useOff;
          nr.rowS=rowS;
          nr.delayS=useOff?((s+shift)-rowStart[rowS]):0;
          nr.cutInNoteRow=cutInNoteRow;
          nr.cutTicks=s-a;
          nr.relTick=relTick;
          nr.obs=obs;
          nr.sidOnset=isSID?a:-1;
          nr.sidDutyCut=isSID && dutyCut;
          nr.set=setIndex;
          nr.envPeriod=usesEnv?envPeriod:-1;
          nr.legato=legato;
          nr.volTail=volTail;
          nr.arpFx=arpFx;
          nr.vibRow=vibRow;
          nr.vibVal=vibVal;
          nr.vibDelay=vibDelay;
          nr.pan=(notePan>=0)?notePan:(vNoise.empty() || !isOPN)?-1:-2;
          if (isSNES || isRF) nr.pan=(notePanLR>=0)?notePanLR:(vNoise.empty()?-1:-2);
          recs.push_back(nr);
          i++;
        }
      }


      // keep the best so far in setsAlt (period mode only if every note
      // comes out exact with it)
      if (curPeriod && periodInexact>0) {
        logI("VGM import: periods for the pitch would change %d notes, staying linear",periodInexact);
        continue;
      }
      // (and only if it saves a lot: at least 10% of the instruments, or
      // the song doesn't fit in 256 instruments otherwise)
      if (curPeriod && passAlt>=0 && !(passAlt&4)) {
        bool overLimit=(setsAlt.size()+ds.ins.size()>256);
        if (!overLimit && sets.size()*10>setsAlt.size()*9) continue;
      }
      if (passAlt<0 || sets.size()<setsAlt.size()) {
        setsAlt.swap(sets);
        recsAlt.swap(recs);
        scaledAlt=scaledMatches;
        prefixAlt=prefixMatches;
        looseVibAlt=looseVib;
        passAlt=pass;
      }
    }
    sets.swap(setsAlt);
    recs.swap(recsAlt);
    scaledMatches=scaledAlt;
    prefixMatches=prefixAlt;
    looseVib=(passAlt&8)?looseVibAlt:0;
    if (!(passAlt&1)) logI("VGM import: using envelopes with release");
    if (passAlt&2) {
      logI("VGM import: using repeated notes");
      for (int c: tonal) onsets[c]=onsetsRep[c];
    }
    curPeriod=(passAlt&4);
    if (ctx.pitchMode<0) {
      ctx.pitchMode=curPeriod?1:0;
      if (curPeriod) {
        ds.compatFlags.linearPitch=false;
        logI("VGM import: using periods for the pitch (non-linear pitch): fewer instruments");
      }
    }

    // --- the whole song at once: an instrument that is only a cut short or
    // quieter version of another one goes away. (while the notes were being
    // read this only worked when the longer/louder one came first.) every
    // note of it must sound exactly the same with the other instrument and
    // its own volume column, for as long as the note is heard.
    int absorbed=0;
    {
      // volume a note plays: column v with macro value m
      auto scaleVol=[&](int type, int v, int m) -> int {
        if (type==0 || type==3 || type==10) return (v*m)/15;
        // (the VRC6 saw scales by 63: not with this)
        if (type==11) return -1000;
        // (YM2612: the volume goes to 127)
        if (type==12 || type==13) return std::max(0,m-(127-v));
        // (the SNES's volume goes to 127, scaled)
        if (type==14) return (v*m)/127;
        // (the RF5C68's goes to 255, scaled)
        if (type==15) return (v*m)/255;
        // (the Game Boy's, scaled)
        if (type>=16 && type<=18) return (v*m)/15;
        return std::max(0,m-(15-v));
      };
      auto simSet=[&](const VGMMacroSet& x, int m, int relT, int n, std::vector<int>& out) {
        if (!x.has[m]) {
          out.assign(n,0);
          return;
        }
        vgmSimMacro(x.v[m],x.loop[m],x.rel[m],relT,n,out);
      };
      std::vector<std::vector<int>> notesOf(sets.size());
      for (size_t i=0; i<recs.size(); i++) notesOf[recs[i].set].push_back((int)i);
      std::vector<bool> gone(sets.size(),false);
      // groups that can stand in for each other: the same kind of instrument
      // (all 2A03 channels share one kind, so do all SN76489 channels: an
      // instrument heard on a pulse can serve the triangle or the noise too)
      // and the same FM sound
      auto kindOf=[](int type) -> int {
        if (type==0 || type==2 || type==3) return 0;
        if (type==5 || type==6) return 5;
        return type;
      };
      std::map<String,std::vector<int>> groups;
      for (size_t i=0; i<sets.size(); i++) {
        const VGMMacroSet& x=sets[i];
        String gk=fmt::sprintf("%d|%d|%s|",kindOf(x.type),x.fm,x.fmKey);
        groups[gk].push_back((int)i);
      }
      std::vector<int> outX, outY;
      for (auto& g: groups) {
        std::vector<int>& mem=g.second;
        if (mem.size()<2) continue;
        // the most heard ones are the best targets
        std::stable_sort(mem.begin(),mem.end(),[&](int x, int y) {
          return sets[x].weight>sets[y].weight;
        });
        for (int yi=(int)mem.size()-1; yi>=0; yi--) {
          int y=mem[yi];
          if (gone[y] || sets[y].ins>=0) continue;
          for (int xi=0; xi<(int)mem.size(); xi++) {
            int x=mem[xi];
            if (x==y || gone[x]) continue;
            const VGMMacroSet& X=sets[x];
            const VGMMacroSet& Y=sets[y];
            // a constant volume can't stand in for an envelope
            if (Y.has[0] && !X.has[0]) continue;
            std::vector<int> newVol(notesOf[y].size());
            std::vector<int> shiftArp(notesOf[y].size(),0), shiftFine(notesOf[y].size(),0);
            // (noise and drums: the pitch is a mode, it can't move)
            const bool canShift=(Y.type!=3 && Y.type!=6 && Y.type!=8);
            bool ok=true;
            for (size_t ni=0; ni<notesOf[y].size() && ok; ni++) {
              const VGMNoteRec& nr=recs[notesOf[y][ni]];
              int n=std::max(1,nr.obs);
              int relT=nr.relTick;
              for (int m=1; m<6 && ok; m++) {
                // the duty does nothing on the triangle
                if (m==1 && Y.type==2) continue;
                if (!X.has[m] && !Y.has[m]) continue;
                // a missing duty/noise/panning/envelope macro is not a 0
                if ((m==1 || m==4 || m==5) && X.has[m]!=Y.has[m]) {
                  ok=false;
                  break;
                }
                simSet(X,m,relT,n,outX);
                simSet(Y,m,relT,n,outY);
                if (outX==outY) continue;
                // an arpeggio or pitch moved by the same amount all along:
                // the note (or its E5xx) takes the difference
                if ((m==2 || m==3) && canShift) {
                  int d=outY[0]-outX[0];
                  bool same=true;
                  for (int k=1; k<n && same; k++) if (outY[k]-outX[k]!=d) same=false;
                  if (same) {
                    if (m==2) {
                      shiftArp[ni]=d;
                    } else {
                      shiftFine[ni]=d;
                    }
                    continue;
                  }
                }
                ok=false;
              }
              if (!ok) break;
              // the shifted note must still be a note (and a fine tune)
              if (shiftArp[ni]!=0) {
                if (nr.base+shiftArp[ni]<0 || nr.base+shiftArp[ni]>179) ok=false;
                for (auto& lg: nr.legato) if (lg[2]+shiftArp[ni]<0 || lg[2]+shiftArp[ni]>179) ok=false;
              }
              if (shiftFine[ni]!=0) {
                if (nr.fine+shiftFine[ni]<-127 || nr.fine+shiftFine[ni]>127) ok=false;
                for (auto& lg: nr.legato) if (lg[3]+shiftFine[ni]<-127 || lg[3]+shiftFine[ni]>127) ok=false;
              }
              if (!ok) break;
              // what the note heard as volume
              std::vector<int> heardVol(n);
              if (Y.has[0]) {
                simSet(Y,0,relT,n,outY);
                for (int k=0; k<n; k++) heardVol[k]=scaleVol(Y.type,nr.noteVol,outY[k]);
              } else {
                for (int k=0; k<n; k++) heardVol[k]=nr.noteVol;
              }
              // the volume column that makes X sound like that
              int found=-1;
              if (X.has[0]) {
                simSet(X,0,relT,n,outX);
                for (int V=(Y.type==15)?255:(Y.type==12 || Y.type==13 || Y.type==14)?127:15; V>=1 && found<0; V--) {
                  if (Y.type==2 && V!=15) break;
                  bool same=true;
                  for (int k=0; k<n && same; k++) {
                    if (scaleVol(Y.type,V,outX[k])!=heardVol[k]) same=false;
                  }
                  if (same) found=V;
                }
              } else {
                found=nr.noteVol;
              }
              if (found<0) ok=false;
              newVol[ni]=found;
            }
            if (!ok) continue;
            // y's notes move to x
            for (size_t ni=0; ni<notesOf[y].size(); ni++) {
              VGMNoteRec& nr=recs[notesOf[y][ni]];
              nr.set=x;
              nr.noteVol=newVol[ni];
              nr.base+=shiftArp[ni];
              nr.fine+=shiftFine[ni];
              for (auto& lg: nr.legato) {
                lg[2]+=shiftArp[ni];
                lg[3]+=shiftFine[ni];
              }
              if (sets[x].rel[0]<0) nr.relTick=-1;
              notesOf[x].push_back(notesOf[y][ni]);
              sets[x].heard.push_back(std::make_pair(nr.relTick,nr.obs));
            }
            sets[x].uses+=sets[y].uses;
            sets[x].weight+=sets[y].weight;
            notesOf[y].clear();
            gone[y]=true;
            absorbed++;
            break;
          }
        }
      }
      if (absorbed>0) {
        // drop the absorbed sets, renumbering the notes
        std::vector<int> remap(sets.size(),-1);
        std::vector<VGMMacroSet> kept;
        for (size_t i=0; i<sets.size(); i++) {
          if (gone[i]) continue;
          remap[i]=(int)kept.size();
          kept.push_back(sets[i]);
        }
        for (VGMNoteRec& nr: recs) nr.set=remap[nr.set];
        sets.swap(kept);
        logI("VGM import: %d instruments were copies of others (cut short, quieter, moved in pitch or on another channel)",absorbed);
      }
    }

    // instrument budget. shapes that already have an instrument (the same
    // shape in an earlier subsong) cost nothing.
    int room=256-(int)ds.ins.size();
    int budget=room;
    if (ctx.shareLimit>0 && ctx.shareLimit<budget) budget=ctx.shareLimit;
    int newSets=0;
    for (VGMMacroSet& ms: sets) if (ms.ins<0) newSets++;
    std::vector<VGMMacroSet*> target(sets.size());
    for (size_t i=0; i<sets.size(); i++) target[i]=&sets[i];
    int mergedSets=0, mergedNotes=0;
    // value of macro m at tick k (follows the loop/hold like Furnace does)
    auto macroAt=[](const VGMMacroSet& s, int m, int k) -> int {
      const std::vector<int>& v=s.v[m];
      if (!s.has[m] || v.empty()) return (m==0)?-1:0;
      if (k<(int)v.size()) return v[k];
      if (s.loop[m]>=0) {
        int ll=(int)v.size()-s.loop[m];
        return v[s.loop[m]+((k-s.loop[m])%ll)];
      }
      return v.back();
    };
    static const int weight[6]={6,10,24,1,4,4};
    auto dist=[&](const VGMMacroSet& x, const VGMMacroSet& y) -> long long {
      long long d=0;
      // (the YM2612's volume goes to 127: one step of it is a small one)
      const int wVol=(x.type==12 || x.type==13 || x.type==14 || x.type==15)?1:weight[0];
      for (int m=0; m<6; m++) {
        for (int k=0; k<96; k++) {
          int ax=macroAt(x,m,k), ay=macroAt(y,m,k);
          // a constant volume comes from the volume column: treat as equal,
          // but an envelope against a constant volume is a big difference
          if (m==0 && ax<0 && ay<0) continue;
          if (m==0 && (ax<0 || ay<0)) {
            d+=(long long)weight[0]*4;
            continue;
          }
          d+=(long long)((m==0)?wVol:weight[m])*abs(ax-ay);
        }
      }
      return d;
    };

    // YM2612: how different two shapes sound. the patch, field by field
    // (with its FM macros, over the first ticks), weighted by how much each
    // field is heard (a different multiplier, SSG-EG or algorithm is another
    // sound; a step of TL, attack or release is a small change), plus the
    // macros (per tick). -1: they can't stand in for each other at all
    struct OPNShape {
      int base[48];
      int mac[48]; // index into fmMac, -1: none
    };
    std::vector<OPNShape> opnShape;
    if (log.hasOPN) {
      opnShape.resize(sets.size());
      for (size_t i=0; i<sets.size(); i++) {
        if ((sets[i].type!=12 && sets[i].type!=13) || sets[i].fm<0) continue;
        OPNShape& sh=opnShape[i];
        const unsigned char* raw=fmSounds[sets[i].fm].opnRaw;
        for (int f=0; f<48; f++) {
          sh.base[f]=opnField(raw,f);
          sh.mac[f]=-1;
        }
        for (size_t fi=0; fi<sets[i].fmMac.size(); fi++) {
          int f=sets[i].fmMac[fi].first;
          if (f>=0 && f<48) sh.mac[f]=(int)fi;
        }
      }
    }
    auto opnDistance=[&](int xi, int yi, double limit) -> double {
      const VGMMacroSet& x=sets[xi];
      const VGMMacroSet& y=sets[yi];
      // (FM 3's operators only stand in for each other, not for channels)
      if ((x.type!=12 && x.type!=13) || x.type!=y.type || x.fm<0 || y.fm<0) return -1;
      // (the same structure: an envelope, a pan macro, key ons in the note)
      if (x.has[0]!=y.has[0] || x.has[4]!=y.has[4] || x.has[5]!=y.has[5]) return -1;
      const OPNShape& a=opnShape[xi];
      const OPNShape& b=opnShape[yi];
      // weights per operator field: detune, multiplier, TL, rate scaling,
      // attack, AM, decay, sustain rate, sustain level, release, SSG-EG
      static const double wOp[11]={3,100,2,1,2,20,1.5,1.5,2,1,100};
      const int algX=a.base[44], algY=b.base[44];
      double d=0;
      if (algX!=algY) d+=400;
      d+=8.0*abs(a.base[45]-b.base[45]);
      d+=10.0*abs(a.base[46]-b.base[46]);
      d+=10.0*abs(a.base[47]-b.base[47]);
      if (d>limit) return d;
      for (int f=0; f<44; f++) {
        int o=f/11, fk=f%11;
        double w=wOp[fk];
        // (TL: the carriers' is relative, a modulator's is the timbre)
        if (fk==2 && opnCarrier[algX][o] && opnCarrier[algY][o]) w=1;
        // (a different multiplier or SSG-EG is another sound, by any step)
        bool flat=(fk==1 || fk==10);
        if (a.mac[f]<0 && b.mac[f]<0) {
          int diff=abs(a.base[f]-b.base[f]);
          if (diff) d+=flat?w:w*diff;
        } else {
          double sum=0;
          for (int k=0; k<32; k++) {
            int vx=(a.mac[f]<0)?a.base[f]:vgmMacroValueAt(x.fmMac[a.mac[f]].second,x.fmMacLoop[a.mac[f]],k);
            int vy=(b.mac[f]<0)?b.base[f]:vgmMacroValueAt(y.fmMac[b.mac[f]].second,y.fmMacLoop[b.mac[f]],k);
            int diff=abs(vx-vy);
            if (diff) sum+=flat?w:w*diff;
          }
          d+=sum/32.0;
        }
        if (d>limit) return d;
      }
      d+=(double)dist(x,y)/96.0;
      return d;
    };
    // (at most this different: the same sound, for the ear)
    const double opnSimilar=3.0;
    // (and this: close enough not to need an instrument of its own when
    // there is no room for all of them)
    const double opnNear=12.0;

    // YM2612: shapes that are almost the same are one instrument. the most
    // heard ones are kept, the others go to the closest of those
    std::vector<bool> similarGone(sets.size(),false);
    int similarSets=0, similarNotes=0;
    if (log.hasOPN) {
      std::vector<int> ord;
      std::vector<int> reps;
      for (size_t i=0; i<sets.size(); i++) {
        if (sets[i].type!=12 && sets[i].type!=13) continue;
        if (sets[i].ins>=0) {
          reps.push_back((int)i);
        } else {
          ord.push_back((int)i);
        }
      }
      std::stable_sort(ord.begin(),ord.end(),[&](int x, int y) {
        return sets[x].weight>sets[y].weight;
      });
      for (int i: ord) {
        int best=-1;
        double bd=opnSimilar;
        for (int j: reps) {
          double d=opnDistance(i,j,bd);
          if (d>=0 && d<=bd) {
            bd=d;
            best=j;
          }
        }
        if (best>=0) {
          target[i]=&sets[best];
          similarGone[i]=true;
          similarSets++;
          similarNotes+=sets[i].uses;
        } else {
          reps.push_back(i);
        }
      }
      if (similarSets>0) {
        newSets=0;
        for (size_t i=0; i<sets.size(); i++) if (sets[i].ins<0 && !similarGone[i]) newSets++;
        logI("VGM import: %d YM2612 instruments were almost the same as others and were joined (%d notes)",similarSets,similarNotes);
      }
    }

    if (newSets>budget) {
      std::vector<int> order;
      for (size_t i=0; i<sets.size(); i++) if (sets[i].ins<0 && !similarGone[i]) order.push_back((int)i);
      // the shapes heard the longest keep their own instrument
      std::stable_sort(order.begin(),order.end(),[&](int x, int y) {
        return sets[x].weight>sets[y].weight;
      });
      std::vector<bool> kept(sets.size(),false);
      int keptCount=0;
      // a channel type with no instrument at all keeps its most used shape
      for (int type: {0,2,3,4,5,6,7,8,12,13,14}) {
        bool any=false;
        for (VGMMacroSet& ms: sets) if (ms.type==type && ms.ins>=0) any=true;
        for (VGMMacroSet& ms: ctx.prevSets) if (ms.type==type) any=true;
        if (any) continue;
        for (int i: order) {
          if (sets[i].type==type) {
            if (!kept[i] && keptCount<room) {
              kept[i]=true;
              keptCount++;
            }
            break;
          }
        }
      }
      // (YM2612: a shape close to one that is already kept leaves its room
      // to a sound that is different)
      std::vector<int> keptOPN;
      for (int i: order) if (kept[i] && (sets[i].type==12 || sets[i].type==13)) keptOPN.push_back(i);
      for (size_t i=0; i<sets.size(); i++) if (sets[i].ins>=0 && (sets[i].type==12 || sets[i].type==13)) keptOPN.push_back((int)i);
      std::vector<bool> nearKept(sets.size(),false);
      for (int i: order) {
        if (keptCount>=budget) break;
        if (kept[i]) continue;
        if (sets[i].type==12 || sets[i].type==13) {
          bool near=false;
          for (int j: keptOPN) {
            double d=opnDistance(i,j,opnNear);
            if (d>=0 && d<=opnNear) {
              near=true;
              break;
            }
          }
          if (near) {
            nearKept[i]=true;
            continue;
          }
          keptOPN.push_back(i);
        }
        kept[i]=true;
        keptCount++;
      }
      // (if there is room left after that, the near ones get it too)
      for (int i: order) {
        if (keptCount>=budget) break;
        if (kept[i] || !nearKept[i]) continue;
        kept[i]=true;
        keptCount++;
      }
      for (int i: order) {
        if (kept[i]) continue;
        long long best=-1;
        VGMMacroSet* bestSet=NULL;
        // YM2612: the closest sound that has an instrument, whatever patch
        if (sets[i].type==12 || sets[i].type==13) {
          double bd=-1;
          int bj=-1;
          for (size_t j=0; j<sets.size(); j++) {
            if (!(kept[j] || sets[j].ins>=0) || sets[j].type!=sets[i].type) continue;
            double d=opnDistance(i,(int)j,(bd<0)?1e30:bd);
            if (d<0) continue;
            if (bd<0 || d<bd) {
              bd=d;
              bj=(int)j;
            }
          }
          if (bj>=0) {
            target[i]=&sets[bj];
            mergedSets++;
            mergedNotes+=sets[i].uses;
            continue;
          }
        }
        auto consider=[&](VGMMacroSet* cand) {
          if (cand->type!=sets[i].type || cand->fm!=sets[i].fm || cand->fmKey!=sets[i].fmKey) return;
          long long d=dist(sets[i],*cand);
          if (best<0 || d<best) {
            best=d;
            bestSet=cand;
          }
        };
        for (size_t j=0; j<sets.size(); j++) {
          if (kept[j] || sets[j].ins>=0) consider(&sets[j]);
        }
        for (VGMMacroSet& ps: ctx.prevSets) consider(&ps);
        if (bestSet==NULL) {
          // nothing of this type: keep it if there is any room at all
          if ((int)ds.ins.size()+keptCount<256) {
            kept[i]=true;
            keptCount++;
            continue;
          }
          // FM: nothing with this exact sound (patch registers, FM macros)
          // left, so the closest instrument of the same kind
          if ((sets[i].type>=7 && sets[i].type<=9) || sets[i].type==12 || sets[i].type==13 || sets[i].type==14 || sets[i].type==15) {
            auto considerAny=[&](VGMMacroSet* cand) {
              if (cand->type!=sets[i].type) return;
              long long d=dist(sets[i],*cand);
              // a different patch is a different sound
              if (cand->fm!=sets[i].fm) d+=100000;
              if (cand->fmKey!=sets[i].fmKey) d+=1000;
              if (best<0 || d<best) {
                best=d;
                bestSet=cand;
              }
            };
            for (size_t j=0; j<sets.size(); j++) {
              if (kept[j] || sets[j].ins>=0) considerAny(&sets[j]);
            }
            for (VGMMacroSet& ps: ctx.prevSets) considerAny(&ps);
          }
          for (VGMMacroSet& ps: ctx.prevSets) {
            if (bestSet!=NULL) break;
            if (ps.type==sets[i].type) {
              bestSet=&ps;
              break;
            }
          }
        }
        if (bestSet!=NULL) {
          target[i]=bestSet;
          mergedSets++;
          mergedNotes+=sets[i].uses;
        }
      }
    }

    // (SNES: the song's sample of each of the log's samples, made when an
    // instrument first needs it)
    std::vector<int> snesSmpIdx(log.snesSamples.size(),-1);
    // make the instruments, in order of first use
    std::map<String,int>& nameCount=ctx.nameCount;
    static const char* dutyNames[4]={"12.5%","25%","50%","75%"};
    std::vector<bool> created(sets.size(),false);
    for (VGMNoteRec& nr: recs) {
      VGMMacroSet& ms=*target[nr.set];
      if (ms.ins>=0) continue;
      if ((int)ds.ins.size()>=256) {
        // should not happen (the budget keeps room), but never go past 256
        ms.ins=255;
        continue;
      }
      created[nr.set]=true;
      DivInstrument* ins=new DivInstrument;
      ins->type=(ms.type>=16 && ms.type<=18)?DIV_INS_GB:(ms.type==15)?DIV_INS_RF5C68:(ms.type==14)?DIV_INS_SNES:(ms.type==12 || ms.type==13)?DIV_INS_FM:(ms.type==10)?DIV_INS_VRC6:(ms.type==11)?DIV_INS_VRC6_SAW:(ms.type==9)?DIV_INS_C64:((ms.type==4)?DIV_INS_AY:((ms.type>=7)?DIV_INS_OPLL:((ms.type>=5)?DIV_INS_STD:DIV_INS_NES)));
      String name;
      if (ms.type==9) {
        // the voice's settings at the start of the note (S<wave>,<bits>,
        // <attack>,<decay>,<sustain>,<release>,<filter>;)
        int b[7]={0,1,0,0,0,0,0};
        size_t sp=ms.fmKey.find('S');
        if (sp!=String::npos) sscanf(ms.fmKey.c_str()+sp+1,"%d,%d,%d,%d,%d,%d,%d",&b[0],&b[1],&b[2],&b[3],&b[4],&b[5],&b[6]);
        DivInstrumentC64& c64=ins->c64;
        c64.triOn=b[0]&1;
        c64.sawOn=b[0]&2;
        c64.pulseOn=b[0]&4;
        c64.noiseOn=b[0]&8;
        c64.oscSync=b[1]&2;
        c64.ringMod=b[1]&4;
        c64.a=b[2];
        c64.d=b[3];
        c64.s=b[4];
        c64.r=b[5];
        c64.toFilter=b[6];
        c64.initFilter=false;
        c64.dutyIsAbs=true;
        c64.filterIsAbs=true;
        c64.resetDuty=true;
        // (a free-running pulse width is left alone by the notes)
        bool freePW=(ms.fmKey.find("P;")!=String::npos);
        if (freePW) c64.resetDuty=false;
        // (Furnace's hard restart: with the test bit only if the player's has it)
        c64.noTest=!(ctx.sidHR>0 && (ctx.sidHR&(1<<16)));
        c64.duty=ms.v[1].empty()?2048:ms.v[1][0];
        static const char* waveNames[16]={
          "Silent","Triangle","Saw","Tri+Saw","Pulse","Tri+Pulse","Saw+Pulse","Tri+Saw+Pulse",
          "Noise","Noise+Tri","Noise+Saw","Noise+Tri+Saw","Noise+Pulse","Noise+Tri+Pulse","Noise+Saw+Pulse","Noise+all"
        };
        name=waveNames[b[0]&15];
        if (c64.ringMod) name+=" ring";
        if (c64.oscSync) name+=" sync";
        if (c64.toFilter) name+=" filter";
        bool anyFilterMac=false;
        for (size_t fi=0; fi<ms.fmMac.size(); fi++) {
          int f=ms.fmMac[fi].first;
          DivInstrumentMacro* mac=NULL;
          switch (f) {
            case 0: mac=&ins->std.waveMacro; break;
            case 1: mac=&ins->std.ex4Macro; break;
            case 2: mac=&ins->std.ex5Macro; break;
            case 3: mac=&ins->std.ex6Macro; break;
            case 4: mac=&ins->std.ex7Macro; break;
            case 5: mac=&ins->std.ex8Macro; break;
            case 6: mac=&ins->std.ex3Macro; break;
            case 10: mac=&ins->std.algMacro; anyFilterMac=true; break;
            case 11: mac=&ins->std.ex2Macro; anyFilterMac=true; break;
            case 12: mac=&ins->std.ex1Macro; anyFilterMac=true; break;
            case 13: mac=&ins->std.volMacro; break;
          }
          if (mac==NULL) continue;
          const std::vector<int>& v=ms.fmMac[fi].second;
          mac->len=v.size();
          for (size_t k=0; k<v.size(); k++) mac->val[k]=v[k];
          if (ms.fmMacLoop[fi]>=0) mac->loop=ms.fmMacLoop[fi];
        }
        if (anyFilterMac) name+=" sweep";
      } else if (ms.type==15) {
        // RF5C68: the sample (the bytes the channel read, sign and
        // magnitude) at the chip's rate
        const VGMFMSound& fs=fmSounds[ms.fm];
        int si=fs.snesSample;
        name="PCM";
        if (si>=0 && si<(int)log.snesSamples.size()) {
          const VGMSNESSample& sm=log.snesSamples[si];
          if (snesSmpIdx[si]<0) {
            DivSample* smp=new DivSample;
            smp->name=fmt::sprintf("PCM $%.4X",sm.addr);
            smp->depth=DIV_SAMPLE_DEPTH_8BIT;
            smp->centerRate=rfRate;
            int len=(int)sm.brr.size();
            smp->init(len);
            for (int i=0; i<len; i++) {
              unsigned char x=sm.brr[i];
              smp->data8[i]=(x&0x80)?(signed char)(x&0x7f):(signed char)(-(int)(x&0x7f));
            }
            if (sm.loopBlock>=0 && sm.loopBlock<len) {
              smp->loop=true;
              smp->loopStart=sm.loopBlock;
              smp->loopEnd=len;
              smp->loopMode=DIV_SAMPLE_LOOP_FORWARD;
            }
            snesSmpIdx[si]=(int)ds.sample.size();
            ds.sample.push_back(smp);
          }
          ins->amiga.initSample=snesSmpIdx[si];
          name=fmt::sprintf("PCM $%.4X",sm.addr);
        }
      } else if (ms.type==14) {
        // SNES: the sample (its BRR blocks as they were) and the envelope
        const VGMFMSound& fs=fmSounds[ms.fm];
        int si=fs.snesSample;
        if (si>=0 && si<(int)log.snesSamples.size()) {
          if (snesSmpIdx[si]<0) {
            const VGMSNESSample& sm=log.snesSamples[si];
            DivSample* smp=new DivSample;
            smp->name=fmt::sprintf("BRR $%.4X",sm.addr);
            smp->depth=DIV_SAMPLE_DEPTH_BRR;
            smp->centerRate=32000;
            int blocks=(int)(sm.brr.size()/9);
            smp->init(16*blocks);
            memcpy(smp->dataBRR,sm.brr.data(),blocks*9);
            if (sm.loopBlock>=0 && sm.loopBlock<blocks) {
              smp->loop=true;
              smp->loopStart=16*sm.loopBlock;
              smp->loopEnd=16*blocks;
              smp->loopMode=DIV_SAMPLE_LOOP_FORWARD;
            }
            snesSmpIdx[si]=(int)ds.sample.size();
            ds.sample.push_back(smp);
          }
          ins->amiga.initSample=snesSmpIdx[si];
        }
        DivInstrumentSNES& sn=ins->snes;
        const unsigned char a1=fs.snesEnv[0], a2=fs.snesEnv[1], g=fs.snesEnv[2];
        const bool followed=(ms.fmKey.find("E;")!=String::npos);
        if (followed) {
          // (the level of the original's envelope, tick by tick)
          sn.useEnv=false;
          sn.gainMode=DivInstrumentSNES::GAIN_MODE_DIRECT;
          sn.gain=ms.fmMac.empty()?127:(ms.fmMac[0].second[0]&127);
          // (the macro's first value is the GAIN to start with)
          if (!ms.fmMac.empty() && (ms.fmMac[0].second[0]&0x80)) {
            sn.gainMode=(DivInstrumentSNES::GainMode)((ms.fmMac[0].second[0]>>5)&7);
            sn.gain=ms.fmMac[0].second[0]&31;
          }
          name="Envelope";
        } else if (a1&0x80) {
          sn.useEnv=true;
          sn.a=a1&15;
          sn.d=(a1>>4)&7;
          sn.s=(a2>>5)&7;
          sn.r=a2&31;
          sn.sus=0;
          name="ADSR";
          size_t rp=ms.fmKey.find("R");
          if (rp!=String::npos && (rp==0 || ms.fmKey[rp-1]==';')) {
            // (the release: a decreasing GAIN at the note off)
            int rg=atoi(ms.fmKey.c_str()+rp+1);
            sn.sus=(rg&0x20)?2:1;
            sn.d2=a2&31;
            sn.r=rg&31;
            name="ADSR release";
          }
        } else {
          sn.useEnv=false;
          if (g&0x80) {
            sn.gainMode=(DivInstrumentSNES::GainMode)((g>>5)&7);
            sn.gain=g&31;
          } else {
            sn.gainMode=DivInstrumentSNES::GAIN_MODE_DIRECT;
            sn.gain=g&127;
          }
          name="Gain";
        }
        if (si>=0 && si<(int)log.snesSamples.size()) name=fmt::sprintf("BRR $%.4X %s",log.snesSamples[si].addr,name);
        for (size_t fi=0; fi<ms.fmMac.size(); fi++) {
          const std::vector<int>& v=ms.fmMac[fi].second;
          DivInstrumentMacro* mac=&ins->std.ex2Macro;
          mac->len=v.size();
          for (size_t k=0; k<v.size(); k++) mac->val[k]=v[k];
          if (ms.fmMacLoop[fi]>=0) mac->loop=ms.fmMacLoop[fi];
        }
        if (!ms.fmMac.empty() && !followed) name+=" gain macro";
      } else if (ms.type==12 || ms.type==13) {
        // YM2612: the channel's patch, register by register
        const VGMFMSound& fs=fmSounds[ms.fm];
        const unsigned char* raw=fs.opnRaw;
        DivInstrumentFM& fm=ins->fm;
        fm.ops=4;
        if (ms.fmKey.size()>1 && ms.fmKey[0]=='B') fm.block=atoi(ms.fmKey.c_str()+1);
        fm.alg=opnField(raw,44);
        fm.fb=opnField(raw,45);
        fm.fms=opnField(raw,46);
        fm.ams=opnField(raw,47);
        for (int o=0; o<4; o++) {
          DivInstrumentFM::Operator& op=fm.op[o];
          op.enable=true;
          op.dt=opnField(raw,o*11+0);
          op.mult=opnField(raw,o*11+1);
          op.tl=opnField(raw,o*11+2);
          op.rs=opnField(raw,o*11+3);
          op.ar=opnField(raw,o*11+4);
          op.am=opnField(raw,o*11+5);
          op.dr=opnField(raw,o*11+6);
          op.d2r=opnField(raw,o*11+7);
          op.sl=opnField(raw,o*11+8);
          op.rr=opnField(raw,o*11+9);
          op.ssgEnv=opnField(raw,o*11+10);
        }
        name=(ms.type==13)?fmt::sprintf("FM3 op alg %d",fm.alg):fmt::sprintf("FM alg %d",fm.alg);
        for (size_t fi=0; fi<ms.fmMac.size(); fi++) {
          int f=ms.fmMac[fi].first;
          DivInstrumentMacro* mac=NULL;
          if (f<44) {
            DivInstrumentSTD::OpMacro& om=ins->std.opMacros[f/11];
            DivInstrumentMacro* opm[11]={
              &om.dtMacro,&om.multMacro,&om.tlMacro,&om.rsMacro,&om.arMacro,&om.amMacro,
              &om.drMacro,&om.d2rMacro,&om.slMacro,&om.rrMacro,&om.ssgMacro
            };
            mac=opm[f%11];
          } else {
            switch (f) {
              case 44: mac=&ins->std.algMacro; break;
              case 45: mac=&ins->std.fbMacro; break;
              case 46: mac=&ins->std.fmsMacro; break;
              case 47: mac=&ins->std.amsMacro; break;
            }
          }
          if (mac==NULL) continue;
          const std::vector<int>& v=ms.fmMac[fi].second;
          mac->len=v.size();
          for (size_t k=0; k<v.size(); k++) mac->val[k]=v[k];
          if (ms.fmMacLoop[fi]>=0) mac->loop=ms.fmMacLoop[fi];
        }
        if (!ms.fmMac.empty()) name+=" FM macro";
      } else if (ms.type>=7 && ms.type<=8) {
        const VGMFMSound& fs=fmSounds[ms.fm];
        static const char* opllNames[16]={
          "FM custom","Violin","Guitar","Piano","Flute","Clarinet","Oboe","Trumpet",
          "Organ","Horn","Synth","Harpsichord","Vibraphone","Synth Bass","Wood Bass","Electric Guitar"
        };
        DivInstrumentFM& fm=ins->fm;
        fm.ops=2;
        // a fixed block, if the notes need it (the key starts with B<block>;)
        if (ms.fmKey.size()>1 && ms.fmKey[0]=='B') fm.block=atoi(ms.fmKey.c_str()+1);
        const int patch=fs.drum?16:ms.v[1][0];
        fm.opllPreset=patch;
        // the channel's sustain bit
        fm.alg=fs.sus?1:0;
        fm.fb=0;
        // the carrier "level" lowers the volume in Furnace: none here
        fm.op[1].tl=0;
        if (fs.drum) {
          name="Drums";
          fm.fixedDrums=false;
        } else if (log.opllVRC7) {
          name=(patch==0)?String("FM custom"):fmt::sprintf("VRC7 patch %d",patch);
        } else {
          name=opllNames[patch&15];
        }
        if (fs.inst==0) {
          // the custom patch, register by register
          const unsigned char* r=fs.cust;
          for (int o=0; o<2; o++) {
            DivInstrumentFM::Operator& op=fm.op[o];
            op.am=(r[o]>>7)&1;
            op.vib=(r[o]>>6)&1;
            op.ssgEnv=(r[o]&0x20)?8:0;
            op.egt=(r[o]>>5)&1;
            op.ksr=(r[o]>>4)&1;
            op.mult=r[o]&15;
            op.ar=r[4+o]>>4;
            op.dr=r[4+o]&15;
            op.sl=r[6+o]>>4;
            op.rr=r[6+o]&15;
          }
          fm.op[0].ksl=r[2]>>6;
          fm.op[0].tl=r[2]&63;
          fm.op[1].ksl=r[3]>>6;
          fm.fms=(r[3]>>4)&1;
          fm.ams=(r[3]>>3)&1;
          fm.fb=r[3]&7;
        }
        if (fs.sus) name+=" sus";
        for (size_t fi=0; fi<ms.fmMac.size(); fi++) {
          int f=ms.fmMac[fi].first;
          DivInstrumentMacro* mac=NULL;
          if (f<10) {
            DivInstrumentSTD::OpMacro& om=ins->std.opMacros[f/5];
            DivInstrumentMacro* opm[5]={&om.amMacro,&om.vibMacro,&om.egtMacro,&om.ksrMacro,&om.multMacro};
            mac=opm[f%5];
          } else {
            switch (f) {
              case 10: mac=&ins->std.opMacros[0].tlMacro; break;
              case 11: mac=&ins->std.opMacros[0].kslMacro; break;
              case 12: mac=&ins->std.opMacros[1].kslMacro; break;
              case 13: mac=&ins->std.fmsMacro; break;
              case 14: mac=&ins->std.amsMacro; break;
              case 15: mac=&ins->std.fbMacro; break;
              default: {
                // AR/DR (registers 4-5), SL/RR (6-7)
                int r=f-16;
                DivInstrumentSTD::OpMacro& om=ins->std.opMacros[(r>>1)&1];
                if (r<4) {
                  mac=(r&1)?&om.drMacro:&om.arMacro;
                } else {
                  mac=(r&1)?&om.rrMacro:&om.slMacro;
                }
                break;
              }
            }
          }
          const std::vector<int>& v=ms.fmMac[fi].second;
          mac->len=v.size();
          for (size_t k=0; k<v.size(); k++) mac->val[k]=v[k];
          if (ms.fmMacLoop[fi]>=0) mac->loop=ms.fmMacLoop[fi];
        }
        if (!ms.fmMac.empty()) name+=" FM macro";
      } else if (ms.type>=16 && ms.type<=18) {
        // Game Boy: the volume goes through the macro (software envelope:
        // the import follows the hardware envelope tick by tick)
        ins->gb.softEnv=true;
        ins->gb.envVol=15;
        ins->gb.envDir=0;
        ins->gb.envLen=0;
        ins->gb.soundLen=64;
        if (ms.type==17) {
          name="Wave";
        } else if (ms.type==18) {
          name=(ms.v[1].size()==1)?(ms.v[1][0]?"Noise short":"Noise"):"Noise mix";
        } else {
          name=(ms.v[1].size()==1)?fmt::sprintf("Pulse %s",dutyNames[ms.v[1][0]&3]):String("Pulse duty var");
        }
      } else if (ms.type==10) {
        name=(ms.v[1].size()==1)?fmt::sprintf("VRC6 pulse %d/16",(ms.v[1][0]&7)+1):String("VRC6 pulse duty var");
      } else if (ms.type==11) {
        name="VRC6 saw";
      } else if (ms.type==5) {
        name="PSG square";
      } else if (ms.type==6) {
        static const char* nModes[4]={"PSG noise periodic","PSG noise white","PSG noise periodic sq3","PSG noise white sq3"};
        name=(ms.v[1].size()==1)?String(nModes[ms.v[1][0]&3]):String("PSG noise mix");
      } else if (ms.type==4) {
        static const char* modeNames[8]={"5B off","5B square","5B noise","5B square+noise","5B env","5B square+env","5B noise+env","5B square+noise+env"};
        name=(ms.v[1].size()==1)?String(modeNames[ms.v[1][0]&7]):String("5B mode var");
      } else if (ms.type==2) {
        name="Triangle";
      } else if (ms.type==3) {
        name=(ms.v[1].size()==1)?(ms.v[1][0]?"Noise metal":"Noise"):"Noise mix";
      } else {
        name=(ms.v[1].size()==1)?fmt::sprintf("Pulse %s",dutyNames[ms.v[1][0]&3]):String("Pulse duty var");
      }
      if (ms.has[0]) name+=" env";
      if (ms.has[2]) name+=" arp";
      if (ms.has[3]) name+=" pitch";
      int num=++nameCount[name];
      ins->name=fmt::sprintf("%s %d",name,num);
      // on the 5B the mode goes to the wave macro, the noise frequency to the
      // duty macro and the envelope shape to the ex2 macro
      // on the SN76489 the duty macro is the noise mode and slot 4 holds
      // the Game Gear panning. on FM the patch number is the wave macro
      DivInstrumentMacro* macros[6]={
        &ins->std.volMacro,
        (ms.type==4 || ms.type==7 || ms.type==17)?&ins->std.waveMacro:&ins->std.dutyMacro,
        &ins->std.arpMacro,
        &ins->std.pitchMacro,
        ((ms.type>=5 && ms.type<9) || (ms.type>=12 && ms.type<=18))?&ins->std.panLMacro:&ins->std.dutyMacro,
        (ms.type==14 || ms.type==15)?&ins->std.panRMacro:((ms.type>=7 && ms.type<9) || ms.type==12 || ms.type==13)?&ins->std.phaseResetMacro:&ins->std.ex2Macro
      };
      for (int m=0; m<6; m++) {
        if (!ms.has[m]) continue;
        // FM: a patch that never changes is the instrument's own patch
        if (m==1 && ms.type==7 && ms.v[1].size()<=1) continue;
        // SID: a pulse width that never changes is the instrument's own
        if (m==1 && ms.type==9 && (ms.v[1].size()<=1 || ms.fmKey.find("P;")!=String::npos)) continue;
        if (ms.type==9 && (m==0 || m>=4)) continue;
        DivInstrumentMacro* mac=macros[m];
        if (ms.loop[m]>=0) mac->loop=ms.loop[m];
        if (ms.rel[m]>=0) {
          mac->rel=ms.rel[m];
          // "active release": the release jumps straight to the release
          // point instead of waiting for the sustain to come around
          mac->open|=8;
        }
        mac->len=ms.v[m].size();
        for (size_t k=0; k<ms.v[m].size(); k++) mac->val[k]=ms.v[m][k];
      }
      ms.ins=(int)ds.ins.size();
      ds.ins.push_back(ins);
      ctx.insByKey[ms.key]=ms.ins;
    }

    // write the notes
    {
      // what the volume column and E5xx are at, per song channel
      int colVol[DIV_MAX_CHANS], colFine[DIV_MAX_CHANS], colArp[DIV_MAX_CHANS], colVib[DIV_MAX_CHANS], colPan[DIV_MAX_CHANS];
      // (the vibrato's shape, E3xx: 0 sine, 3 triangle. it is in vibVal's
      // bits 8 and up)
      int colShape[DIV_MAX_CHANS];
      for (int c=0; c<DIV_MAX_CHANS; c++) colShape[c]=0;
      for (int c=0; c<DIV_MAX_CHANS; c++) {
        colVol[c]=-1;
        colFine[c]=0;
        colArp[c]=0;
        colVib[c]=0;
        // (Furnace's YM2612 starts with both sides on)
        colPan[c]=3;
      }
      // every change of those (and of legato), per song channel: (row, value),
      // to set them again on the loop row
      std::map<int,std::vector<std::pair<int,int>>> stateW[6];
      auto noteW=[&](int prop, int pc, int row, int val) {
        stateW[prop][pc].push_back(std::make_pair(row,val));
      };
      // in the order they play (channels that share a song channel, like FM 3
      // and its OP1, take turns on it: what the column holds goes in time)
      std::vector<int> recOrder(recs.size());
      for (size_t i=0; i<recs.size(); i++) recOrder[i]=(int)i;
      std::stable_sort(recOrder.begin(),recOrder.end(),[&](int x, int y) {
        return recs[x].rowA<recs[y].rowA;
      });
      for (int ri: recOrder) {
        VGMNoteRec& nr=recs[ri];
        int c=nr.c;
        VGMMacroSet& ms=*target[nr.set];
        // notes of their own set carry their volume column (15, or lower for a
        // scaled envelope); a merged note borrows the target's shape
        int noteVol=nr.noteVol;
        // (YM2612: a shape joined to a close one keeps its volume column
        // when both have an envelope, or both don't)
        const bool keepVol=isOPNChan(c) && ms.has[0]==sets[nr.set].has[0];
        if (target[nr.set]!=&sets[nr.set] && !keepVol) noteVol=ms.has[0]?(isOPNChan(c)?127:((c==31)?63:15)):nr.noteVol;
        short* cell=cellAt(c,nr.rowA);
        cell[DIV_PAT_NOTE]=nr.base;
        cell[DIV_PAT_INS]=ms.ins;
        const int pc=patIdx[c];
        // (the SID's volume column is the volume of the whole chip: never)
        if (noteVol!=colVol[pc] && !(c>=26 && c<29)) {
          cell[DIV_PAT_VOL]=noteVol;
          colVol[pc]=noteVol;
          noteW(0,pc,nr.rowA,noteVol);
        }
        // (YM2612: the volume after the macro's end, in the volume column.
        // Furnace adds the two: the volume is the column plus the macro's
        // last value, minus 127)
        if (!nr.volTail.empty() && (!ms.has[0] || ms.loop[0]<0)) {
          int held=127;
          if (ms.has[0] && !ms.v[0].empty()) held=ms.v[0][std::min((size_t)254,ms.v[0].size()-1)];
          for (const std::pair<int,int>& vt: nr.volTail) {
            short* tc=cellAt(c,vt.first);
            if (tc[DIV_PAT_NOTE]!=-1 || tc[DIV_PAT_VOL]!=-1) continue;
            int v=ms.has[0]?(vt.second-held+127):vt.second;
            v=std::max(0,std::min(127,v));
            if (v==colVol[pc]) continue;
            tc[DIV_PAT_VOL]=v;
            colVol[pc]=v;
            noteW(0,pc,vt.first,v);
          }
        }
        if (nr.fine!=colFine[pc]) {
          vgmPutFx(cell,0xe5,0x80+nr.fine,maxFxCol[c]);
          colFine[pc]=nr.fine;
          noteW(1,pc,nr.rowA,nr.fine);
        }
        // the envelope period is shared and notes are written channel by
        // channel, so set it on every note that uses the envelope
        if (nr.envPeriod>=0) {
          vgmPutFx(cell,0x23,nr.envPeriod&0xff,maxFxCol[c]);
          vgmPutFx(cell,0x24,nr.envPeriod>>8,maxFxCol[c]);
        }
        if (nr.delayA>0) vgmPutFx(cell,0xed,nr.delayA,maxFxCol[c]);
        // YM2612 panning: 08xx where it changes (after a pan macro it is
        // not known, so it is set again)
        if (nr.pan>=0 && nr.pan!=colPan[pc] && c>=44 && c<52 && log.hasRF) {
          // (RF5C68: 08xy, x the left side: the register's low nibble)
          vgmPutFx(cell,0x08,((nr.pan&15)<<4)|((nr.pan>>4)&15),maxFxCol[c]);
          colPan[pc]=nr.pan;
          noteW(5,pc,nr.rowA,nr.pan);
        } else if (nr.pan>=0 && nr.pan!=colPan[pc] && c>=44 && c<52) {
          // (SNES: 81xx left, 82xx right; Furnace takes half of each)
          int pl=nr.pan>>8, pr=nr.pan&0xff;
          vgmPutFx(cell,0x81,(pl>=127)?255:(pl*2),maxFxCol[c]);
          vgmPutFx(cell,0x82,(pr>=127)?255:(pr*2),maxFxCol[c]);
          colPan[pc]=nr.pan;
          noteW(5,pc,nr.rowA,nr.pan);
        } else if (nr.pan>=0 && nr.pan!=colPan[pc]) {
          vgmPutFx(cell,0x08,((nr.pan&2)?0xf0:0)|((nr.pan&1)?0x0f:0),maxFxCol[c]);
          colPan[pc]=nr.pan;
          noteW(5,pc,nr.rowA,nr.pan);
        } else if (nr.pan==-2) {
          colPan[pc]=-1;
          noteW(5,pc,nr.rowA,-1);
        }
        // a merged note borrows another shape: it can't use its release tick
        if (nr.relTick>0 && target[nr.set]==&sets[nr.set] && nr.relTick<=255) {
          vgmPutFx(cell,0xe7,nr.relTick,maxFxCol[c]);
        }
        // arpeggio effect: the note's own, or stop the last one
        if (!nr.arpFx.empty()) {
          for (auto& af: nr.arpFx) {
            if (af.second==colArp[pc]) continue;
            vgmPutFx(cellAt(c,af.first),0x00,af.second,maxFxCol[c]);
            colArp[pc]=af.second;
            noteW(2,pc,af.first,af.second);
          }
        } else if (colArp[pc]!=0) {
          vgmPutFx(cell,0x00,0x00,maxFxCol[c]);
          colArp[pc]=0;
          noteW(2,pc,nr.rowA,0);
        }
        // vibrato effect: the note's own (off until its row), or stop the last one
        if (nr.vibRow>=0) {
          if (nr.vibRow!=nr.rowA && colVib[pc]!=0) {
            vgmPutFx(cell,0x04,0x00,maxFxCol[c]);
            colVib[pc]=0;
            noteW(4,pc,nr.rowA,0);
          }
          if (colVib[pc]!=nr.vibVal) {
            short* vc=cellAt(c,nr.vibRow);
            if (nr.vibDelay>0 && nr.vibRow!=nr.rowA) vgmPutFx(vc,0xed,nr.vibDelay,maxFxCol[c]);
            if ((nr.vibVal>>8)!=colShape[pc]) {
              vgmPutFx(vc,0xe3,nr.vibVal>>8,maxFxCol[c]);
              colShape[pc]=nr.vibVal>>8;
            }
            vgmPutFx(vc,0x04,nr.vibVal&0xff,maxFxCol[c]);
            colVib[pc]=nr.vibVal;
            noteW(4,pc,nr.vibRow,nr.vibVal);
          }
        } else if (colVib[pc]!=0) {
          vgmPutFx(cell,0x04,0x00,maxFxCol[c]);
          colVib[pc]=0;
          noteW(4,pc,nr.rowA,0);
        }
        // legato notes: EA01 on the first one, EA00 on the row after the
        // last one (before the next note, so that one starts normally)
        for (size_t li=0; li<nr.legato.size(); li++) {
          const std::array<int,4>& lg=nr.legato[li];
          short* lc=cellAt(c,lg[0]);
          lc[DIV_PAT_NOTE]=lg[2];
          if (li==0) {
            vgmPutFx(lc,0xea,0x01,maxFxCol[c]);
            noteW(3,pc,lg[0],1);
          }
          if (lg[3]!=colFine[pc]) {
            vgmPutFx(lc,0xe5,0x80+lg[3],maxFxCol[c]);
            colFine[pc]=lg[3];
            noteW(1,pc,lg[0],lg[3]);
          }
          if (lg[1]>0) vgmPutFx(lc,0xed,lg[1],maxFxCol[c]);
        }
        if (!nr.legato.empty() && nr.legato.back()[0]+1<numRows) {
          vgmPutFx(cellAt(c,nr.legato.back()[0]+1),0xea,0x00,maxFxCol[c]);
          noteW(3,pc,nr.legato.back()[0]+1,0);
        }
        if (nr.useOff && nr.cutInNoteRow) {
          vgmPutFx(cell,0xec,nr.cutTicks,maxFxCol[c]);
        } else if (nr.useOff) {
          if (nr.delayS==0) {
            short* offCell=cellAt(c,nr.rowS);
            if (offCell[DIV_PAT_NOTE]==-1) offCell[DIV_PAT_NOTE]=DIV_NOTE_OFF;
            // (and the arpeggio stops with the note)
            if (colArp[pc]!=0) {
              vgmPutFx(offCell,0x00,0x00,maxFxCol[c]);
              colArp[pc]=0;
              noteW(2,pc,nr.rowS,0);
            }
          } else {
            vgmPutFx(cellAt(c,nr.rowS),0xec,nr.delayS,maxFxCol[c]);
          }
        }
      }
      // the loop row: what the channel had there the first time (volume, fine
      // tune, arpeggio, legato), if the end of the song left it different
      if (loopRow>=0) {
        // a channel that was silent at the loop point the first time, but
        // still plays at the end of the song: stop it there, or the last
        // note goes on into the loop
        if (loopTick>=0 && loopTick<numTicks) {
          for (int c: tonal) {
            if (patIdx[c]<0) continue;
            if (chanTicks[c][loopTick].aud || !chanTicks[c][numTicks-1].aud) continue;
            // (a channel shared with another one that plays there is fine)
            bool otherPlays=false;
            for (int c2: tonal) if (c2!=c && patIdx[c2]==patIdx[c] && chanTicks[c2][loopTick].aud) otherPlays=true;
            if (otherPlays) continue;
            short* lc=cellAt(c,loopRow);
            if (lc[DIV_PAT_NOTE]==-1) lc[DIV_PAT_NOTE]=DIV_NOTE_OFF;
          }
        }
        // (pc -> any internal channel of it, for the effect columns)
        std::map<int,int> anyChan;
        for (int c: tonal) if (patIdx[c]>=0 && !anyChan.count(patIdx[c])) anyChan[patIdx[c]]=c;
        static const int initState[6]={15,0,0,0,0,3};
        for (int prop=0; prop<6; prop++) {
          for (auto& sw: stateW[prop]) {
            std::vector<std::pair<int,int>>& w=sw.second;
            std::stable_sort(w.begin(),w.end(),[](const std::pair<int,int>& x, const std::pair<int,int>& y) {
              return x.first<y.first;
            });
            int c=anyChan.count(sw.first)?anyChan[sw.first]:-1;
            if (c<0) continue;
            // (the YM2612's volume starts at 127)
            const int init=(prop==0 && isOPNChan(c))?127:initState[prop];
            int atLoop=init;
            bool onLoopRow=false;
            for (auto& x: w) {
              if (x.first<loopRow) atLoop=x.second;
              if (x.first==loopRow) onLoopRow=true;
            }
            int atEnd=w.empty()?init:w.back().second;
            if (onLoopRow || atLoop==atEnd) continue;
            short* lc=cellAt(c,loopRow);
            switch (prop) {
              case 0:
                if (lc[DIV_PAT_VOL]==-1) lc[DIV_PAT_VOL]=atLoop;
                break;
              case 1:
                vgmPutFx(lc,0xe5,0x80+atLoop,maxFxCol[c]);
                break;
              case 2:
                vgmPutFx(lc,0x00,atLoop,maxFxCol[c]);
                break;
              case 3:
                vgmPutFx(lc,0xea,atLoop,maxFxCol[c]);
                break;
              case 4:
                // (and its shape, if it has one)
                if (atLoop!=0) vgmPutFx(lc,0xe3,atLoop>>8,maxFxCol[c]);
                vgmPutFx(lc,0x04,atLoop&0xff,maxFxCol[c]);
                break;
              case 5:
                // (a pan macro sets it there by itself)
                if (atLoop>=0 && c>=44 && c<52 && log.hasRF) {
                  vgmPutFx(lc,0x08,((atLoop&15)<<4)|((atLoop>>4)&15),maxFxCol[c]);
                } else if (atLoop>=0 && c>=44 && c<52) {
                  int pl=atLoop>>8, pr=atLoop&0xff;
                  vgmPutFx(lc,0x81,(pl>=127)?255:(pl*2),maxFxCol[c]);
                  vgmPutFx(lc,0x82,(pr>=127)?255:(pr*2),maxFxCol[c]);
                } else if (atLoop>=0) {
                  vgmPutFx(lc,0x08,((atLoop&2)?0xf0:0)|((atLoop&1)?0x0f:0),maxFxCol[c]);
                }
                break;
            }
          }
        }
      }
    }
    for (size_t i=0; i<sets.size(); i++) {
      if (created[i]) ctx.prevSets.push_back(sets[i]);
    }

    // YM2612 DAC: each sample is a note on FM 6 with a sample instrument
    // (Furnace turns the DAC on for a sample instrument, off for an FM one).
    // - samples that start in the same row are one sample (what the DAC
    //   played from the first one's start, the second one cutting the first).
    // - the samples are made longest first, and a hit that is the start of a
    //   longer one (cut short by the driver) or almost the same as one (the
    //   timing of the writes moves a little) plays that one, cut where it
    //   stopped. the same data at another rate is the same sample on another
    //   note
    if (log.hasOPN && !opnDacStreams.empty()) {
      const int c=38;
      std::map<String,int>& pcmSampleMap=ctx.pcmSampleMap;
      int& pcmCount=ctx.pcmCount;
      int dropped=0, merged=0;
      struct DACHit {
        int tick, row, delay;
        double start, end; // (end: when its last value ends)
        double rate;
        std::vector<signed char> data;
        int sample=-1, note=108;
        // (fine tune, 1/128 semitone: the hit's rate exactly)
        int fine=0;
        // its values are the writes themselves (not taken from their timing)
        bool regular=true;
      };
      std::vector<DACHit> hits;
      // which row each sample goes to. one that starts in the row of the
      // last one (a sample right after a short one) goes to the next row,
      // a moment late but a note of its own; only if that is not enough it
      // joins the last one (what the DAC played: the second cutting the first)
      struct DACGroup {
        int row, delay, tick;
        bool moved;
        std::vector<size_t> st;
      };
      std::vector<DACGroup> groups;
      int moved=0;
      {
        int lastRow=-1;
        for (size_t si=0; si<opnDacStreams.size(); si++) {
          OPNDACStream& s=opnDacStreams[si];
          int natRow=rowOf(s.tick+shift);
          if (natRow>=numRows) break;
          int row=natRow;
          int delay=(s.tick+shift)-rowStart[row];
          bool mv=false;
          if (natRow<=lastRow) {
            row=lastRow+1;
            delay=0;
            mv=true;
            if (row>=numRows || row-natRow>1 || groups.empty()) {
              if (!groups.empty()) {
                groups.back().st.push_back(si);
                merged++;
              }
              continue;
            }
          }
          short* cell=cellAt(c,row);
          // (an FM 6 note there)
          if ((cell[DIV_PAT_NOTE]!=-1 && cell[DIV_PAT_NOTE]!=DIV_NOTE_OFF) || delay>255) {
            dropped++;
            continue;
          }
          DACGroup g;
          g.row=row;
          g.delay=delay;
          g.tick=mv?(rowStart[row]-shift):s.tick;
          g.moved=mv;
          g.st.push_back(si);
          groups.push_back(g);
          if (mv) moved++;
          lastRow=row;
        }
      }
      for (DACGroup& g: groups) {
        OPNDACStream& s=opnDacStreams[g.st[0]];
        std::vector<VGMOPNDACWrite> w;
        for (size_t k: g.st) w.insert(w.end(),opnDacStreams[k].w.begin(),opnDacStreams[k].w.end());
        DACHit h;
        h.tick=g.tick;
        h.row=g.row;
        h.delay=g.delay;
        h.rate=s.rate;
        h.start=w.front().time;
        h.end=w.back().time+44100.0/s.rate;
        // (a sample moved to the next row plays later than it did: where
        // it ended moves too)
        if (g.moved) {
          double late=(t0+((double)g.tick+0.5)*period)-h.start;
          if (late>0) {
            h.start+=late;
            h.end+=late;
          }
        }
        const bool regular=(g.st.size()==1) && s.regular;
        h.regular=regular;
        if (regular) {
          h.data.resize(w.size());
          for (size_t k=0; k<w.size(); k++) h.data[k]=(signed char)(w[k].val-128);
        } else {
          // the DAC holds each value until the next write
          double start=w.front().time;
          double dur=w.back().time-start;
          size_t count=(size_t)round(dur*h.rate/44100.0)+1;
          h.data.resize(count);
          size_t wi=0;
          for (size_t k=0; k<count; k++) {
            double tt=start+(double)k*44100.0/h.rate;
            while (wi+1<w.size() && w[wi+1].time<=tt+0.001) wi++;
            h.data[k]=(signed char)(w[wi].val-128);
          }
        }
        hits.push_back(h);
      }

      // the samples, longest first
      struct DACSmp {
        int index;
        double rate;
        bool regular;
        std::vector<signed char> data;
      };
      std::vector<DACSmp> smps;
      std::vector<int> byLen(hits.size());
      for (size_t i=0; i<hits.size(); i++) byLen[i]=(int)i;
      std::stable_sort(byLen.begin(),byLen.end(),[&](int x, int y) {
        return hits[x].data.size()>hits[y].data.size();
      });
      int reused=0;
      for (int hi: byLen) {
        DACHit& h=hits[hi];
        const size_t hl=h.data.size();
        int best=-1, bestNote=108, bestFine=0;
        long long bestDiff=-1;
        // (how much the hit moves: a hit that is nearly flat could pass for
        // the start of anything)
        int changes=0;
        for (size_t k=1; k<hl; k++) if (h.data[k]!=h.data[k-1]) changes++;
        for (size_t j=0; j<smps.size(); j++) {
          DACSmp& sm=smps[j];
          // (a little longer is fine too: its ends may hold a few values of
          // the samples around it)
          if (sm.data.size()+16<hl) continue;
          // the note (and fine tune) that plays it at this hit's rate: drivers
          // play their samples at rates of their own (toms, rolls), not only
          // on semitones
          double semis=12.0*std::log(h.rate/sm.rate)/std::log(2.0);
          int n=(int)round(semis);
          if (108+n<0 || 108+n>=180) continue;
          int fn=(int)round((semis-(double)n)*128.0);
          // (a couple of cents: the rate of a hit is only known that well)
          if (abs(fn)<=3) fn=0;
          long long sum=0;
          bool ok=true;
          if (h.regular && sm.regular) {
            // the same values, one by one: the hit is the sample, or its
            // start (cut short), with enough sound in it to tell.
            // where a driver starts a sample right after another one, the
            // place it changes is only known within a few values: the hit
            // may start up to 8 values early or late, and its last 8 values
            // may be the next sample's
            if (hl==sm.data.size() && memcmp(h.data.data(),sm.data.data(),hl)==0) {
              sum=0;
            } else {
              if (hl<64+16 || changes<32) continue;
              // (the first and last 8 values of the hit are left out of the
              // comparison, on both sides of the boundary: they may be what
              // was left of the samples around it)
              long long bestShift=-1;
              const size_t L=hl-16;
              for (int e=0; e<=16 && bestShift<0; e++) {
                // e: 0, -1, 1, -2, 2... (the sample's place for the hit's)
                int d=(e&1)?-((e+1)/2):(e/2);
                long long sStart=8+d;
                if (sStart<0 || sStart+(long long)L>(long long)sm.data.size()) continue;
                if (memcmp(h.data.data()+8,sm.data.data()+sStart,L)!=0) continue;
                bestShift=abs(d)+1;
              }
              // almost the same: some drivers put a value of their own in
              // the middle of a sample now and then, or the hit starts a bit
              // into it (a few dozen values). long enough and moving enough
              // to tell, and nearly every value the same
              if (bestShift<0 && L>=256 && changes>=64) {
                long long bestSum=-1;
                for (int d=-40; d<=40; d++) {
                  // (the hit's value i is the sample's i+d; the hit's first
                  // and last 8 are left out, and what is outside the sample)
                  long long i0=std::max(8LL,(long long)-d);
                  long long i1=std::min((long long)hl-8,(long long)sm.data.size()-d);
                  if (i1-i0<256 || i1-i0<(long long)L-48) continue;
                  const size_t Ld=(size_t)(i1-i0);
                  const signed char* hp=h.data.data()+i0;
                  const signed char* sp=sm.data.data()+i0+d;
                  const long long limitSum=(long long)Ld;
                  const size_t limitBig=Ld/50;
                  long long su=0;
                  size_t big=0;
                  bool ok=true;
                  for (size_t k=0; k<Ld; k++) {
                    int dd=abs((int)hp[k]-(int)sp[k]);
                    su+=dd;
                    if (dd>8) big++;
                    if (su>limitSum || big>limitBig) {
                      ok=false;
                      break;
                    }
                  }
                  if (!ok) continue;
                  if (bestSum<0 || su<bestSum) bestSum=su;
                }
                // (always after an exact match)
                if (bestSum>=0) bestShift=1000+bestSum;
              }
              if (bestShift<0) continue;
              sum=bestShift;
            }
          } else {
            // values taken from the timing of the writes move a little: the
            // same length (almost) and almost the same values
            if (sm.data.size()<hl || sm.data.size()-hl>std::max((size_t)2,hl/100)) continue;
            if (changes<32) continue;
            size_t L=hl;
            long long limitSum=(long long)L;
            size_t limitBig=L/200;
            size_t big=0;
            for (size_t k=0; k<L; k++) {
              int d=abs((int)h.data[k]-(int)sm.data[k]);
              sum+=d;
              if (d>8) big++;
              if (sum>limitSum || big>limitBig) {
                ok=false;
                break;
              }
            }
          }
          if (!ok) continue;
          if (bestDiff<0 || sum<bestDiff) {
            bestDiff=sum;
            best=(int)j;
            bestNote=108+n;
            bestFine=fn;
          }
        }
        if (best>=0) {
          h.sample=smps[best].index;
          h.note=bestNote;
          h.fine=bestFine;
          reused++;
          continue;
        }
        DivSample* smp=new DivSample;
        smp->name=fmt::sprintf("DAC %d",++pcmCount);
        smp->depth=DIV_SAMPLE_DEPTH_8BIT;
        smp->centerRate=(int)round(h.rate);
        smp->init(hl);
        memcpy(smp->data8,h.data.data(),hl);
        int sampleIndex=(int)ds.sample.size();
        ds.sample.push_back(smp);
        String dataKey="dac:";
        dataKey.append((const char*)h.data.data(),hl);
        pcmSampleMap[dataKey]=sampleIndex;
        DACSmp sm;
        sm.index=sampleIndex;
        sm.rate=(double)smp->centerRate;
        sm.regular=h.regular;
        sm.data=h.data;
        smps.push_back(sm);
        h.sample=sampleIndex;
        h.note=108;
      }

      // the notes, and a cut where a longer sample would go on past the hit
      // (unless the next note on FM 6 comes first)
      std::vector<int>& fm6On=onsets[37];
      int cuts=0;
      // (the fine tune each DAC note's row needs)
      std::map<int,int> dacFineAt;
      for (size_t i=0; i<hits.size(); i++) {
        DACHit& h=hits[i];
        String insKey=fmt::sprintf("pcm|%d",h.sample);
        int insIndex;
        auto ii=insMap.find(insKey);
        if (ii!=insMap.end()) {
          insIndex=ii->second;
        } else {
          if (ds.ins.size()>=256) {
            instrumentsOver++;
            continue;
          }
          DivInstrument* ins=new DivInstrument;
          ins->type=DIV_INS_AMIGA;
          ins->name=ds.sample[h.sample]->name;
          ins->amiga.initSample=h.sample;
          insIndex=(int)ds.ins.size();
          ds.ins.push_back(ins);
          insMap[insKey]=insIndex;
        }
        short* cell=cellAt(c,h.row);
        cell[DIV_PAT_NOTE]=h.note;
        cell[DIV_PAT_INS]=insIndex;
        if (h.delay>0) vgmPutFx(cell,0xed,h.delay,maxFxCol[37]);
        dacFineAt[h.row]=h.fine;
        // how long the sample plays at this note, against the hit
        DivSample* smp=ds.sample[h.sample];
        double playRate=(double)smp->centerRate*pow(2.0,((double)(h.note-108)+(double)h.fine/128.0)/12.0);
        double smpEnd=h.start+(double)smp->samples*44100.0/playRate;
        if (smpEnd-h.end<period) continue;
        int cutTick=tickOf(h.end);
        if (cutTick<=h.tick) cutTick=h.tick+1;
        int nextTick=numTicks;
        if (i+1<hits.size()) nextTick=std::min(nextTick,hits[i+1].tick);
        auto it=std::upper_bound(fm6On.begin(),fm6On.end(),h.tick);
        if (it!=fm6On.end()) nextTick=std::min(nextTick,*it);
        if (cutTick>=nextTick || cutTick>=numTicks) continue;
        int ticks=cutTick-h.tick;
        if (ticks<=255) {
          vgmPutFx(cell,0xec,ticks,maxFxCol[37]);
          cuts++;
        } else {
          int r=rowOf(cutTick+shift);
          if (r<=h.row || r>=numRows) continue;
          short* oc=cellAt(c,r);
          if (oc[DIV_PAT_NOTE]!=-1) continue;
          int d=(cutTick+shift)-rowStart[r];
          if (d==0) {
            oc[DIV_PAT_NOTE]=DIV_NOTE_OFF;
          } else {
            vgmPutFx(oc,0xec,d,maxFxCol[37]);
          }
          cuts++;
        }
      }
      // the fine tune on FM 6: E5xx where a DAC note needs another one, and
      // back to the FM notes' own after it (the FM notes were written
      // thinking only of themselves). at the loop row what the channel has
      // is not known, so the next note sets it again
      {
        const int NONE=-100000;
        int actual=0, fmFine=0;
        bool fmFineSet=false;
        int fineFx=0;
        for (int r=0; r<numRows; r++) {
          if (r==loopRow) actual=NONE;
          short* cl=cellAt(37,r);
          int e5=NONE;
          for (int fx=0; fx<DIV_MAX_EFFECTS; fx++) {
            if (cl[DIV_PAT_FX(fx)]==0xe5 && cl[DIV_PAT_FXVAL(fx)]>=0) e5=cl[DIV_PAT_FXVAL(fx)]-0x80;
          }
          auto di=dacFineAt.find(r);
          if (di!=dacFineAt.end()) {
            if (actual!=di->second) {
              vgmPutFx(cl,0xe5,0x80+di->second,maxFxCol[37]);
              actual=di->second;
              fineFx++;
            }
            continue;
          }
          if (e5!=NONE) {
            fmFine=e5;
            fmFineSet=true;
            actual=e5;
            continue;
          }
          const short nt=cl[DIV_PAT_NOTE];
          if (nt!=-1 && nt!=DIV_NOTE_OFF && nt!=DIV_NOTE_REL && nt!=DIV_MACRO_REL) {
            int want=fmFineSet?fmFine:0;
            if (actual!=want) {
              vgmPutFx(cl,0xe5,0x80+want,maxFxCol[37]);
              actual=want;
              fineFx++;
            }
          }
        }
        if (fineFx>0) logI("VGM import: %d fine tunes on FM 6 for the DAC notes",fineFx);
      }
      logI("VGM import: %d DAC hits, %d samples (%d hits reuse a longer or almost equal one, %d cuts)",(int)hits.size(),(int)smps.size(),reused,cuts);
      if (moved>0) logI("VGM import: %d DAC samples started in the same row as another one and went to the next row",moved);
      if (merged>0) logI("VGM import: %d DAC samples started in the same row as another one and were joined to it",merged);
      if (dropped>0) warns.push_back(fmt::sprintf("%d DAC samples landed on the same row as an FM 6 note and were left out",dropped));
      if (opnDacOff>0) logI("VGM import: %d DAC writes while the DAC was off were left out",opnDacOff);
      if (opnDacEmpty>0) logI("VGM import: %d empty DAC samples (silence) were left out",opnDacEmpty);
    }

    // 32X PWM: each sample a note on the Generic PCM DAC's channel, with a
    // sample instrument (C-4 plays it at its rate). the same data (or the
    // start of a longer one, cut where it stopped) is the same sample
    if (log.hasPWM && !pwmHits.empty()) {
      std::map<String,int>& pcmSampleMap=ctx.pcmSampleMap;
      int& pcmCount=ctx.pcmCount;
      // the rows. a sample that starts in the row of the last one joins it
      // when it goes right on from it (what the PWM played, without a gap),
      // or goes to the next row
      struct PWMNote {
        int row, delay, tick;
        bool moved;
        size_t hit;
      };
      std::vector<PWMNote> pn;
      int movedP=0, droppedP=0, joinedP=0;
      {
        int lastRow=-1;
        for (size_t i=0; i<pwmHits.size(); i++) {
          PWMHit& h=pwmHits[i];
          int natRow=rowOf(h.tick+shift);
          if (natRow>=numRows) break;
          int row=natRow;
          int delay=(h.tick+shift)-rowStart[row];
          bool mv=false;
          if (natRow<=lastRow) {
            PWMHit& prev=pwmHits[pn.back().hit];
            if (fabs(h.start-prev.end)<=2.0*44100.0/prev.rate && fabs(h.rate-prev.rate)<1.0) {
              prev.data.insert(prev.data.end(),h.data.begin(),h.data.end());
              prev.dataR.insert(prev.dataR.end(),h.dataR.begin(),h.dataR.end());
              prev.end=h.end;
              h.data.clear();
              h.dataR.clear();
              joinedP++;
              continue;
            }
            row=lastRow+1;
            delay=0;
            mv=true;
            if (row>=numRows || row-natRow>1) {
              droppedP++;
              continue;
            }
          }
          if (delay>255) {
            droppedP++;
            continue;
          }
          PWMNote n;
          n.row=row;
          n.delay=delay;
          n.tick=mv?(rowStart[row]-shift):h.tick;
          n.moved=mv;
          n.hit=i;
          pn.push_back(n);
          if (mv) movedP++;
          lastRow=row;
        }
      }
      // the samples (the same data, or the start of a longer one, is one)
      struct PWMSmp {
        int index;
        double rate;
        const std::vector<short>* data;
      };
      std::vector<PWMSmp> smps;
      // (in stereo the right output's samples too: one that plays the same
      // on both sides is one sample)
      const int sides=log.pwmStereo?2:1;
      std::vector<int> hitSample[2];
      std::vector<std::pair<int,int>> order; // (side, hit)
      for (int side=0; side<sides; side++) {
        hitSample[side].assign(pwmHits.size(),-1);
        for (PWMNote& n: pn) order.push_back(std::make_pair(side,(int)n.hit));
      }
      auto hitData=[&](int side, int hi) -> std::vector<short>& {
        return side?pwmHits[hi].dataR:pwmHits[hi].data;
      };
      std::stable_sort(order.begin(),order.end(),[&](const std::pair<int,int>& x, const std::pair<int,int>& y) {
        return hitData(x.first,x.second).size()>hitData(y.first,y.second).size();
      });
      int reused=0;
      for (const std::pair<int,int>& o: order) {
        const int side=o.first;
        const int hi=o.second;
        PWMHit& h=pwmHits[hi];
        const std::vector<short>& hd=hitData(side,hi);
        const size_t hl=hd.size();
        if (hl==0) continue;
        // (in stereo, a side that stays silent while the other plays: no note)
        if (log.pwmStereo) {
          int lo=32767, hi2=-32768;
          for (short v: hd) {
            lo=std::min(lo,(int)v);
            hi2=std::max(hi2,(int)v);
          }
          if (hi2-lo<64) continue;
        }
        int found=-1;
        for (size_t j=0; j<smps.size() && found<0; j++) {
          const PWMSmp& sm=smps[j];
          if (fabs(sm.rate-h.rate)>h.rate*0.005) continue;
          if (sm.data->size()<hl) continue;
          // (a cut one needs enough values to tell)
          if (hl<sm.data->size() && hl<256) continue;
          if (memcmp(hd.data(),sm.data->data(),hl*sizeof(short))!=0) continue;
          found=sm.index;
        }
        if (found>=0) {
          hitSample[side][hi]=found;
          reused++;
          continue;
        }
        DivSample* smp=new DivSample;
        smp->name=fmt::sprintf("PWM %d",++pcmCount);
        smp->depth=DIV_SAMPLE_DEPTH_16BIT;
        smp->centerRate=(int)round(h.rate);
        smp->init(hl);
        memcpy(smp->data16,hd.data(),hl*sizeof(short));
        int sampleIndex=(int)ds.sample.size();
        ds.sample.push_back(smp);
        String dataKey="pwm:";
        dataKey.append((const char*)hd.data(),hl*sizeof(short));
        pcmSampleMap[dataKey]=sampleIndex;
        PWMSmp sm;
        sm.index=sampleIndex;
        sm.rate=h.rate;
        sm.data=&hd;
        smps.push_back(sm);
        hitSample[side][hi]=sampleIndex;
      }
      // the samples go in kits: one instrument, each sample on a note of its
      // own (played at the sample's rate), like a drum kit
      std::map<int,std::pair<int,int>> kitSlot; // sample -> (instrument, note)
      std::vector<int> pwmKits;
      int pwmKitFill=0;
      int cutsP=0;
      for (int side=0; side<sides; side++)
      for (size_t ni=0; ni<pn.size(); ni++) {
        const int c=side?56:43;
        PWMNote& n=pn[ni];
        PWMHit& h=pwmHits[n.hit];
        const int si=hitSample[side][n.hit];
        if (si<0) continue;
        std::pair<int,int> slot;
        auto ks=kitSlot.find(si);
        if (ks!=kitSlot.end()) {
          slot=ks->second;
        } else {
          if (pwmKits.empty() || pwmKitFill>=120) {
            if (ds.ins.size()>=256) {
              instrumentsOver++;
              continue;
            }
            DivInstrument* ins=new DivInstrument;
            ins->type=DIV_INS_AMIGA;
            ins->name=pwmKits.empty()?String("PWM Kit"):fmt::sprintf("PWM Kit %d",(int)pwmKits.size()+1);
            ins->amiga.useNoteMap=true;
            for (int k=0; k<180; k++) {
              ins->amiga.noteMap[k].map=-1;
              ins->amiga.noteMap[k].freq=k;
            }
            pwmKits.push_back((int)ds.ins.size());
            ds.ins.push_back(ins);
            pwmKitFill=0;
          }
          slot=std::make_pair(pwmKits.back(),48+pwmKitFill);
          DivInstrument* kit=ds.ins[pwmKits.back()];
          kit->amiga.noteMap[slot.second].map=si;
          // (C-4 plays the sample at its rate)
          kit->amiga.noteMap[slot.second].freq=108;
          pwmKitFill++;
          kitSlot[si]=slot;
        }
        short* cell=cellAt(c,n.row);
        cell[DIV_PAT_NOTE]=slot.second;
        cell[DIV_PAT_INS]=slot.first;
        if (n.delay>0) vgmPutFx(cell,0xed,n.delay,maxFxCol[c]);
        // a longer sample than this hit: cut it where the hit stopped, if
        // the next one doesn't come first
        DivSample* smp=ds.sample[si];
        double smpEnd=h.start+(double)smp->samples*44100.0/(double)smp->centerRate;
        if (n.moved || smpEnd-h.end<period) continue;
        int cutTick=tickOf(h.end);
        if (cutTick<=n.tick) cutTick=n.tick+1;
        int nextTick=(ni+1<pn.size())?pn[ni+1].tick:numTicks;
        if (cutTick>=nextTick || cutTick>=numTicks) continue;
        int ticks=cutTick-n.tick;
        if (ticks<=255) {
          vgmPutFx(cell,0xec,ticks,maxFxCol[c]);
          cutsP++;
        } else {
          int r=rowOf(cutTick+shift);
          if (r<=n.row || r>=numRows) continue;
          short* oc=cellAt(c,r);
          if (oc[DIV_PAT_NOTE]!=-1) continue;
          int d=(cutTick+shift)-rowStart[r];
          if (d==0) {
            oc[DIV_PAT_NOTE]=DIV_NOTE_OFF;
          } else {
            vgmPutFx(oc,0xec,d,maxFxCol[c]);
          }
          cutsP++;
        }
      }
      // (stereo: the left output's channel all the way left, the right one's
      // right, from the start)
      if (log.pwmStereo) {
        vgmPutFx(cellAt(43,0),0x08,0xf0,maxFxCol[43]);
        vgmPutFx(cellAt(56,0),0x08,0x0f,maxFxCol[56]);
      }
      logI("VGM import: PWM: %d hits (%d joined in a row), %d notes, %d samples (%d reuse one) in %d kits, %d cuts, %d moved to the next row, %d empty",(int)pwmHits.size(),joinedP,(int)pn.size(),(int)smps.size(),reused,(int)pwmKits.size(),cutsP,movedP,pwmEmpty);
      if (droppedP>0) warns.push_back(fmt::sprintf("%d PWM samples landed on the same row as others and were left out",droppedP));
    }

    // YM2612 LFO (register $22): 10xy on the first channel where it changes
    if (log.hasOPN) {
      int cur=0;
      int changes=0;
      for (int k=0; k<numTicks; k++) {
        if (opnLFO[k]==cur) continue;
        cur=opnLFO[k];
        int r=rowOf(k+shift);
        if (r>=numRows) break;
        vgmPutFx(cellAt(cmdChan,r),0x10,((cur&8)?0x10:0)|(cur&7),maxFxCol[cmdChan]);
        changes++;
      }
      // (the loop goes back to the LFO it had there)
      if (loopRow>=0 && changes>0 && loopTick>=0 && loopTick<numTicks && opnLFO[loopTick]!=cur) {
        int v=opnLFO[loopTick];
        vgmPutFx(cellAt(cmdChan,loopRow),0x10,((v&8)?0x10:0)|(v&7),maxFxCol[cmdChan]);
      }
      if (opnCh3Ticks>0) warns.push_back("this VGM uses the YM2612's channel 3 special mode (one frequency per operator); FM 3 was imported with one frequency");
      // channel 3's special mode on and off: 18xx on OP1's channel (Furnace's
      // Extended Channel 3 starts with it on)
      if (log.opnExt) {
        bool cur=true;
        int switches=0;
        for (int k=0; k<numTicks; k++) {
          if (opnSpecial[k]==cur) continue;
          cur=opnSpecial[k];
          int r=rowOf(k+shift);
          if (r>=numRows) break;
          vgmPutFx(cellAt(39,r),0x18,cur?1:0,maxFxCol[39]);
          switches++;
        }
        if (loopRow>=0 && switches>0 && loopTick>=0 && loopTick<numTicks && opnSpecial[loopTick]!=cur) {
          vgmPutFx(cellAt(39,loopRow),0x18,opnSpecial[loopTick]?1:0,maxFxCol[39]);
        }
        logI("VGM import: YM2612 channel 3 special mode (Extended Channel 3), %d switches",switches);
      }
    }
    // the instrument names say which channels use them, like "Pulse 3 [P1, P2]"
    {
      static const char* chanName[NCH]={
        "P1","P2","TRI","NOI","DMC","5B-A","5B-B","5B-C","SQ1","SQ2","SQ3","NOI",
        "FM1","FM2","FM3","FM4","FM5","FM6","FM7","FM8","FM9",
        "BD","SD","TOM","TOP","HH",
        "SID1","SID2","SID3",
        "V1","V2","VS",
        "F1","F2","F3","F4","F5","F6","DAC",
        "O1","O2","O3","O4",
        "PWM",
        "S1","S2","S3","S4","S5","S6","S7","S8",
        "PU1","PU2","WAV","NOI",
        "PWM R"
      };
      std::map<int,String> pcName;
      for (int c=0; c<NCH; c++) {
        if (patIdx[c]<0) continue;
        auto it=pcName.find(patIdx[c]);
        if (it==pcName.end()) {
          pcName[patIdx[c]]=chanName[c];
        } else {
          it->second+=String("/")+chanName[c];
        }
      }
      std::set<int> touched;
      for (int pc=0; pc<numChans; pc++) {
        if (!pcName.count(pc)) continue;
        std::set<int> used;
        for (int o=0; o<sub->ordersLen; o++) {
          DivPattern* pat=sub->pat[pc].getPattern(sub->orders.ord[pc][o],false);
          if (pat==NULL) continue;
          for (int r=0; r<sub->patLen; r++) {
            int in=pat->newData[r][DIV_PAT_INS];
            if (in>=0 && in<(int)ds.ins.size()) used.insert(in);
          }
        }
        for (int in: used) {
          std::vector<String>& v=ctx.insChans[in];
          if (std::find(v.begin(),v.end(),pcName[pc])==v.end()) v.push_back(pcName[pc]);
          touched.insert(in);
        }
      }
      for (int in: touched) {
        String& nm=ds.ins[in]->name;
        size_t br=nm.find(" [");
        if (br!=String::npos) nm=nm.substr(0,br);
        nm+=" [";
        const std::vector<String>& v=ctx.insChans[in];
        for (size_t i=0; i<v.size(); i++) {
          if (i>0) nm+=", ";
          nm+=v[i];
        }
        nm+="]";
      }
    }
    logI("VGM import: %d note shapes, %d merged (%d notes), %d scaled matches, %d prefix matches",(int)sets.size(),mergedSets,mergedNotes,scaledMatches,prefixMatches);
    if (loopedLoose>0) logI("VGM import: %d long macros loop within a tick",loopedLoose);
    if (looseVib>0) logI("VGM import: %d YM2612 notes take their vibrato from 04xy (close, not exact) instead of a pitch macro",looseVib);
    ctx.mergedNotes+=mergedNotes;
    if (mergedSets>0) {
      warns.push_back(fmt::sprintf("this song has more note shapes than the 256 instrument limit: %d rare ones (%d notes) use the closest instrument instead, so they may sound slightly different",mergedSets,mergedNotes));
    }

    // YM2413 rhythm mode on and off: 18xx on the kick channel (Furnace's
    // "OPLL with drums" starts with the drums on)
    if (log.hasOPLL && log.opllDrums) {
      bool cur=true;
      int switches=0;
      for (int k=0; k<numTicks; k++) {
        if (opllRhythm[k]==cur) continue;
        cur=opllRhythm[k];
        int r=rowOf(k+shift);
        if (r>=numRows) break;
        vgmPutFx(cellAt(21,r),0x18,cur?1:0,maxFxCol[21]);
        switches++;
      }
      if (loopRow>=0 && switches>0 && loopTick>=0 && loopTick<numTicks && opllRhythm[loopTick]!=cur) {
        vgmPutFx(cellAt(21,loopRow),0x18,opllRhythm[loopTick]?1:0,maxFxCol[21]);
      }
    }

    // SID filter and volume during a note longer than a macro (255 ticks):
    // from there on the changes go to the pattern, on the row they land on
    // (4xxx cutoff, 13xx resonance, 14xx mode, volume column)
    if (log.hasSID) {
      auto gField=[&](int k, int f) -> int {
        const unsigned char* g=&sidGlobal[(size_t)k*4];
        switch (f) {
          case 0: return (g[0]&7)|(g[1]<<3);
          case 1: return g[2]>>4;
          case 2: return g[3]>>4;
          case 3: return g[3]&15;
        }
        return 0;
      };
      int known[4]={-1,-1,-1,-1};
      int lastVolCol[3]={15,15,15};
      int tailFx=0;
      for (int r=0; r<numRows; r++) {
        int k0=rowStart[r]-shift;
        int k1=(r+1<numRows)?(rowStart[r+1]-shift):numTicks;
        if (k0<0 || k0>=numTicks) continue;
        for (int k=k0; k<k1 && k<numTicks; k++) {
          int id=sidOwner[k];
          if (id<0) continue;
          int oc=id>>24, oa=id&0xffffff;
          if (k-oa<255) {
            for (int f=0; f<4; f++) known[f]=gField(k,f);
            continue;
          }
          // (only at the row's start)
          if (k!=k0) continue;
          for (int f=0; f<4; f++) {
            int v=gField(k,f);
            if (v==known[f] || known[f]<0) continue;
            if (f==3 && log.sidDigi) continue;
            short* cell=cellAt(oc,r);
            switch (f) {
              case 0: vgmPutFx(cell,0x40|(v>>8),v&0xff,maxFxCol[oc]); break;
              case 1: vgmPutFx(cell,0x13,v,maxFxCol[oc]); break;
              case 2: vgmPutFx(cell,0x14,v&7,maxFxCol[oc]); break;
              case 3: {
                // (Furnace ignores a volume column equal to the channel's
                // last one, even if a macro changed the volume since: use a
                // channel where it is new; the volume is the chip's anyway)
                int vc=oc;
                for (int t=0; t<3 && lastVolCol[vc-26]==v; t++) vc=26+((vc-26+1)%3);
                cellAt(vc,r)[DIV_PAT_VOL]=v;
                lastVolCol[vc-26]=v;
                break;
              }
            }
            known[f]=v;
            tailFx++;
          }
        }
      }
      if (tailFx>0) logI("SID import: %d filter/volume changes in long notes went to the pattern",tailFx);
      // pulse width of notes longer than their macro: 3xxx on the rows after
      {
        int tails=0;
        for (VGMNoteRec& nr: recs) {
          if (nr.c<26 || nr.c>=29 || !nr.sidDutyCut || nr.sidOnset<0) continue;
          int c=nr.c;
          std::vector<VGMChanTick>& ct=chanTicks[c];
          std::vector<int>& on=onsets[c];
          auto it=std::upper_bound(on.begin(),on.end(),nr.sidOnset);
          int nextOn=(it!=on.end())?*it:numTicks;
          int from=nr.sidOnset+255;
          if (from>=nextOn) continue;
          int known=ct[from-1].duty;
          for (int r=rowOf(from+shift); r<numRows; r++) {
            int k0=rowStart[r]-shift;
            if (k0>=nextOn || k0>=numTicks) break;
            if (k0<from) continue;
            if (ct[k0].duty==known) continue;
            known=ct[k0].duty;
            vgmPutFx(cellAt(c,r),0x30|(known>>8),known&0xff,maxFxCol[c]);
            tails++;
          }
        }
        if (tails>0) logI("SID import: %d pulse width changes in long notes went to the pattern",tails);
      }
      // free-running pulse width: Furnace's goes down by the slide (22xx +,
      // 23xx -) every tick; at each row start the slide that the row needs,
      // and 3xxx where the value is off
      for (int v=0; v<3; v++) {
        if (!sidFreePW[v]) continue;
        std::vector<VGMChanTick>& ct=chanTicks[26+v];
        int P=-1, S=0;
        int fx=0;
        for (int r=0; r<numRows; r++) {
          int k0=rowStart[r]-shift;
          int k1=(r+1<numRows)?(rowStart[r+1]-shift):numTicks;
          if (k0<0 || k0>=numTicks) continue;
          if (k1>numTicks) k1=numTicks;
          // the most common step in this row
          std::map<int,int> dc;
          for (int k=k0+1; k<k1; k++) dc[ct[k].duty-ct[k-1].duty]++;
          int step=0, bc=0;
          for (auto& i: dc) {
            if (i.second>bc || (i.second==bc && i.first==-S)) {
              bc=i.second;
              step=i.first;
            }
          }
          if (k1-k0<2) step=-S;
          int Snew=-step;
          if (Snew>255) Snew=255;
          if (Snew<-255) Snew=-255;
          short* cell=cellAt(26+v,r);
          if (Snew!=S) {
            if (Snew>=0) {
              vgmPutFx(cell,0x22,Snew,maxFxCol[26+v]);
            } else {
              vgmPutFx(cell,0x23,-Snew,maxFxCol[26+v]);
            }
            S=Snew;
            fx++;
          }
          int pred=(P<0)?-1:std::max(0,std::min(4095,P-S));
          if (pred!=ct[k0].duty) {
            int X=std::max(0,std::min(4095,ct[k0].duty+S));
            vgmPutFx(cell,0x30|(X>>8),X&0xff,maxFxCol[26+v]);
            fx++;
            P=std::max(0,std::min(4095,X-S));
          } else {
            P=pred;
          }
          for (int k=k0+1; k<k1; k++) P=std::max(0,std::min(4095,P-S));
        }
        logI("SID import: voice %d: %d pulse width effects",v+1,fx);
      }
      // notes without the hard restart: 1A01 turns Furnace's off for them,
      // 1A00 back on (where the reset would happen)
      if (hrN>0) {
        int masks=0;
        for (int v=0; v<3; v++) {
          bool mask=false;
          std::vector<int>& on=onsets[26+v];
          for (size_t i=1; i<on.size(); i++) {
            int b=on[i];
            bool want=!sidHR[v][b];
            if (want==mask) continue;
            int r=rowOf(std::max(0,b-hrN)+shift);
            if (r>=numRows) break;
            vgmPutFx(cellAt(26+v,r),0x1a,want?1:0,maxFxCol[26+v]);
            mask=want;
            masks++;
          }
        }
        if (masks>0) logI("SID import: %d hard restart switches",masks);
      }
    }

    // SNES: the echo and the main volume changing during the song (the start
    // is in the system's settings): their effects where they change. the
    // echo of each voice (EON) goes to its own channel (12xx), the rest to
    // the first one
    if (log.hasSNES) {
      const unsigned char* in=log.snesInit;
      unsigned char cur[16];
      cur[0]=in[0x6c];
      cur[1]=in[0x7d];
      cur[2]=snesGainDiv(in[0x2c]);
      cur[3]=in[0x3c];
      cur[3]=snesGainDiv(cur[3]);
      cur[4]=in[0x0d];
      for (int i=0; i<8; i++) cur[5+i]=in[(i<<4)|0x0f];
      cur[13]=snesGainDiv(in[0x0c]);
      cur[14]=snesGainDiv(in[0x1c]);
      cur[15]=in[0x4d];
      int fx=0;
      for (int k=0; k<numTicks; k++) {
        const unsigned char* gl=&snesGlobal[(size_t)k*16];
        int r=rowOf(k+shift);
        if (r>=numRows) break;
        for (int i=0; i<16; i++) {
          if (gl[i]==cur[i]) continue;
          short* cell=cellAt(44,r);
          switch (i) {
            case 0:
              // (echo writes on or off)
              if ((gl[0]&0x20)!=(cur[0]&0x20)) {
                vgmPutFx(cell,0x18,(gl[0]&0x20)?0:1,maxFxCol[44]);
                fx++;
              }
              break;
            case 1: vgmPutFx(cell,0x19,gl[1]&15,maxFxCol[44]); fx++; break;
            case 2: vgmPutFx(cell,0x1a,gl[2],maxFxCol[44]); fx++; break;
            case 3: vgmPutFx(cell,0x1b,gl[3],maxFxCol[44]); fx++; break;
            case 4: vgmPutFx(cell,0x1c,gl[4],maxFxCol[44]); fx++; break;
            case 13: vgmPutFx(cell,0x1e,gl[13],maxFxCol[44]); fx++; break;
            case 14: vgmPutFx(cell,0x1f,gl[14],maxFxCol[44]); fx++; break;
            case 15:
              for (int v=0; v<8; v++) {
                if (((gl[15]^cur[15])>>v)&1) {
                  vgmPutFx(cellAt(44+v,r),0x12,(gl[15]>>v)&1,maxFxCol[44+v]);
                  fx++;
                }
              }
              break;
            default:
              // (FIR coefficients)
              vgmPutFx(cell,0x30+(i-5),gl[i],maxFxCol[44]);
              fx++;
              break;
          }
          cur[i]=gl[i];
        }
      }
      // (and at the loop point, what they were there: after the loop the
      // song goes on from that state, not from the end's)
      // (unless that row already sets it: a change right after the loop
      // point, which the row plays anyway)
      auto rowHasFx=[](short* cell, int fxId) -> bool {
        for (int q=0; q<DIV_MAX_EFFECTS; q++) if (cell[DIV_PAT_FX(q)]==fxId) return true;
        return false;
      };
      if (loopTick>=0 && loopTick<numTicks) {
        int lr=rowOf(loopTick+shift);
        if (lr>=0 && lr<numRows) {
          const unsigned char* gl=&snesGlobal[(size_t)loopTick*16];
          short* cell=cellAt(44,lr);
          static const int fxOf[16]={0x18,0x19,0x1a,0x1b,0x1c,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x1e,0x1f,0x12};
          for (int i=0; i<16; i++) {
            if (gl[i]==cur[i]) continue;
            if (i!=15 && rowHasFx(cell,fxOf[i])) continue;
            switch (i) {
              case 0:
                if ((gl[0]&0x20)!=(cur[0]&0x20)) vgmPutFx(cell,0x18,(gl[0]&0x20)?0:1,maxFxCol[44]);
                break;
              case 1: vgmPutFx(cell,0x19,gl[1]&15,maxFxCol[44]); break;
              case 2: vgmPutFx(cell,0x1a,gl[2],maxFxCol[44]); break;
              case 3: vgmPutFx(cell,0x1b,gl[3],maxFxCol[44]); break;
              case 4: vgmPutFx(cell,0x1c,gl[4],maxFxCol[44]); break;
              case 13: vgmPutFx(cell,0x1e,gl[13],maxFxCol[44]); break;
              case 14: vgmPutFx(cell,0x1f,gl[14],maxFxCol[44]); break;
              case 15:
                for (int v=0; v<8; v++) {
                  if (((gl[15]^cur[15])>>v)&1) vgmPutFx(cellAt(44+v,lr),0x12,(gl[15]>>v)&1,maxFxCol[44+v]);
                }
                break;
              default:
                vgmPutFx(cell,0x30+(i-5),gl[i],maxFxCol[44]);
                break;
            }
            fx++;
          }
        }
      }
      if (fx>0) logI("SPC import: %d echo/volume effects",fx);

      // the voices' negative volumes (the "surround" some drivers use: one
      // side, or both, with the wave upside down): Furnace's invert (14xy)
      // where they change while the voice plays, and at the loop point
      {
        int inv=0;
        for (int v=0; v<8; v++) {
          int curInv=0, atLoop=-1;
          for (int k=0; k<numTicks; k++) {
            const unsigned char q=snesInv[(size_t)k*8+v];
            if (k==loopTick) atLoop=curInv;
            if (!(q&0x80)) continue;
            const int want=q&3;
            if (k==loopTick) atLoop=want;
            if (want==curInv) continue;
            int r=rowOf(k+shift);
            if (r<0 || r>=numRows) break;
            vgmPutFx(cellAt(44+v,r),0x14,((want&1)?0x10:0)|((want&2)?1:0),maxFxCol[44+v]);
            curInv=want;
            inv++;
          }
          if (atLoop>=0 && atLoop!=curInv) {
            int lr=rowOf(loopTick+shift);
            if (lr>=0 && lr<numRows && !rowHasFx(cellAt(44+v,lr),0x14)) {
              vgmPutFx(cellAt(44+v,lr),0x14,((atLoop&1)?0x10:0)|((atLoop&2)?1:0),maxFxCol[44+v]);
              inv++;
            }
          }
        }
        if (inv>0) logI("SPC import: %d invert effects",inv);
      }
    }
    if (log.hasSNES) {
      if (snesNoiseTicks>0) warns.push_back("this song uses the SNES's noise, which is not imported yet");
      if (snesPmonTicks>0) warns.push_back("this song uses pitch modulation, which is not imported yet");
      (void)snesInvTicks;
    }

    // --- step 12: loop or end ---
    {
      int lastRow=numRows-1;
      short* cell=cellAt(cmdChan,lastRow);
      if (loopRow>=0) {
        int lr=loopRow;
        int ld=loopTick-rowStart[lr];
        if (ld!=0) warns.push_back("the loop point does not start on a row; it was moved to the nearest row");
        if ((lr%patLen)!=0) vgmPutFx(cell,0x0d,lr%patLen,maxFxCol[cmdChan]);
        vgmPutFx(cell,0x0b,lr/patLen,maxFxCol[cmdChan]);
      } else {
        vgmPutFx(cell,0xff,0x00,maxFxCol[cmdChan]);
      }
    }

    for (int c=0; c<NCH; c++) {
      if (patIdx[c]<0) continue;
      sub->pat[patIdx[c]].effectCols=1;
    }
    for (int c=0; c<NCH; c++) {
      if (patIdx[c]<0) continue;
      DivChannelData& cd=sub->pat[patIdx[c]];
      cd.effectCols=std::min(DIV_MAX_EFFECTS,std::max((int)cd.effectCols,maxFxCol[c]));
    }

    logI("NES import: %d ticks at %.3f Hz, %d rows, %d instruments, %d samples",numTicks,44100.0/period,numRows,(int)ds.ins.size(),(int)ds.sample.size());
    return true;
}

// --- Ricoh RF5C68/RF5C164 (Sega CD PCM, FM Towns) ---

// what a VGM does to the chip, in order: register writes (kind 0: a is the
// register, b the value), memory writes (kind 1: a is the address in the
// bank window, b the value) and RAM blocks (kind 2: a is the address in the
// bank window, the data is len bytes of the blob from src)
struct VGMRFOp {
  double time;
  int kind;
  unsigned int a, b;
  size_t src, len;
};

// runs the chip from the VGM's writes, following the read pointer of each
// channel: what a note plays is what that channel reads, from its key on
// until its key off (drivers that stream their samples through a ring
// buffer get the whole sample this way, not what the RAM holds at the key
// on). the events: 0x800|(channel<<4)|register for the channel registers
// (0-6), 0x880 the key off register (8: a bit set is a channel off), 0x881
// the chip on or off (bit 7 of register 7), 0x900|channel the sample of a
// key on (an index into samples), 0xA00|channel 1 when the channel is
// left reading end markers only (silent), 0 when it plays again.
// samples: the raw bytes (sign and magnitude) in brr, the loop in loopBlock
// (a byte position, or -1), the start address in addr
void vgmRunRF(const std::vector<VGMRFOp>& ops, const std::vector<unsigned char>& blob, double clock, double endTime, std::vector<VGMEvent>& out, std::vector<VGMSNESSample>& samples, std::vector<String>& warns, bool& needStream) {
  const double rate=clock/384.0;
  const double tPer=44100.0/rate;
  std::vector<unsigned char> ram(65536,0);
  std::vector<unsigned int> wseq(65536,0);
  unsigned int seq=1;
  // (per channel: the pass each byte was read in, its place in the note's
  // data, and when it was read)
  std::vector<unsigned int> passStamp((size_t)8*65536,0);
  std::vector<int> passIdx((size_t)8*65536,0);
  std::vector<unsigned int> appSeq((size_t)8*65536,0);
  struct Note {
    int evIdx;
    std::vector<unsigned char> data;
    int loop;
    unsigned int addr;
  };
  std::vector<Note> notes;
  struct Chan {
    unsigned char env=0, pan=0, st=0;
    unsigned int fd=0, ls=0;
    bool on=false;
    unsigned int addr=0;
    bool silent=false;
    // the note: its data so far
    int note=-1;
    bool collecting=false;
    int lastPos=-1;
    unsigned int pass=1;
  } ch[8];
  int wbank=0, cbank=0;
  bool enable=false;
  int truncated=0;
  size_t n=0;
  auto evAt=[&](double t, int reg, int val) {
    VGMEvent e;
    e.time=t;
    e.reg=reg;
    e.val=val;
    out.push_back(e);
  };
  auto finish=[&](int v) {
    Chan& c=ch[v];
    c.note=-1;
    c.collecting=false;
  };
  // the bytes from where the channel was to p (the ones it skips too)
  auto append=[&](int v, unsigned int p) {
    Chan& c=ch[v];
    if (c.note<0 || !c.collecting) return;
    Note& nt=notes[c.note];
    int from=c.lastPos+1;
    if ((int)p<c.lastPos) {
      // (the address went round the memory)
      c.pass++;
      from=(int)p;
    }
    for (int i=from; i<=(int)p; i++) {
      size_t ix=(size_t)v*65536+i;
      unsigned char x=ram[i];
      // (an end marker it skips over: the chip never sees it)
      if (x==0xff) x=0xfe;
      passStamp[ix]=c.pass;
      passIdx[ix]=(int)nt.data.size();
      appSeq[ix]=seq;
      nt.data.push_back(x);
    }
    if ((int)p>c.lastPos) c.lastPos=(int)p;
    if (nt.data.size()>=4194304) {
      c.collecting=false;
      truncated++;
    }
  };
  // DEBUGREF: the chip's own output, to compare with the import
  const char* rfRef=getenv("FURNACE_RF_REF");
  std::vector<short> refOut;
  auto stepAll=[&]() {
    int mixL=0, mixR=0;
    if (!enable) {
      if (rfRef) {
        refOut.push_back(0);
        refOut.push_back(0);
      }
      return;
    }
    double t=(double)n*tPer;
    for (int v=0; v<8; v++) {
      Chan& c=ch[v];
      if (!c.on) continue;
      unsigned int p=(c.addr>>11)&0xffff;
      unsigned char b=ram[p];
      if (b==0xff) {
        unsigned int q=c.ls&0xffff;
        if (c.collecting && c.note>=0) {
          // a loop: the channel goes back to bytes it already read, and
          // nothing wrote over them since. otherwise the driver is
          // streaming: the note goes on with the new bytes
          bool isLoop=false;
          size_t qi=(size_t)v*65536+q;
          if (passStamp[qi]==c.pass) {
            isLoop=true;
            for (unsigned int i=q; i!=p; i=(i+1)&0xffff) {
              size_t ix=(size_t)v*65536+i;
              if (passStamp[ix]!=c.pass) continue;
              if (wseq[i]>appSeq[ix]) {
                isLoop=false;
                break;
              }
            }
          }
          if (isLoop) {
            notes[c.note].loop=passIdx[qi];
            c.collecting=false;
          } else {
            c.pass++;
            c.lastPos=(int)q-1;
          }
        }
        c.addr=q<<11;
        p=q;
        b=ram[p];
        if (b==0xff) {
          // (only end markers: silent until the next key on)
          if (!c.silent) {
            c.silent=true;
            evAt(t,0xa00|v,1);
          }
          c.collecting=false;
          continue;
        }
      }
      append(v,p);
      c.addr=(c.addr+c.fd)&0x7ffffff;
      if (rfRef) {
        int lv=(c.pan&15)*c.env, rv=((c.pan>>4)&15)*c.env;
        int lo=((b&0x7f)*lv)>>5, ro=((b&0x7f)*rv)>>5;
        if (!(b&0x80)) {
          lo=-lo;
          ro=-ro;
        }
        mixL+=lo;
        mixR+=ro;
      }
    }
    if (rfRef) {
      refOut.push_back((short)std::max(-32768,std::min(32767,mixL)));
      refOut.push_back((short)std::max(-32768,std::min(32767,mixR)));
    }
  };
  for (const VGMRFOp& op: ops) {
    while ((double)n*tPer<op.time) {
      stepAll();
      n++;
    }
    double t=op.time;
    switch (op.kind) {
      case 0: {
        int r=op.a&15;
        int val=op.b&0xff;
        if (r<=6) {
          Chan& c=ch[cbank];
          switch (r) {
            case 0: c.env=val; break;
            case 1: c.pan=val; break;
            case 2: c.fd=(c.fd&0xff00)|val; break;
            case 3: c.fd=(c.fd&0xff)|(val<<8); break;
            case 4: c.ls=(c.ls&0xff00)|val; break;
            case 5: c.ls=(c.ls&0xff)|(val<<8); break;
            case 6:
              c.st=val;
              if (!c.on) c.addr=(unsigned int)c.st<<19;
              break;
          }
          evAt(t,0x800|(cbank<<4)|r,val);
        } else if (r==7) {
          if (val&0x40) {
            cbank=val&7;
          } else {
            wbank=val&15;
          }
          bool en=val&0x80;
          if (en!=enable) evAt(t,0x881,en?1:0);
          enable=en;
        } else if (r==8) {
          for (int v=0; v<8; v++) {
            Chan& c=ch[v];
            bool nowOn=!((val>>v)&1);
            if (nowOn && !c.on) {
              // key on: a new note from the start address
              if (c.note>=0) finish(v);
              c.addr=(unsigned int)c.st<<19;
              c.silent=false;
              Note nt;
              nt.evIdx=(int)out.size();
              nt.loop=-1;
              nt.addr=(unsigned int)c.st<<8;
              notes.push_back(nt);
              c.note=(int)notes.size()-1;
              c.collecting=true;
              c.pass++;
              c.lastPos=(int)(c.st<<8)-1;
              evAt(t,0x900|v,-1);
              evAt(t,0xa00|v,0);
            } else if (!nowOn && c.on) {
              finish(v);
            }
            c.on=nowOn;
            if (!c.on) c.addr=(unsigned int)c.st<<19;
          }
          evAt(t,0x880,val);
        }
        break;
      }
      case 1: {
        unsigned int a=((unsigned int)wbank<<12)|(op.a&0xfff);
        a&=0xffff;
        ram[a]=op.b&0xff;
        wseq[a]=seq++;
        break;
      }
      case 2: {
        unsigned int a=(op.a|((unsigned int)wbank<<12));
        for (size_t i=0; i<op.len && a+i<65536 && op.src+i<blob.size(); i++) {
          ram[a+i]=blob[op.src+i];
          wseq[a+i]=seq;
        }
        seq++;
        break;
      }
    }
  }
  // (the notes still playing at the end)
  while ((double)n*tPer<endTime) {
    stepAll();
    n++;
  }
  for (int v=0; v<8; v++) if (ch[v].note>=0) finish(v);
  if (rfRef) {
    FILE* rf=fopen(rfRef,"wb");
    if (rf) {
      unsigned int sr=(unsigned int)round(rate), br=sr*4;
      unsigned int dataLen=refOut.size()*2, rl=dataLen+36;
      unsigned char h[44]={'R','I','F','F',0,0,0,0,'W','A','V','E','f','m','t',' ',16,0,0,0,1,0,2,0,0,0,0,0,0,0,0,0,4,0,16,0,'d','a','t','a',0,0,0,0};
      memcpy(h+4,&rl,4);
      memcpy(h+24,&sr,4);
      memcpy(h+28,&br,4);
      memcpy(h+40,&dataLen,4);
      fwrite(h,1,44,rf);
      fwrite(refOut.data(),2,refOut.size(),rf);
      fclose(rf);
    }
  }

  // the samples: one per different note data. a note that only got part of
  // another one's (cut short by the next note) plays that one
  std::map<String,int> exact;
  std::vector<int> noteSmp(notes.size(),-1);
  std::vector<int> smpNote;
  for (size_t i=0; i<notes.size(); i++) {
    Note& nt=notes[i];
    // (the silence at the end of a note without a loop: drivers often let
    // the channel run on through silence after the sound. the sample ends
    // there, which sounds the same)
    if (nt.loop<0) {
      size_t e=nt.data.size();
      while (e>0 && (nt.data[e-1]&0x7f)<=1) e--;
      nt.data.resize(e);
    }
    if (nt.data.empty()) continue;
    String key=fmt::sprintf("%d:",nt.loop);
    key.append((const char*)nt.data.data(),nt.data.size());
    auto it=exact.find(key);
    if (it!=exact.end()) {
      noteSmp[i]=it->second;
      continue;
    }
    exact[key]=(int)smpNote.size();
    noteSmp[i]=(int)smpNote.size();
    smpNote.push_back((int)i);
  }
  // (longest first: a shorter one without a loop that starts the same is
  // played by the longer one)
  std::vector<int> order(smpNote.size());
  for (size_t i=0; i<order.size(); i++) order[i]=(int)i;
  std::sort(order.begin(),order.end(),[&](int x, int y) {
    return notes[smpNote[x]].data.size()>notes[smpNote[y]].data.size();
  });
  std::vector<int> remap(smpNote.size(),-1);
  std::vector<int> kept;
  for (int s: order) {
    const Note& ns=notes[smpNote[s]];
    if (ns.loop<0) {
      for (int k: kept) {
        const Note& nk=notes[smpNote[k]];
        if (nk.data.size()<ns.data.size()) continue;
        // (a looped one plays its data past the loop the same way)
        if (memcmp(nk.data.data(),ns.data.data(),ns.data.size())==0) {
          remap[s]=k;
          break;
        }
      }
      if (remap[s]>=0) continue;
    }
    remap[s]=s;
    kept.push_back(s);
  }
  std::vector<int> outIdx(smpNote.size(),-1);
  for (int k: kept) {
    const Note& nk=notes[smpNote[k]];
    VGMSNESSample sm;
    sm.brr=nk.data;
    sm.loopBlock=nk.loop;
    sm.addr=nk.addr;
    outIdx[k]=(int)samples.size();
    samples.push_back(sm);
  }
  for (size_t i=0; i<notes.size(); i++) {
    int s=noteSmp[i];
    out[notes[i].evIdx].val=(s>=0)?outIdx[remap[s]]:-1;
  }
  // the memory: Furnace puts each sample on a 256-byte boundary, with 32
  // end markers after it. if they don't fit in 64 KB, the chip streams
  // them (a ring buffer per channel, filled from the sample as it plays),
  // like the game's driver does
  size_t total=0;
  for (const VGMSNESSample& sm: samples) total+=(sm.brr.size()+32+255)&~(size_t)255;
  needStream=(total>65536);
  logI("RF5C68 import: %d notes, %d samples, %d bytes of samples%s",(int)notes.size(),(int)samples.size(),(int)total,needStream?" (streamed)":"");
  if (truncated>0) warns.push_back(fmt::sprintf("%d PCM notes were longer than 4 MB and were cut",truncated));
}



}

bool DivEngine::loadVGM(unsigned char* file, size_t len) {
  bool success=false;
  warnings="";
  std::vector<unsigned char> b;

  try {
    // .vgz is gzip
    if (len>=2 && file[0]==0x1f && file[1]==0x8b) {
      if (!vgmGunzip(file,len,b)) {
        lastError="could not decompress the .vgz file";
        throw VGMInvalidException();
      }
    } else {
      b.assign(file,file+len);
    }
    if (b.size()<0x40 || memcmp(b.data(),"Vgm ",4)!=0) {
      lastError="not a VGM file";
      throw VGMInvalidException();
    }

    unsigned int version=vgmRead32(b,0x08);
    unsigned int gd3Off=vgmRead32(b,0x14);
    unsigned int totalSamples=vgmRead32(b,0x18);
    unsigned int loopOff=vgmRead32(b,0x1c);
    unsigned int headerRate=(version>=0x101)?vgmRead32(b,0x24):0;
    size_t dataStart=0x40;
    if (version>=0x150 && vgmRead32(b,0x34)!=0) dataStart=0x34+vgmRead32(b,0x34);
    if (dataStart>=b.size()) {
      lastError="VGM data offset is out of range";
      throw VGMInvalidException();
    }
    size_t loopPos=(loopOff!=0)?(0x1c+loopOff):0;

    unsigned int nesClockRaw=0;
    if (version>=0x161 && dataStart>0x84+4) nesClockRaw=vgmRead32(b,0x84);
    double nesClock=(double)(nesClockRaw&0x3fffffff);
    bool hasFDS=nesClockRaw&0x80000000;
    bool dualNES=nesClockRaw&0x40000000;

    // SN76489 (Master System, Game Gear, Mega Drive PSG...)
    unsigned int snClockRaw=vgmRead32(b,0x0c);
    double snClock=(double)(snClockRaw&0x3fffffff);
    bool dualSN=snClockRaw&0x40000000;
    bool t6w28=snClockRaw&0x80000000;
    unsigned int snFeedback=(version>=0x110)?(b[0x28]|(b[0x29]<<8)):0x0009;
    unsigned int snWidth=(version>=0x110)?b[0x2a]:16;
    if (snFeedback==0) snFeedback=0x0009;
    if (snWidth==0) snWidth=16;

    // YM2413 (MSX-MUSIC, Master System FM...). bit 31 means a VRC7 (the
    // Famicom's Konami expansion, an OPLL with its own patches)
    unsigned int opllClockRaw=vgmRead32(b,0x10);
    double opllClock=(double)(opllClockRaw&0x3fffffff);
    bool opllVRC7=opllClockRaw&0x80000000;
    bool dualOPLL=opllClockRaw&0x40000000;

    // YM2612 (Mega Drive/Genesis, FM Towns). bit 31 means a YM3438
    unsigned int opnClockRaw=(version>=0x110)?vgmRead32(b,0x2c):0;
    // (before 1.10 the YM2612 shared the YM2413's clock)
    if (version<0x110) opnClockRaw=0;
    double opnClock=(double)(opnClockRaw&0x3fffffff);
    bool opnYM3438=opnClockRaw&0x80000000;
    bool dualOPN=opnClockRaw&0x40000000;

    // Ricoh RF5C68 or RF5C164 (Sega CD PCM): Furnace's RF5C68 (the RF5C164
    // if the file has one: only one of them is imported)
    unsigned int rf68ClockRaw=(version>=0x151 && dataStart>=0x44)?vgmRead32(b,0x40):0;
    unsigned int rf164ClockRaw=(version>=0x151 && dataStart>=0x70)?vgmRead32(b,0x6c):0;
    const bool rf164=(rf164ClockRaw&0x3fffffff)!=0;
    const bool useRF=rf164 || (rf68ClockRaw&0x3fffffff)!=0;
    double rfClock=(double)((rf164?rf164ClockRaw:rf68ClockRaw)&0x3fffffff);
    std::vector<VGMRFOp> rfOps;
    std::vector<unsigned char> rfBank, rfBlob;

    // other chips: they are reported but not imported (yet)
    static const struct {
      size_t off;
      unsigned int minVer;
      const char* name;
    } otherChips[]={
      {0x0c,0x100,"SN76489"}, {0x10,0x100,"YM2413"}, {0x2c,0x110,"YM2612"},
      {0x30,0x110,"YM2151"}, {0x38,0x151,"Sega PCM"}, {0x40,0x151,"RF5C68"},
      {0x44,0x151,"YM2203"}, {0x48,0x151,"YM2608"}, {0x4c,0x151,"YM2610"},
      {0x50,0x151,"YM3812"}, {0x54,0x151,"YM3526"}, {0x58,0x151,"Y8950"},
      {0x5c,0x151,"YMF262"}, {0x60,0x151,"YMF278B"}, {0x64,0x151,"YMF271"},
      {0x68,0x151,"YMZ280B"}, {0x6c,0x151,"RF5C164"}, {0x70,0x151,"PWM"},
      {0x74,0x151,"AY-3-8910"}, {0x80,0x161,"Game Boy"}, {0x88,0x161,"MultiPCM"},
      {0x8c,0x161,"uPD7759"}, {0x90,0x161,"OKIM6258"}, {0x98,0x161,"OKIM6295"},
      {0x9c,0x161,"K051649"}, {0xa0,0x161,"K054539"}, {0xa4,0x161,"HuC6280"},
      {0xa8,0x161,"C140"}, {0xac,0x161,"K053260"}, {0xb0,0x161,"Pokey"},
      {0xb4,0x161,"QSound"}, {0xb8,0x171,"SCSP"}, {0xc0,0x171,"WonderSwan"},
      {0xc4,0x171,"VSU"}, {0xc8,0x171,"SAA1099"}, {0xcc,0x171,"ES5503"},
      {0xd0,0x171,"ES5506"}, {0xd8,0x171,"X1-010"}, {0xdc,0x171,"C352"},
      {0xe0,0x171,"GA20"},
    };
    String otherNames;
    for (auto& oc: otherChips) {
      if (version<oc.minVer) continue;
      // the chips being imported are not "other"
      if (oc.off==0x0c && nesClock<=0) continue;
      if (oc.off==0x10) continue;
      if (oc.off==0x2c) continue;
      if (oc.off==0x70) continue;
      if (oc.off==0x6c && rf164) continue;
      if (oc.off==0x40 && useRF && !rf164) continue;
      if (oc.off+4>dataStart && oc.off>=0x40) continue;
      if ((vgmRead32(b,oc.off)&0x3fffffff)!=0) {
        if (!otherNames.empty()) otherNames+=", ";
        otherNames+=oc.name;
      }
    }

    const bool useSN=(nesClock<=0 && snClock>0);
    const bool useOPLL=(opllClock>0);
    const bool useOPN=(nesClock<=0 && opnClock>0);
    // Sega 32X PWM: imported as a Generic PCM DAC
    unsigned int pwmClockRaw=(version>=0x151 && dataStart>=0x74)?vgmRead32(b,0x70):0;
    const bool usePWM=(pwmClockRaw&0x3fffffff)!=0;
    std::vector<unsigned char> pwmBank;
    std::vector<std::pair<size_t,size_t>> pwmBankBlocks;
    std::vector<VGMPWMWrite> pwmW;
    int pwmCycle=0;
    // (the control register: its output modes, LMD/RMD, as the music plays
    // them: the last one written before the first value, else the last one)
    int pwmCtrl=-1, pwmCtrlAtStart=-1;
    int pwmCompressed=0;
    std::vector<VGMDecompTable> decompTables;
    // (channel 3's special mode, register $27 bits 6-7 = 01, with FM 3
    // playing in it at some point)
    bool opnSpecialUsed=false, opnSpecialNow=false;
    if (nesClock<=0 && !useSN && !useOPLL && !useOPN && !usePWM && !useRF) {
      if (otherNames.empty()) {
        lastError="this VGM has no sound chip that can be imported";
      } else {
        lastError=fmt::sprintf("this VGM uses %s. only the NES APU (2A03), the SN76489, the YM2413/VRC7, the YM2612, the 32X PWM and the RF5C68/RF5C164 can be imported for now",otherNames);
      }
      throw VGMInvalidException();
    }
    // YM2612 DAC: the PCM data bank (data blocks of type 0), where each block
    // starts in it, the read position of the 0x8n commands, and the DAC
    // stream commands (0x90-0x95), played back once all are read
    std::vector<unsigned char> opnBank;
    std::vector<std::pair<size_t,size_t>> opnBankBlocks;
    size_t opnBankPos=0;
    std::vector<VGMOPNDACWrite> opnDac;
    struct DACStreamCmd {
      double time;
      int cmd; // 0x90-0x95
      int id;
      unsigned int a, b, c;
    };
    std::vector<DACStreamCmd> dacStreamCmds;
    int opnWrites2=0, opnCompressed=0;
    bool pal=(nesClock>0)?(nesClock<1700000.0):(headerRate==50);
    bool opllRhythm=false;
    int opllWrites2=0;
    bool snStereo=false;
    int snWrites2=0;

    // --- step 1: read the commands ---
    std::vector<VGMEvent> events;
    std::vector<VGMRAMBlock> ramBlocks;
    std::vector<VGMDACWrite> dacWrites;
    double now=0;
    double loopTime=-1;
    int fdsWrites=0, nes2Writes=0, otherWrites=0;
    size_t pos=dataStart;
    bool ended=false;
    while (pos<b.size() && !ended) {
      if (loopPos!=0 && pos==loopPos) loopTime=now;
      unsigned char cmd=b[pos];
      switch (cmd) {
        case 0x61:
          if (pos+3>=b.size()) { ended=true; break; }
          now+=b[pos+1]|(b[pos+2]<<8);
          pos+=3;
          break;
        case 0x62: now+=735; pos++; break;
        case 0x63: now+=882; pos++; break;
        case 0x66: ended=true; break;
        case 0x67: {
          if (pos+7>b.size()) { ended=true; break; }
          unsigned char type=b[pos+2];
          unsigned int size=vgmRead32(b,pos+3)&0x7fffffff;
          size_t dpos=pos+7;
          if (dpos+size>b.size()) { ended=true; break; }
          if (type==0xc2 && size>2) {
            VGMRAMBlock rb;
            rb.addr=b[dpos]|(b[dpos+1]<<8);
            rb.data.assign(b.begin()+dpos+2,b.begin()+dpos+size);
            VGMEvent e;
            e.time=now;
            e.reg=-1;
            e.val=(int)ramBlocks.size();
            ramBlocks.push_back(rb);
            events.push_back(e);
          } else if (type==0x7f) {
            // a decompression table, for the compressed blocks after it
            if (size>=6) {
              VGMDecompTable t;
              t.type=b[dpos];
              t.subType=b[dpos+1];
              t.bitsDec=b[dpos+2];
              t.bitsCmp=b[dpos+3];
              int count=b[dpos+4]|(b[dpos+5]<<8);
              int vb=(t.bitsDec+7)/8;
              for (int k=0; k<count && dpos+6+(size_t)(k+1)*vb<=dpos+size; k++) {
                unsigned int v=0;
                for (int ob=0; ob<vb; ob++) v|=b[dpos+6+k*vb+ob]<<(8*ob);
                t.values.push_back(v);
              }
              // (a new one for the same kind replaces the old one)
              for (size_t k=0; k<decompTables.size(); k++) {
                if (decompTables[k].type==t.type && decompTables[k].subType==t.subType && decompTables[k].bitsDec==t.bitsDec && decompTables[k].bitsCmp==t.bitsCmp) {
                  decompTables.erase(decompTables.begin()+k);
                  break;
                }
              }
              decompTables.push_back(t);
            }
          } else if ((type==0x00 || type==0x40) && useOPN) {
            // the YM2612's PCM data (0x40: compressed)
            std::vector<unsigned char> data;
            bool ok=true;
            if (type==0x40) {
              ok=vgmDecompressBlock(&b[dpos],size,decompTables,data);
            } else {
              data.assign(b.begin()+dpos,b.begin()+dpos+size);
            }
            if (ok) {
              opnBankBlocks.push_back(std::make_pair(opnBank.size(),data.size()));
              opnBank.insert(opnBank.end(),data.begin(),data.end());
            } else {
              opnCompressed++;
            }
          } else if (useRF && type==(rf164?0x02:0x01)) {
            // the RF5C68's PCM data (copied to its memory with 0x68)
            rfBank.insert(rfBank.end(),b.begin()+dpos,b.begin()+dpos+size);
          } else if (useRF && type==(rf164?0xc1:0xc0)) {
            // a block written to the RF5C68's memory (an address, the data)
            if (size>2) {
              VGMRFOp op;
              op.time=now;
              op.kind=2;
              op.a=b[dpos]|(b[dpos+1]<<8);
              op.b=0;
              op.src=rfBlob.size();
              op.len=size-2;
              rfBlob.insert(rfBlob.end(),b.begin()+dpos+2,b.begin()+dpos+size);
              rfOps.push_back(op);
            }
          } else if ((type==0x03 || type==0x43) && usePWM) {
            // the PWM's data: 16-bit words
            std::vector<unsigned char> data;
            bool ok=true;
            if (type==0x43) {
              ok=vgmDecompressBlock(&b[dpos],size,decompTables,data);
            } else {
              data.assign(b.begin()+dpos,b.begin()+dpos+size);
            }
            if (ok) {
              if (data.size()&1) data.push_back(0);
              pwmBankBlocks.push_back(std::make_pair(pwmBank.size(),data.size()));
              pwmBank.insert(pwmBank.end(),data.begin(),data.end());
            } else {
              pwmCompressed++;
            }
          }
          pos=dpos+size;
          break;
        }
        case 0x68: {
          // a copy from a data bank to a chip's memory
          if (pos+12>b.size()) { ended=true; break; }
          unsigned char type=b[pos+2];
          unsigned int src=b[pos+3]|(b[pos+4]<<8)|(b[pos+5]<<16);
          unsigned int dst=b[pos+6]|(b[pos+7]<<8)|(b[pos+8]<<16);
          unsigned int sz=b[pos+9]|(b[pos+10]<<8)|(b[pos+11]<<16);
          if (sz==0) sz=0x1000000;
          if (useRF && type==(rf164?0x02:0x01) && src<rfBank.size()) {
            size_t n=std::min((size_t)sz,rfBank.size()-src);
            VGMRFOp op;
            op.time=now;
            op.kind=2;
            op.a=dst;
            op.b=0;
            op.src=rfBlob.size();
            op.len=n;
            rfBlob.insert(rfBlob.end(),rfBank.begin()+src,rfBank.begin()+src+n);
            rfOps.push_back(op);
          }
          pos+=12;
          break;
        }
        case 0xb4: {
          if (pos+3>b.size()) { ended=true; break; }
          int aa=b[pos+1];
          int dd=b[pos+2];
          pos+=3;
          if (aa&0x80) {
            nes2Writes++;
            break;
          }
          if (aa<=0x1f) {
            if (aa==0x11) {
              VGMDACWrite w;
              w.time=now;
              w.val=dd&0x7f;
              dacWrites.push_back(w);
            } else {
              VGMEvent e;
              e.time=now;
              e.reg=aa;
              e.val=dd;
              events.push_back(e);
            }
          } else {
            fdsWrites++;
          }
          break;
        }
        default:
          if (cmd>=0x70 && cmd<=0x7f) {
            now+=(cmd&15)+1;
            pos++;
          } else if (cmd>=0x80 && cmd<=0x8f) {
            // YM2612 DAC write from the PCM bank, then a wait
            if (useOPN && opnBankPos<opnBank.size()) {
              VGMOPNDACWrite w;
              w.time=now;
              w.val=opnBank[opnBankPos];
              w.src=(int)opnBankPos;
              w.rate=0;
              opnDac.push_back(w);
            }
            opnBankPos++;
            now+=cmd&15;
            pos++;
          } else if (cmd==0x52 || cmd==0x53) {
            if (pos+2<b.size() && useOPN) {
              int port=cmd&1;
              int aa=b[pos+1];
              int dd=b[pos+2];
              if (port==0 && aa==0x2a) {
                VGMOPNDACWrite w;
                w.time=now;
                w.val=dd;
                w.src=-1;
                w.rate=0;
                opnDac.push_back(w);
              } else {
                VGMEvent e;
                e.time=now;
                e.reg=0x600|(port<<8)|aa;
                e.val=dd;
                events.push_back(e);
                // (it counts if FM 3 plays in it: some drivers turn it on
                // and never use it)
                if (port==0 && aa==0x27) opnSpecialNow=(dd&0xc0)==0x40;
                if (port==0 && aa==0x28 && (dd&7)==2 && (dd&0xf0) && opnSpecialNow) opnSpecialUsed=true;
              }
            } else {
              otherWrites++;
            }
            pos+=3;
          } else if ((cmd==0xa2 || cmd==0xa3) && useOPN) {
            // second YM2612
            opnWrites2++;
            pos+=3;
          } else if (cmd==0xb2) {
            // 32X PWM: register (4 bits), value (12 bits)
            if (pos+2<b.size() && usePWM) {
              int reg=b[pos+1]>>4;
              int val=((b[pos+1]&15)<<8)|b[pos+2];
              if (reg==0) {
                pwmCtrl=val;
              } else if (reg==1) {
                pwmCycle=val;
              } else if (reg>=2 && reg<=4) {
                if (pwmCtrlAtStart<0 && pwmW.empty()) pwmCtrlAtStart=pwmCtrl;
                VGMPWMWrite w;
                w.time=now;
                w.ch=reg-2;
                w.val=val;
                w.src=-1;
                w.rate=0;
                pwmW.push_back(w);
              }
            } else {
              otherWrites++;
            }
            pos+=3;
          } else if (cmd==0xe0) {
            // seek in the PCM bank
            if (pos+5<=b.size()) opnBankPos=vgmRead32(b,pos+1);
            pos+=5;
          } else if (cmd>=0x90 && cmd<=0x95) {
            // DAC stream control
            static const int sizes[6]={5,5,6,11,2,5};
            int sz=sizes[cmd-0x90];
            if (pos+sz>b.size()) { ended=true; break; }
            DACStreamCmd sc;
            sc.time=now;
            sc.cmd=cmd;
            sc.id=b[pos+1];
            sc.a=sc.b=sc.c=0;
            switch (cmd) {
              case 0x90: // chip type, port, register
                sc.a=b[pos+2];
                sc.b=(b[pos+3]<<8)|b[pos+4];
                break;
              case 0x91: // data bank, step size, step base
                sc.a=b[pos+2];
                sc.b=b[pos+3];
                sc.c=b[pos+4];
                break;
              case 0x92: // frequency
                sc.a=vgmRead32(b,pos+2);
                break;
              case 0x93: // start offset, length mode, length
                sc.a=vgmRead32(b,pos+2);
                sc.b=b[pos+6];
                sc.c=vgmRead32(b,pos+7);
                break;
              case 0x95: // block, flags
                sc.a=b[pos+2]|(b[pos+3]<<8);
                sc.b=b[pos+4];
                break;
            }
            if (useOPN || usePWM) dacStreamCmds.push_back(sc);
            pos+=sz;
          } else if (cmd>=0x30 && cmd<=0x3f) {
            pos+=2;
          } else if (cmd==0x51) {
            if (pos+2<b.size() && useOPLL) {
              VGMEvent e;
              e.time=now;
              e.reg=0x300|b[pos+1];
              e.val=b[pos+2];
              if (b[pos+1]==0x0e && (e.val&0x20)) opllRhythm=true;
              events.push_back(e);
            }
            pos+=3;
          } else if (cmd==0xa1) {
            opllWrites2++;
            pos+=3;
          } else if (cmd==0x4f || cmd==0x50) {
            if (useSN && pos+1<b.size()) {
              VGMEvent e;
              e.time=now;
              e.reg=(cmd==0x50)?0x200:0x201;
              e.val=b[pos+1];
              if (cmd==0x4f && e.val!=0xff) snStereo=true;
              events.push_back(e);
            } else {
              otherWrites++;
            }
            pos+=2;
          } else if (cmd==0x30 || cmd==0x3f) {
            // second SN76489 / its stereo
            snWrites2++;
            pos+=2;
          } else if (cmd>=0x40 && cmd<=0x5f) {
            pos+=3;
            otherWrites++;
          } else if ((cmd==0xb0 || cmd==0xb1) && useRF && (cmd==0xb1)==rf164) {
            // RF5C68/RF5C164 register
            if (pos+2<b.size()) {
              VGMRFOp op;
              op.time=now;
              op.kind=0;
              op.a=b[pos+1];
              op.b=b[pos+2];
              op.src=0;
              op.len=0;
              rfOps.push_back(op);
            }
            pos+=3;
          } else if ((cmd==0xc0 || cmd==0xc1) && useRF && (cmd==0xc1)==rf164) {
            // RF5C68/RF5C164 memory (in the bank's window)
            if (pos+3<b.size()) {
              VGMRFOp op;
              op.time=now;
              op.kind=1;
              op.a=b[pos+1]|(b[pos+2]<<8);
              op.b=b[pos+3];
              op.src=0;
              op.len=0;
              rfOps.push_back(op);
            }
            pos+=4;
          } else if (cmd>=0xa0 && cmd<=0xbf) {
            pos+=3;
            otherWrites++;
          } else if (cmd>=0xc0 && cmd<=0xdf) {
            pos+=4;
            otherWrites++;
          } else if (cmd>=0xe0) {
            pos+=5;
            otherWrites++;
          } else if (cmd==0x90 || cmd==0x91 || cmd==0x95) {
            pos+=5;
          } else if (cmd==0x92) {
            pos+=6;
          } else if (cmd==0x93) {
            pos+=11;
          } else if (cmd==0x94) {
            pos+=2;
          } else {
            logW("VGM: unknown command %.2x at %x",cmd,(int)pos);
            pos++;
          }
          break;
      }
    }
    double endTime=now;
    if (totalSamples>0 && (double)totalSamples>endTime) endTime=totalSamples;

    // the RF5C68: run it, for what each note plays
    std::vector<VGMSNESSample> rfSamples;
    std::vector<String> rfWarns;
    bool rfStreaming=false;
    if (useRF && !rfOps.empty()) {
      std::vector<VGMEvent> rfEvents;
      vgmRunRF(rfOps,rfBlob,(rfClock>0)?rfClock:12500000.0,endTime,rfEvents,rfSamples,rfWarns,rfStreaming);
      rfOps.clear();
      rfBlob.clear();
      rfBank.clear();
      events.insert(events.end(),rfEvents.begin(),rfEvents.end());
      std::stable_sort(events.begin(),events.end(),[](const VGMEvent& x, const VGMEvent& y) {
        return x.time<y.time;
      });
    }
    const bool rfUsed=useRF && !rfSamples.empty();

    // YM2612 DAC streams: each start plays the PCM bank from its offset at the
    // stream's rate, until its length, the end of the data, a stop or the
    // next start of the same stream. the writes go with the others
    if ((useOPN || usePWM) && !dacStreamCmds.empty() && (!opnBank.empty() || !pwmBank.empty())) {
      // each stream plays on its own: before any command, the streams play
      // up to its time (so a stop, a new start or a new rate takes effect
      // there, and playing goes on from where it was)
      struct StreamState {
        // what it plays to: 0 nothing, 1 the YM2612 DAC, 2-4 the PWM (left,
        // right, both), and from which data (0 the YM2612's, 3 the PWM's)
        int target=0;
        int bank=0;
        bool toDAC=false;
        int step=1, base=0;
        double freq=0;
        // playing: the time of the next write, the next position, writes
        // left (-1: no end), and the loop (start, length in writes)
        bool playing=false;
        double t=0;
        long long pos=0, left=-1;
        bool loop=false;
        long long loopStart=0, loopLen=0, done=0;
        // (the next write is the first of a start)
        bool fresh=false;
      };
      std::map<int,StreamState> st;
      // (positions count bytes: a write takes one byte of the YM2612's data,
      // two of the PWM's. the step and its base count writes)
      auto bankSizeOf=[&](const StreamState& s) -> long long {
        return (s.target>=2)?(long long)pwmBank.size():(long long)opnBank.size();
      };
      auto cmdSizeOf=[&](const StreamState& s) -> long long {
        return (s.target>=2)?2:1;
      };
      auto playTo=[&](StreamState& s, double until) {
        const long long bankSize=bankSizeOf(s);
        if (!s.playing) return;
        if (!s.toDAC || s.freq<=0) {
          s.playing=false;
          return;
        }
        const double dt=44100.0/s.freq;
        while (s.t<until) {
          if (s.left==0) {
            if (s.loop && s.loopLen>0) {
              s.pos=s.loopStart;
              s.left=s.loopLen;
            } else {
              s.playing=false;
              return;
            }
          }
          if (s.pos<0 || s.pos+cmdSizeOf(s)>bankSize) {
            s.playing=false;
            return;
          }
          if (s.target>=2) {
            VGMPWMWrite w;
            w.time=s.t;
            w.ch=s.target-2;
            w.val=(pwmBank[s.pos]|(pwmBank[s.pos+1]<<8))&0xfff;
            w.src=(int)(s.pos/2);
            w.rate=s.freq;
            w.start=s.fresh;
            pwmW.push_back(w);
          } else {
            VGMOPNDACWrite w;
            w.time=s.t;
            w.val=opnBank[s.pos];
            w.src=(int)s.pos;
            w.rate=s.freq;
            w.start=s.fresh;
            opnDac.push_back(w);
          }
          s.fresh=false;
          s.pos+=s.step*cmdSizeOf(s);
          if (s.left>0) s.left--;
          s.t+=dt;
          // (safety: a stream with no end can't go on forever)
          if (++s.done>(1LL<<26)) {
            s.playing=false;
            return;
          }
        }
      };
      for (size_t i=0; i<dacStreamCmds.size(); i++) {
        DACStreamCmd& sc=dacStreamCmds[i];
        // every stream plays up to this command
        for (auto& x: st) playTo(x.second,sc.time);
        if (sc.cmd==0x94 && sc.id==0xff) {
          for (auto& x: st) x.second.playing=false;
          continue;
        }
        StreamState& s=st[sc.id];
        switch (sc.cmd) {
          case 0x90:
            // YM2612 (chip type 2), port 0, register 0x2A; or the PWM
            // (chip type 0x11), register 2-4 (left, right, both)
            s.target=0;
            if ((sc.a&0x7f)==0x02 && sc.b==0x002a && useOPN) s.target=1;
            if ((sc.a&0x7f)==0x11 && (sc.b&0xff)>=2 && (sc.b&0xff)<=4 && usePWM) s.target=2+(int)(sc.b&0xff)-2;
            s.toDAC=(s.target!=0);
            break;
          case 0x91:
            s.bank=sc.a;
            if (s.target==1 && sc.a!=0x00) s.toDAC=false;
            if (s.target>=2 && sc.a!=0x03) s.toDAC=false;
            s.step=std::max(1,(int)sc.b);
            s.base=sc.c;
            break;
          case 0x92:
            // a new rate: the next write comes one new period after the last
            if (s.playing && s.freq>0 && sc.a>0) {
              s.t=sc.time+(s.t-sc.time)*(s.freq/(double)sc.a);
            }
            s.freq=(double)sc.a;
            break;
          case 0x93: {
            long long start=(sc.a==0xffffffff)?s.pos:((long long)sc.a+(long long)s.base*cmdSizeOf(s));
            int mode=sc.b&0x0f;
            long long count=-1;
            switch (mode) {
              case 1: count=sc.c; break;
              case 2: count=(s.freq>0)?(long long)((double)sc.c*s.freq/1000.0):0; break;
              case 3: count=(bankSizeOf(s)-start)/(s.step*cmdSizeOf(s)); break;
              default: count=-1; break;
            }
            // (a start right where the stream already is only goes on: some
            // files restart a long stream every so often)
            const bool goesOn=(s.playing && llabs(start-s.pos)<=2*s.step*cmdSizeOf(s));
            s.playing=true;
            s.fresh=!goesOn;
            s.t=sc.time;
            s.pos=start;
            s.left=count;
            s.loop=sc.b&0x80;
            s.loopStart=start;
            s.loopLen=(count>0)?count:0;
            s.done=0;
            break;
          }
          case 0x94:
            s.playing=false;
            break;
          case 0x95: {
            const std::vector<std::pair<size_t,size_t>>& blocks=(s.target>=2)?pwmBankBlocks:opnBankBlocks;
            if (sc.a>=blocks.size()) {
              s.playing=false;
              break;
            }
            long long start=(long long)blocks[sc.a].first+(long long)s.base*cmdSizeOf(s);
            long long count=(long long)blocks[sc.a].second/(s.step*cmdSizeOf(s));
            s.playing=true;
            s.fresh=true;
            s.t=sc.time;
            s.pos=start;
            s.left=count;
            s.loop=sc.b&1;
            s.loopStart=start;
            s.loopLen=count;
            s.done=0;
            break;
          }
        }
      }
      for (auto& x: st) playTo(x.second,endTime);
      std::stable_sort(opnDac.begin(),opnDac.end(),[](const VGMOPNDACWrite& x, const VGMOPNDACWrite& y) {
        return x.time<y.time;
      });
      std::stable_sort(pwmW.begin(),pwmW.end(),[](const VGMPWMWrite& x, const VGMPWMWrite& y) {
        return x.time<y.time;
      });
    }

    // --- step 2: turn the log into a song ---
    DivSong ds;
    ds.version=DIV_VERSION_VGM;
    // a chord arpeggio (00xy) starts over with every note, like the game's
    ds.compatFlags.resetArpPhaseOnNewNote=true;
    ds.systemLen=0;
    if (nesClock>0) {
      ds.system[0]=DIV_SYSTEM_NES;
      ds.systemFlags[0].set("clockSel",pal?1:0);
      ds.systemLen=1;
    }
    // the YM2612 first, then its PSG (like Furnace's Genesis)
    if (useOPN) {
      int si=ds.systemLen;
      // (Extended Channel 3 only for the songs that use the special mode)
      ds.system[si]=opnSpecialUsed?DIV_SYSTEM_YM2612_EXT:DIV_SYSTEM_YM2612;
      // (the real DAC has no volume: Furnace's would follow FM 6's volume)
      ds.compatFlags.noOPN2Vol=true;
      // (Furnace: 0 YM3438, 1 YM2612)
      ds.systemFlags[si].set("chipType",opnYM3438?0:1);
      static const double clocks[5]={
        (315000000.0/88.0)*15.0/7.0,
        (283.75*15625.0+25.0)*12.0/7.0,
        8000000.0,
        (315000000.0/88.0)*12.0/7.0,
        (315000000.0/88.0)*9.0/4.0
      };
      int bestSel=0;
      double bestD=1e30;
      for (int i=0; i<5; i++) {
        double d=fabs(clocks[i]-opnClock);
        if (d<bestD) {
          bestD=d;
          bestSel=i;
        }
      }
      ds.systemFlags[si].set("clockSel",bestSel);
      opnClock=clocks[bestSel];
      ds.systemLen++;
    }
    const int snSys=ds.systemLen;
    if (useSN) {
      ds.system[snSys]=DIV_SYSTEM_SMS;
      ds.systemLen++;
      // Game Gear when the file uses its stereo, otherwise the Sega PSG
      // (or a TI one, by its noise feedback)
      int chipType=0;
      if (snStereo) {
        chipType=3;
      } else if ((snFeedback==0x0003 || snFeedback==0x0006) && snWidth==15) {
        chipType=1;
      }
      ds.systemFlags[snSys].set("chipType",chipType);
      // the clock closest to the file's
      static const double clocks[6]={315000000.0/88.0,(283.75*15625.0+25.0)*4.0/5.0,4000000.0,(315000000.0/88.0)/2.0,3000000.0,2000000.0};
      static const int sels[6]={0,1,2,3,4,5};
      int bestSel=0;
      double bestD=1e30;
      for (int i=0; i<6; i++) {
        double d=fabs(clocks[i]-snClock);
        if (d<bestD) {
          bestD=d;
          bestSel=sels[i];
        }
      }
      ds.systemFlags[snSys].set("clockSel",bestSel);
      snClock=clocks[bestSel];
    }
    // the YM2413 after the other chip (song commands stay in the first channel)
    const bool opllDrums=opllRhythm && !opllVRC7;
    if (useOPLL) {
      int si=ds.systemLen;
      ds.system[si]=opllVRC7?DIV_SYSTEM_VRC7:(opllDrums?DIV_SYSTEM_OPLL_DRUMS:DIV_SYSTEM_OPLL);
      static const double clocks[4]={315000000.0/88.0,(283.75*15625.0+25.0)*4.0/5.0,4000000.0,(315000000.0/88.0)/2.0};
      int bestSel=0;
      double bestD=1e30;
      for (int i=0; i<4; i++) {
        double d=fabs(clocks[i]-opllClock);
        if (d<bestD) {
          bestD=d;
          bestSel=i;
        }
      }
      ds.systemFlags[si].set("clockSel",bestSel);
      opllClock=clocks[bestSel];
      ds.systemLen++;
    }
    // the 32X's PWM: a Generic PCM DAC (after the other chips), if it plays
    const bool pwmUsed=usePWM && !pwmW.empty();
    // (stereo: when its left and right outputs differ enough, measured on
    // what they hold after the writes of each moment. then two PCM DACs, one
    // on each side)
    bool pwmStereo=false;
    int pwmSys=-1;
    if (pwmUsed) {
      const double mid=(double)(((pwmCycle>2)?pwmCycle:1047)-1)*0.5;
      double curL=mid, curR=mid, side=0, both=0;
      for (size_t i=0; i<pwmW.size(); i++) {
        const VGMPWMWrite& w=pwmW[i];
        if (w.ch==0) {
          curL=w.val;
        } else if (w.ch==1) {
          curR=w.val;
        } else {
          curL=curR=w.val;
        }
        if (i+1<pwmW.size() && pwmW[i+1].time-w.time<0.01) continue;
        const double s=(curL-curR)*0.5;
        const double m=(curL+curR)*0.5-mid;
        side+=s*s;
        both+=m*m;
      }
      // (a mono one has none at all: its sides are written the same)
      pwmStereo=(side>both*0.002);
      logI("VGM import: PWM stereo width %.3f (%s)",(both>0)?(side/both):0.0,pwmStereo?"stereo":"mono");
      // Furnace's 32X PWM, with the file's clock (NTSC or PAL) and cycle
      // register: one channel, or two (left and right) in stereo
      pwmSys=ds.systemLen;
      ds.system[pwmSys]=DIV_SYSTEM_PWM_32X;
      const double pwmClk=(double)(pwmClockRaw&0x3fffffff);
      ds.systemFlags[pwmSys].set("clockSel",(fabs(pwmClk-22801467.0)<fabs(pwmClk-23011361.0))?1:0);
      ds.systemFlags[pwmSys].set("cycle",(pwmCycle>2)?std::max(128,std::min(4095,pwmCycle)):1047);
      {
        // (written as 0 by a file that never sets them: then both normal)
        int ctrl=(pwmCtrlAtStart>=0)?pwmCtrlAtStart:pwmCtrl;
        if (ctrl<0 || (ctrl&15)==0) ctrl=0x05;
        ds.systemFlags[pwmSys].set("lmode",ctrl&3);
        ds.systemFlags[pwmSys].set("rmode",(ctrl>>2)&3);
        if ((ctrl&15)!=0x05) logI("VGM import: PWM output modes LMD %d, RMD %d",ctrl&3,(ctrl>>2)&3);
      }
      ds.systemLen++;
    }
    // the RF5C68/RF5C164 (Sega CD PCM): after the other chips
    if (rfUsed) {
      int si=ds.systemLen;
      ds.system[si]=DIV_SYSTEM_RF5C68;
      ds.systemFlags[si].set("chipType",rf164?1:0);
      // (samples that don't fit in its 64 KB: streamed, like the game does)
      if (rfStreaming) ds.systemFlags[si].set("streaming",true);
      static const double clocks[3]={8000000.0,10000000.0,12500000.0};
      int bestSel=2;
      double bestD=1e30;
      for (int i=0; i<3; i++) {
        double d=fabs(clocks[i]-rfClock);
        if (d<bestD) {
          bestD=d;
          bestSel=i;
        }
      }
      ds.systemFlags[si].set("clockSel",bestSel);
      rfClock=clocks[bestSel];
      ds.systemLen++;
    }
    for (int i=0; i<ds.systemLen; i++) {
      ds.systemVol[i]=1.0f;
      ds.systemPan[i]=0.0f;
    }
    // Mega Drive/Genesis: the balance of the YM2612 and its PSG (the FM a
    // bit up, the PSG down: about 3.9 dB more FM against the PSG than at
    // the same chip volume)
    const float opnMixFM=1.1f;
    const float opnMixPSG=0.7f;
    if (useOPN && useSN) {
      ds.systemVol[0]=opnMixFM;
      ds.systemVol[snSys]=opnMixPSG;
    }
    // the 32X's PWM: its level against the Mega Drive's chips
    const float pwmMix=2.0f;
    if (pwmUsed) ds.systemVol[pwmSys]=pwmMix;
    ds.initDefaultSystemChans();
    // (in stereo its first channel plays the left output and the second one
    // the right: the converter pans them)
    if (pwmUsed) ds.systemChans[pwmSys]=pwmStereo?2:1;
    VGMNESLog log;
    log.hasNES=(nesClock>0);
    log.hasSN=useSN;
    log.hasOPLL=useOPLL;
    log.opllVRC7=opllVRC7;
    log.opllDrums=opllDrums;
    log.opllClock=opllClock;
    log.snStereo=snStereo;
    log.snClock=snClock;
    // the TI chips run the noise at clock/60 in Furnace
    log.snNoiseDivider=(ds.systemFlags[snSys].getInt("chipType",0)==1)?60.0:64.0;
    log.hasOPN=useOPN;
    log.opnExt=useOPN && opnSpecialUsed;
    log.opnClock=opnClock;
    log.opnDac.swap(opnDac);
    log.hasPWM=pwmUsed;
    log.hasRF=rfUsed;
    log.rfClock=rfClock;
    if (rfUsed) log.snesSamples.swap(rfSamples);
    log.pwmStereo=pwmStereo;
    if (pwmCycle>2) {
      log.pwmCycle=pwmCycle;
      // (the PWM outputs a value every cycle-1 clocks)
      const double pwmClk=(double)(pwmClockRaw&0x3fffffff);
      if (pwmClk>0) log.pwmRate=pwmClk/(double)(pwmCycle-1);
    }
    log.pwm.swap(pwmW);
    log.events.swap(events);
    log.ramBlocks.swap(ramBlocks);
    log.dacWrites.swap(dacWrites);
    log.endTime=endTime;
    log.loopTime=loopTime;
    log.clock=nesClock;
    log.pal=pal;
    log.headerRate=headerRate;
    VGMNESContext ctx;
    std::vector<String> warns;
    String err;
    ctx.gridMode=vgmPickGrid(log,ctx,ds);
    if (!vgmConvertNES(log,ctx,ds,ds.subsong[0],warns,err)) {
      lastError=err;
      throw VGMInvalidException();
    }
    for (String& w: warns) addWarning(w);
    for (String& w: rfWarns) addWarning(w);
    int instrumentsOver=ctx.instrumentsOver;

    // --- step 13: GD3 tag ---
    if (gd3Off!=0) {
      size_t gp=0x14+gd3Off;
      if (gp+12<=b.size() && memcmp(&b[gp],"Gd3 ",4)==0) {
        size_t glen=vgmRead32(b,gp+8);
        size_t gend=std::min(b.size(),gp+12+glen);
        size_t sp=gp+12;
        String tags[11];
        for (int i=0; i<11; i++) tags[i]=vgmUTF16ToUTF8(b,sp,gend);
        ds.name=tags[0];
        ds.nameJ=tags[1];
        ds.category=tags[2];
        ds.categoryJ=tags[3];
        ds.author=tags[6];
        ds.authorJ=tags[7];
        String notes="Imported from VGM.";
        if (!tags[2].empty()) notes+="\nGame: "+tags[2];
        if (!tags[4].empty()) notes+="\nSystem: "+tags[4];
        if (!tags[8].empty()) notes+="\nDate: "+tags[8];
        if (!tags[9].empty()) notes+="\nRipped by: "+tags[9];
        if (!tags[10].empty()) notes+="\n\n"+tags[10];
        ds.notes=notes;
      }
    }

    ds.insLen=(int)ds.ins.size();
    ds.sampleLen=(int)ds.sample.size();
    ds.recalcChans();

    if (instrumentsOver>0) addWarning(fmt::sprintf("%d DPCM/PCM hits needed more than 256 instruments and were left out",instrumentsOver));
    if (useSN && (dualSN || snWrites2>0)) addWarning("this VGM has a second SN76489; only the first one was imported");
    if (useSN && t6w28) addWarning("this VGM uses the T6W28 (Neo Geo Pocket); it was imported as a Game Gear PSG");
    if (useOPLL && (dualOPLL || opllWrites2>0)) addWarning("this VGM has a second YM2413; only the first one was imported");
    if (useOPN && (dualOPN || opnWrites2>0)) addWarning("this VGM has a second YM2612; only the first one was imported");
    if (useOPN && opnCompressed>0) addWarning("this VGM has DAC data in a compression that is not supported; those samples are missing");
    if (usePWM && pwmCompressed>0) addWarning("this VGM has PWM data in a compression that is not supported; those samples are missing");
    if (hasFDS || fdsWrites>0) addWarning("this VGM uses the Famicom Disk System sound; FDS import is not supported yet, so that part is missing");
    if (dualNES || nes2Writes>0) addWarning("this VGM has a second NES APU; only the first one was imported");
    if (!otherNames.empty() || otherWrites>0) addWarning(fmt::sprintf("this VGM also uses %s, which was not imported",otherNames.empty()?String("other chips"):otherNames));


    if (active) quitDispatch();
    BUSY_BEGIN_SOFT;
    saveLock.lock();
    song.unload();
    song=ds;
    hasLoadedSomething=true;
    changeSong(0);
    saveLock.unlock();
    BUSY_END;
    if (active) {
      initDispatch();
      BUSY_BEGIN;
      renderSamples();
      reset();
      BUSY_END;
    }
    success=true;
  } catch (VGMInvalidException& e) {
  }

  delete[] file;
  return success;
}

// ---------------------------------------------------------------------------
// NSF import
//
// an NSF is the sound code of a game, not a recording. to know what it plays,
// the code is run on a small 6502 + NSF mapper: INIT once with the track
// number, then PLAY once per frame. every write to the APU is logged with its
// time, and the log goes through the same converter as a VGM.
// the end of the song is found by watching the program's memory: when it
// becomes equal to a state seen before, the program (and so the music) will
// repeat from there on, which is the loop.
// ---------------------------------------------------------------------------

namespace {

struct NSFFile {
  int version=1;
  int songs=1, start=1;
  unsigned short loadAddr=0x8000, initAddr=0x8000, playAddr=0x8000;
  String title, artist, copyright;
  unsigned short ntscSpeed=16639, palSpeed=19997;
  unsigned char banks[8]={0,0,0,0,0,0,0,0};
  bool bankswitched=false;
  bool pal=false, dual=false;
  unsigned char expansion=0;
  std::vector<unsigned char> data;
  std::vector<String> trackNames;
  std::vector<int> trackLengths; // ms, -1 unknown
};

unsigned short nsfRead16(const unsigned char* p) {
  return p[0]|(p[1]<<8);
}

String nsfString(const unsigned char* p, size_t maxLen) {
  String r;
  for (size_t i=0; i<maxLen && p[i]; i++) r+=(char)p[i];
  return r;
}

// NSFe/NSF2 metadata chunks (track names, lengths, author)
void nsfParseChunks(NSFFile& nsf, const unsigned char* p, size_t len, bool isNSFE) {
  size_t pos=0;
  while (pos+8<=len) {
    unsigned int clen=p[pos]|(p[pos+1]<<8)|(p[pos+2]<<16)|((unsigned int)p[pos+3]<<24);
    char id[5];
    memcpy(id,p+pos+4,4);
    id[4]=0;
    pos+=8;
    if (pos+clen>len) break;
    const unsigned char* c=p+pos;
    String sid=id;
    if (sid=="INFO" && isNSFE && clen>=9) {
      nsf.loadAddr=nsfRead16(c);
      nsf.initAddr=nsfRead16(c+2);
      nsf.playAddr=nsfRead16(c+4);
      nsf.pal=(c[6]&3)==1;
      nsf.dual=c[6]&2;
      nsf.expansion=c[7];
      nsf.songs=c[8];
      nsf.start=(clen>=10)?(c[9]+1):1;
    } else if (sid=="DATA" && isNSFE) {
      nsf.data.assign(c,c+clen);
    } else if (sid=="BANK" && isNSFE) {
      for (unsigned int i=0; i<8 && i<clen; i++) nsf.banks[i]=c[i];
      nsf.bankswitched=true;
    } else if (sid=="RATE" && isNSFE && clen>=2) {
      nsf.ntscSpeed=nsfRead16(c);
      if (clen>=4) nsf.palSpeed=nsfRead16(c+2);
    } else if (sid=="auth") {
      String strs[4];
      size_t sp=0;
      for (int i=0; i<4 && sp<clen; i++) {
        while (sp<clen && c[sp]) strs[i]+=(char)c[sp++];
        sp++;
      }
      if (!strs[0].empty()) nsf.title=strs[0];
      if (!strs[1].empty()) nsf.artist=strs[1];
      if (!strs[2].empty()) nsf.copyright=strs[2];
    } else if (sid=="tlbl") {
      nsf.trackNames.clear();
      size_t sp=0;
      while (sp<clen) {
        String n;
        while (sp<clen && c[sp]) n+=(char)c[sp++];
        sp++;
        nsf.trackNames.push_back(n);
      }
    } else if (sid=="time") {
      nsf.trackLengths.clear();
      for (size_t sp=0; sp+4<=clen; sp+=4) {
        nsf.trackLengths.push_back((int)(c[sp]|(c[sp+1]<<8)|(c[sp+2]<<16)|((unsigned int)c[sp+3]<<24)));
      }
    } else if (sid=="NEND") {
      break;
    }
    pos+=clen;
  }
}

bool nsfParse(const unsigned char* f, size_t len, NSFFile& nsf, String& err) {
  if (len>=0x80 && memcmp(f,"NESM\x1a",5)==0) {
    nsf.version=f[5];
    nsf.songs=f[6];
    nsf.start=f[7];
    nsf.loadAddr=nsfRead16(f+0x08);
    nsf.initAddr=nsfRead16(f+0x0a);
    nsf.playAddr=nsfRead16(f+0x0c);
    nsf.title=nsfString(f+0x0e,32);
    nsf.artist=nsfString(f+0x2e,32);
    nsf.copyright=nsfString(f+0x4e,32);
    nsf.ntscSpeed=nsfRead16(f+0x6e);
    for (int i=0; i<8; i++) {
      nsf.banks[i]=f[0x70+i];
      if (f[0x70+i]) nsf.bankswitched=true;
    }
    nsf.palSpeed=nsfRead16(f+0x78);
    nsf.pal=(f[0x7a]&3)==1;
    nsf.dual=f[0x7a]&2;
    nsf.expansion=f[0x7b];
    size_t progLen=f[0x7d]|(f[0x7e]<<8)|(f[0x7f]<<16);
    if (nsf.version>=2 && progLen>0 && 0x80+progLen<len) {
      nsf.data.assign(f+0x80,f+0x80+progLen);
      nsfParseChunks(nsf,f+0x80+progLen,len-0x80-progLen,false);
    } else {
      nsf.data.assign(f+0x80,f+len);
    }
  } else if (len>=4 && memcmp(f,"NSFE",4)==0) {
    nsfParseChunks(nsf,f+4,len-4,true);
    if (nsf.data.empty()) {
      err="this NSFe file has no program data";
      return false;
    }
  } else {
    err="not an NSF file";
    return false;
  }
  if (nsf.songs<1) nsf.songs=1;
  if (nsf.start<1 || nsf.start>nsf.songs) nsf.start=1;
  if (nsf.ntscSpeed==0) nsf.ntscSpeed=16639;
  if (nsf.palSpeed==0) nsf.palSpeed=19997;
  return true;
}

// the NSF machine: 6502, 2 KB RAM, $6000-$FFFF (banked ROM, or RAM for FDS)
struct NSFMachine {
  const NSFFile* nsf;
  std::vector<unsigned char> prg; // program data, padded to whole 4 KB banks
  unsigned char ram[0x800];
  unsigned char hi[0xa000]; // $6000-$FFFF
  unsigned char exram[0x400];
  unsigned char bankReg[10];
  unsigned char mulA=0, mulB=0;
  bool fds=false;

  // CPU
  unsigned char A=0, X=0, Y=0, S=0xfd, P=0x24;
  unsigned short PC=0;
  long long cycles=0;

  // logging
  VGMNESLog* log=NULL;
  double callTime=0; // VGM sample time of the start of the current call
  long long callCycles=0;
  double clock=1789773.0;
  unsigned char reg4012=0, reg4013=0;
  unsigned char latch5B=0;
  int expWrites=0;
  std::vector<std::pair<int,int>> frameWrites; // writes of the current frame
  // where a call returns to (an address the program never runs)
  unsigned short retAddr=0x3ff0;

  // C64 mode (SID files): 64 KB of RAM, the I/O area and a few KERNAL
  // routines, enough for music players
  bool c64=false;
  unsigned char* mem=NULL;
  unsigned char kernal[0x2000]; // $E000-$FFFF when the KERNAL is banked in
  unsigned char sidRegs[32];
  unsigned char vicIrqMask=0;
  unsigned short ciaLatch=0x4025;
  bool ciaLatchSet=false;
  unsigned char ciaMask=0x01; // CIA 1 interrupts enabled (timer A)
  unsigned int lfsr=0x7ffff8;
  int linesPerFrame=312;
  int volWrites=0; // writes to $D418 in this frame (many: a digi)
  bool pageDirty[256];

  bool c64IO() {
    return (mem[1]&3)!=0 && (mem[1]&4);
  }
  bool c64Kernal() {
    return (mem[1]&2);
  }
  int rasterLine() {
    return (int)((cycles/63)%linesPerFrame);
  }

  unsigned char c64Read(unsigned short a) {
    if (a>=0xd000 && a<0xe000 && c64IO()) {
      if (a>=0xd400 && a<0xd800) {
        int r=a&0x1f;
        // oscillator 3 output: players use it as a random number
        if (r==0x1b) {
          lfsr=((lfsr<<1)|(((lfsr>>22)^(lfsr>>17))&1))&0x7fffff;
          return (unsigned char)(lfsr>>15);
        }
        return 0;
      }
      if (a>=0xd000 && a<0xd400) {
        int r=a&0x3f;
        if (r==0x12) return (unsigned char)rasterLine();
        if (r==0x11) return (unsigned char)(0x1b|((rasterLine()&0x100)>>1));
        if (r==0x19) return 0x81;
        if (r==0x1a) return vicIrqMask;
        return 0;
      }
      if (a>=0xdc00 && a<0xde00) {
        int r=a&15;
        long long per=(long long)ciaLatch+1;
        int cnt=(int)(per-(cycles%per));
        if (r==4) return (unsigned char)(cnt&0xff);
        if (r==5) return (unsigned char)(cnt>>8);
        if (r==0x0d) return 0x81;
        if (r==0) return 0xff;
        if (r==1) return 0xff;
        return 0;
      }
      return 0;
    }
    if (a>=0xe000 && c64Kernal()) return kernal[a-0xe000];
    return mem[a];
  }

  void c64Write(unsigned short a, unsigned char v) {
    if (a>=0xd000 && a<0xe000 && c64IO()) {
      if (a>=0xd400 && a<0xd800) {
        int r=a&0x1f;
        if (r>0x18) return;
        sidRegs[r]=v;
        if (r==0x18) volWrites++;
        frameWrites.push_back(std::make_pair(0x400|r,v));
        VGMEvent e;
        e.time=timeNow();
        e.reg=0x400|r;
        e.val=v;
        log->events.push_back(e);
        return;
      }
      if (a>=0xd000 && a<0xd400) {
        if ((a&0x3f)==0x1a) vicIrqMask=v;
        return;
      }
      if (a>=0xdc00 && a<0xdd00) {
        int r=a&15;
        if (r==4) {
          ciaLatch=(ciaLatch&0xff00)|v;
          ciaLatchSet=true;
        }
        if (r==5) {
          ciaLatch=(ciaLatch&0xff)|(v<<8);
          ciaLatchSet=true;
        }
        if (r==0x0d) {
          if (v&0x80) {
            ciaMask|=v&0x1f;
          } else {
            ciaMask&=~(v&0x1f);
          }
        }
        return;
      }
      return;
    }
    pageDirty[a>>8]=true;
    mem[a]=v;
  }

  // the KERNAL bits music players rely on: the IRQ entry (saves the
  // registers and jumps through $0314) and its exits
  void c64Setup() {
    memset(kernal,0,sizeof(kernal));
    memset(sidRegs,0,sizeof(sidRegs));
    memset(pageDirty,0,sizeof(pageDirty));
    static const unsigned char irqEntry[]={0x48,0x8a,0x48,0x98,0x48,0x6c,0x14,0x03};
    memcpy(&kernal[0xff48-0xe000],irqEntry,sizeof(irqEntry));
    // $EA31 (the usual end of an IRQ handler) and $EA7E: acknowledge, then
    // $EA81: restore the registers and return
    static const unsigned char irqExit[]={0xad,0x0d,0xdc,0x68,0xa8,0x68,0xaa,0x68,0x40};
    memcpy(&kernal[0x0a7e],irqExit,sizeof(irqExit));
    kernal[0xea31-0xe000]=0x4c;
    kernal[0xea32-0xe000]=0x7e;
    kernal[0xea33-0xe000]=0xea;
    memcpy(&kernal[0xfebc-0xe000],&irqExit[3],6);
    // NMI: just return
    kernal[0x1e43]=0x40;
    kernal[0xfffa-0xe000]=0x43;
    kernal[0xfffb-0xe000]=0xfe;
    kernal[0x1ffe]=0x48;
    kernal[0xffff-0xe000]=0xff;
    mem[0x314]=0x31;
    mem[0x315]=0xea;
    mem[0x318]=0x43;
    mem[0x319]=0xfe;
  }

  // runs the interrupt handler, as the CPU would (registers pushed by the
  // handler itself, then RTI)
  bool irq(long long maxCycles) {
    push(retAddr>>8);
    push(retAddr&0xff);
    push(P&0xef);
    P|=0x04;
    PC=read(0xfffe)|(read(0xffff)<<8);
    long long limit=cycles+maxCycles;
    while (PC!=retAddr) {
      if (cycles>=limit) {
        S=0xfd;
        return false;
      }
      step();
    }
    return true;
  }

  void mapBank(int slot, int bank) {
    // slot 0-7: $8000-$FFFF, 8-9: $6000/$7000 (FDS)
    size_t src=(size_t)bank*4096;
    unsigned char* dst=(slot>=8)?&hi[(slot-8)*4096]:&hi[0x2000+slot*4096];
    if (src+4096<=prg.size()) {
      memcpy(dst,&prg[src],4096);
    } else {
      memset(dst,0,4096);
      if (src<prg.size()) memcpy(dst,&prg[src],prg.size()-src);
    }
  }

  void setup() {
    memset(ram,0,sizeof(ram));
    memset(hi,0,sizeof(hi));
    memset(exram,0,sizeof(exram));
    fds=nsf->expansion&4;
    if (nsf->bankswitched) {
      // the data starts at (load address & $FFF) inside the first bank
      size_t pad=nsf->loadAddr&0xfff;
      prg.assign(pad,0);
      prg.insert(prg.end(),nsf->data.begin(),nsf->data.end());
      while (prg.size()&0xfff) prg.push_back(0);
      for (int i=0; i<8; i++) {
        bankReg[i]=nsf->banks[i];
        mapBank(i,nsf->banks[i]);
      }
      if (fds) {
        bankReg[8]=nsf->banks[6];
        bankReg[9]=nsf->banks[7];
        mapBank(8,nsf->banks[6]);
        mapBank(9,nsf->banks[7]);
      }
    } else {
      prg=nsf->data;
      size_t at=(nsf->loadAddr>=0x6000)?(nsf->loadAddr-0x6000):0;
      size_t n=std::min(prg.size(),sizeof(hi)-at);
      memcpy(&hi[at],prg.data(),n);
    }
  }

  double timeNow() {
    return callTime+(double)(cycles-callCycles)*44100.0/clock;
  }

  unsigned char read(unsigned short a) {
    if (c64) return c64Read(a);
    if (a<0x2000) return ram[a&0x7ff];
    if (a>=0x6000) return hi[a-0x6000];
    if (a==0x5205) return (unsigned char)((mulA*mulB)&0xff);
    if (a==0x5206) return (unsigned char)((mulA*mulB)>>8);
    if (a>=0x5c00 && a<0x5ff6) return exram[(a-0x5c00)&0x3ff];
    return 0;
  }

  void write(unsigned short a, unsigned char v) {
    if (c64) {
      c64Write(a,v);
      return;
    }
    if (a<0x2000) {
      ram[a&0x7ff]=v;
      return;
    }
    if (a>=0x4000 && a<=0x4017) {
      int reg=a-0x4000;
      double t=timeNow();
      frameWrites.push_back(std::make_pair(reg,v));
      if (reg==0x11) {
        VGMDACWrite w;
        w.time=t;
        w.val=v&0x7f;
        log->dacWrites.push_back(w);
        return;
      }
      if (reg==0x12) reg4012=v;
      if (reg==0x13) reg4013=v;
      if (reg==0x15 && (v&0x10)) {
        // the DMC reads the sample from the current memory map: copy it now
        VGMRAMBlock rb;
        rb.addr=0xc000+(reg4012<<6);
        int l=(reg4013<<4)+1;
        rb.data.resize(l);
        for (int i=0; i<l; i++) {
          int ad=rb.addr+i;
          if (ad>0xffff) ad=0x8000+(ad-0x10000);
          rb.data[i]=read((unsigned short)ad);
        }
        VGMEvent be;
        be.time=t;
        be.reg=-1;
        be.val=(int)log->ramBlocks.size();
        log->ramBlocks.push_back(rb);
        log->events.push_back(be);
      }
      VGMEvent e;
      e.time=t;
      e.reg=reg;
      e.val=v;
      log->events.push_back(e);
      return;
    }
    if (a>=0x5ff6 && a<=0x5fff) {
      if (a>=0x5ff8) {
        if (nsf->bankswitched) {
          bankReg[a-0x5ff8]=v;
          mapBank(a-0x5ff8,v);
        }
      } else if (fds && nsf->bankswitched) {
        bankReg[8+(a-0x5ff6)]=v;
        mapBank(8+(a-0x5ff6),v);
      }
      return;
    }
    if (a==0x5205) {
      mulA=v;
      return;
    }
    if (a==0x5206) {
      mulB=v;
      return;
    }
    if (a>=0x5c00 && a<0x5ff6) {
      exram[(a-0x5c00)&0x3ff]=v;
      return;
    }
    if (a>=0x6000 && a<0x8000) {
      hi[a-0x6000]=v;
      return;
    }
    if (a>=0x8000 && fds) {
      if (a<0xe000) hi[a-0x6000]=v;
      return;
    }
    // Konami VRC6: $9000-$9003 (pulse 1 and the frequency control), $A000-$A002
    // (pulse 2), $B000-$B002 (saw)
    if ((nsf->expansion&1) && a>=0x9000 && a<0xc000) {
      int ch=(a>>12)-9;
      int r=a&0xfff;
      if (r<=2 || (ch==0 && r==3)) {
        int reg=0x500|(ch<<2)|r;
        frameWrites.push_back(std::make_pair(reg,v));
        VGMEvent e;
        e.time=timeNow();
        e.reg=reg;
        e.val=v;
        log->events.push_back(e);
      }
      return;
    }
    // Sunsoft 5B: $C000 selects a register, $E000 writes it
    if ((nsf->expansion&0x20) && a>=0xc000) {
      if (a<0xe000) {
        latch5B=v&15;
      } else {
        frameWrites.push_back(std::make_pair(0x100|latch5B,v));
        VGMEvent e;
        e.time=timeNow();
        e.reg=0x100|latch5B;
        e.val=v;
        log->events.push_back(e);
      }
      return;
    }
    // other expansion sound registers (VRC6, VRC7, FDS, MMC5, N163)
    if ((a>=0x4040 && a<0x40a0) || (a>=0x4800 && a<0x4900) || (a>=0x5000 && a<0x5016) || a>=0x8000) {
      expWrites++;
    }
  }

  // --- 6502 ---
  void push(unsigned char v) {
    if (c64) {
      mem[0x100|S]=v;
    } else {
      ram[0x100|S]=v;
    }
    S--;
  }
  unsigned char pull() {
    S++;
    return c64?mem[0x100|S]:ram[0x100|S];
  }
  void setNZ(unsigned char v) {
    P=(P&0x7d)|(v&0x80)|((v==0)?2:0);
  }
  unsigned short read16(unsigned short a) {
    return read(a)|(read((unsigned short)(a+1))<<8);
  }
  unsigned short read16zp(unsigned char a) {
    return read(a)|(read((unsigned char)(a+1))<<8);
  }

  void adc(unsigned char v) {
    int sum=A+v+(P&1);
    P&=~0x41;
    if (sum>0xff) P|=1;
    if (~(A^v)&(A^sum)&0x80) P|=0x40;
    A=(unsigned char)sum;
    setNZ(A);
  }
  void cmp(unsigned char r, unsigned char v) {
    int d=r-v;
    P=(P&~1)|((r>=v)?1:0);
    setNZ((unsigned char)d);
  }

  // executes one instruction
  void step() {
    unsigned char op=read(PC++);
    unsigned short addr=0;
    bool pageCross=false;
    int cyc=2;

    // addressing mode by opcode
    static const unsigned char modeTab[256]={
    // 0=imp 1=imm 2=zp 3=zpx 4=zpy 5=abs 6=abx 7=aby 8=ind 9=izx 10=izy 11=rel 12=acc
       0, 9, 0, 9, 2, 2, 2, 2, 0, 1,12, 1, 5, 5, 5, 5,
      11,10, 0,10, 3, 3, 3, 3, 0, 7, 0, 7, 6, 6, 6, 6,
       5, 9, 0, 9, 2, 2, 2, 2, 0, 1,12, 1, 5, 5, 5, 5,
      11,10, 0,10, 3, 3, 3, 3, 0, 7, 0, 7, 6, 6, 6, 6,
       0, 9, 0, 9, 2, 2, 2, 2, 0, 1,12, 1, 5, 5, 5, 5,
      11,10, 0,10, 3, 3, 3, 3, 0, 7, 0, 7, 6, 6, 6, 6,
       0, 9, 0, 9, 2, 2, 2, 2, 0, 1,12, 1, 8, 5, 5, 5,
      11,10, 0,10, 3, 3, 3, 3, 0, 7, 0, 7, 6, 6, 6, 6,
       1, 9, 1, 9, 2, 2, 2, 2, 0, 1, 0, 1, 5, 5, 5, 5,
      11,10, 0,10, 3, 3, 4, 4, 0, 7, 0, 7, 6, 6, 7, 7,
       1, 9, 1, 9, 2, 2, 2, 2, 0, 1, 0, 1, 5, 5, 5, 5,
      11,10, 0,10, 3, 3, 4, 4, 0, 7, 0, 7, 6, 6, 7, 7,
       1, 9, 1, 9, 2, 2, 2, 2, 0, 1, 0, 1, 5, 5, 5, 5,
      11,10, 0,10, 3, 3, 3, 3, 0, 7, 0, 7, 6, 6, 6, 6,
       1, 9, 1, 9, 2, 2, 2, 2, 0, 1, 0, 1, 5, 5, 5, 5,
      11,10, 0,10, 3, 3, 3, 3, 0, 7, 0, 7, 6, 6, 6, 6,
    };
    static const unsigned char cycTab[256]={
      7,6,2,8,3,3,5,5,3,2,2,2,4,4,6,6, 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
      6,6,2,8,3,3,5,5,4,2,2,2,4,4,6,6, 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
      6,6,2,8,3,3,5,5,3,2,2,2,3,4,6,6, 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
      6,6,2,8,3,3,5,5,4,2,2,2,5,4,6,6, 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
      2,6,2,6,3,3,3,3,2,2,2,2,4,4,4,4, 2,6,2,6,4,4,4,4,2,5,2,5,5,5,5,5,
      2,6,2,6,3,3,3,3,2,2,2,2,4,4,4,4, 2,5,2,5,4,4,4,4,2,4,2,4,4,4,4,4,
      2,6,2,8,3,3,5,5,2,2,2,2,4,4,6,6, 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
      2,6,2,8,3,3,5,5,2,2,2,2,4,4,6,6, 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
    };
    unsigned char mode=modeTab[op];
    cyc=cycTab[op];
    switch (mode) {
      case 1: addr=PC++; break;
      case 2: addr=read(PC++); break;
      case 3: addr=(unsigned char)(read(PC++)+X); break;
      case 4: addr=(unsigned char)(read(PC++)+Y); break;
      case 5: addr=read16(PC); PC+=2; break;
      case 6: {
        unsigned short b=read16(PC);
        PC+=2;
        addr=b+X;
        pageCross=((b^addr)&0xff00);
        break;
      }
      case 7: {
        unsigned short b=read16(PC);
        PC+=2;
        addr=b+Y;
        pageCross=((b^addr)&0xff00);
        break;
      }
      case 8: {
        unsigned short p=read16(PC);
        PC+=2;
        // the 6502 indirect JMP bug: the high byte doesn't cross pages
        addr=read(p)|(read((p&0xff00)|((p+1)&0xff))<<8);
        break;
      }
      case 9: addr=read16zp((unsigned char)(read(PC++)+X)); break;
      case 10: {
        unsigned short b=read16zp(read(PC++));
        addr=b+Y;
        pageCross=((b^addr)&0xff00);
        break;
      }
      case 11: addr=PC+1; break;
      default: break;
    }

    // reads with a page cross take one more cycle
    auto rd=[&]() -> unsigned char {
      if (pageCross) cyc++;
      return read(addr);
    };
    auto branch=[&](bool cond) {
      signed char off=(signed char)read(PC++);
      if (cond) {
        unsigned short n=PC+off;
        cyc+=((n^PC)&0xff00)?2:1;
        PC=n;
      }
    };
    // read-modify-write helpers (accumulator or memory)
    auto rmwRead=[&]() -> unsigned char {
      return (mode==12)?A:read(addr);
    };
    auto rmwWrite=[&](unsigned char v) {
      if (mode==12) {
        A=v;
      } else {
        write(addr,v);
      }
    };
    auto asl=[&](unsigned char v) -> unsigned char {
      P=(P&~1)|(v>>7);
      v<<=1;
      setNZ(v);
      return v;
    };
    auto lsr=[&](unsigned char v) -> unsigned char {
      P=(P&~1)|(v&1);
      v>>=1;
      setNZ(v);
      return v;
    };
    auto rol=[&](unsigned char v) -> unsigned char {
      unsigned char c=P&1;
      P=(P&~1)|(v>>7);
      v=(v<<1)|c;
      setNZ(v);
      return v;
    };
    auto ror=[&](unsigned char v) -> unsigned char {
      unsigned char c=P&1;
      P=(P&~1)|(v&1);
      v=(v>>1)|(c<<7);
      setNZ(v);
      return v;
    };

    switch (op) {
      // loads/stores
      case 0xa9: case 0xa5: case 0xb5: case 0xad: case 0xbd: case 0xb9: case 0xa1: case 0xb1:
        A=rd(); setNZ(A); break;
      case 0xa2: case 0xa6: case 0xb6: case 0xae: case 0xbe:
        X=rd(); setNZ(X); break;
      case 0xa0: case 0xa4: case 0xb4: case 0xac: case 0xbc:
        Y=rd(); setNZ(Y); break;
      case 0x85: case 0x95: case 0x8d: case 0x9d: case 0x99: case 0x81: case 0x91:
        write(addr,A); break;
      case 0x86: case 0x96: case 0x8e:
        write(addr,X); break;
      case 0x84: case 0x94: case 0x8c:
        write(addr,Y); break;
      // transfers
      case 0xaa: X=A; setNZ(X); break;
      case 0xa8: Y=A; setNZ(Y); break;
      case 0xba: X=S; setNZ(X); break;
      case 0x8a: A=X; setNZ(A); break;
      case 0x9a: S=X; break;
      case 0x98: A=Y; setNZ(A); break;
      // stack
      case 0x48: push(A); break;
      case 0x08: push(P|0x30); break;
      case 0x68: A=pull(); setNZ(A); break;
      case 0x28: P=(pull()&0xef)|0x20; break;
      // logic and arithmetic
      case 0x29: case 0x25: case 0x35: case 0x2d: case 0x3d: case 0x39: case 0x21: case 0x31:
        A&=rd(); setNZ(A); break;
      case 0x09: case 0x05: case 0x15: case 0x0d: case 0x1d: case 0x19: case 0x01: case 0x11:
        A|=rd(); setNZ(A); break;
      case 0x49: case 0x45: case 0x55: case 0x4d: case 0x5d: case 0x59: case 0x41: case 0x51:
        A^=rd(); setNZ(A); break;
      case 0x69: case 0x65: case 0x75: case 0x6d: case 0x7d: case 0x79: case 0x61: case 0x71:
        adc(rd()); break;
      case 0xe9: case 0xeb: case 0xe5: case 0xf5: case 0xed: case 0xfd: case 0xf9: case 0xe1: case 0xf1:
        adc(rd()^0xff); break;
      case 0xc9: case 0xc5: case 0xd5: case 0xcd: case 0xdd: case 0xd9: case 0xc1: case 0xd1:
        cmp(A,rd()); break;
      case 0xe0: case 0xe4: case 0xec:
        cmp(X,rd()); break;
      case 0xc0: case 0xc4: case 0xcc:
        cmp(Y,rd()); break;
      case 0x24: case 0x2c: {
        unsigned char v=read(addr);
        P=(P&0x3d)|(v&0xc0)|(((A&v)==0)?2:0);
        break;
      }
      // increments
      case 0xe6: case 0xf6: case 0xee: case 0xfe: {
        unsigned char v=read(addr)+1; write(addr,v); setNZ(v); break;
      }
      case 0xc6: case 0xd6: case 0xce: case 0xde: {
        unsigned char v=read(addr)-1; write(addr,v); setNZ(v); break;
      }
      case 0xe8: X++; setNZ(X); break;
      case 0xc8: Y++; setNZ(Y); break;
      case 0xca: X--; setNZ(X); break;
      case 0x88: Y--; setNZ(Y); break;
      // shifts
      case 0x0a: case 0x06: case 0x16: case 0x0e: case 0x1e: rmwWrite(asl(rmwRead())); break;
      case 0x4a: case 0x46: case 0x56: case 0x4e: case 0x5e: rmwWrite(lsr(rmwRead())); break;
      case 0x2a: case 0x26: case 0x36: case 0x2e: case 0x3e: rmwWrite(rol(rmwRead())); break;
      case 0x6a: case 0x66: case 0x76: case 0x6e: case 0x7e: rmwWrite(ror(rmwRead())); break;
      // jumps
      case 0x4c: case 0x6c: PC=addr; break;
      case 0x20: {
        unsigned short ret=PC-1;
        push(ret>>8);
        push(ret&0xff);
        PC=addr;
        break;
      }
      case 0x60: {
        unsigned short lo=pull();
        unsigned short hiB=pull();
        PC=((hiB<<8)|lo)+1;
        break;
      }
      case 0x40: {
        P=(pull()&0xef)|0x20;
        unsigned short lo=pull();
        unsigned short hiB=pull();
        PC=(hiB<<8)|lo;
        break;
      }
      case 0x00: {
        // BRK: nothing to interrupt into in an NSF; treat as the end of the call
        PC=retAddr;
        break;
      }
      // branches
      case 0x10: branch(!(P&0x80)); break;
      case 0x30: branch(P&0x80); break;
      case 0x50: branch(!(P&0x40)); break;
      case 0x70: branch(P&0x40); break;
      case 0x90: branch(!(P&1)); break;
      case 0xb0: branch(P&1); break;
      case 0xd0: branch(!(P&2)); break;
      case 0xf0: branch(P&2); break;
      // flags
      case 0x18: P&=~1; break;
      case 0x38: P|=1; break;
      case 0x58: P&=~4; break;
      case 0x78: P|=4; break;
      case 0xb8: P&=~0x40; break;
      case 0xd8: P&=~8; break;
      case 0xf8: P|=8; break;
      // undocumented opcodes some drivers use
      case 0xa7: case 0xb7: case 0xaf: case 0xbf: case 0xa3: case 0xb3: // LAX
        A=X=rd(); setNZ(A); break;
      case 0x87: case 0x97: case 0x8f: case 0x83: // SAX
        write(addr,A&X); break;
      case 0xc7: case 0xd7: case 0xcf: case 0xdf: case 0xdb: case 0xc3: case 0xd3: { // DCP
        unsigned char v=read(addr)-1; write(addr,v); cmp(A,v); break;
      }
      case 0xe7: case 0xf7: case 0xef: case 0xff: case 0xfb: case 0xe3: case 0xf3: { // ISC
        unsigned char v=read(addr)+1; write(addr,v); adc(v^0xff); break;
      }
      case 0x07: case 0x17: case 0x0f: case 0x1f: case 0x1b: case 0x03: case 0x13: { // SLO
        unsigned char v=asl(read(addr)); write(addr,v); A|=v; setNZ(A); break;
      }
      case 0x27: case 0x37: case 0x2f: case 0x3f: case 0x3b: case 0x23: case 0x33: { // RLA
        unsigned char v=rol(read(addr)); write(addr,v); A&=v; setNZ(A); break;
      }
      case 0x47: case 0x57: case 0x4f: case 0x5f: case 0x5b: case 0x43: case 0x53: { // SRE
        unsigned char v=lsr(read(addr)); write(addr,v); A^=v; setNZ(A); break;
      }
      case 0x67: case 0x77: case 0x6f: case 0x7f: case 0x7b: case 0x63: case 0x73: { // RRA
        unsigned char v=ror(read(addr)); write(addr,v); adc(v); break;
      }
      case 0x0b: case 0x2b: // ANC
        A&=read(addr); setNZ(A); P=(P&~1)|(A>>7); break;
      case 0x4b: // ALR
        A&=read(addr); A=lsr(A); break;
      case 0x6b: // ARR
        A&=read(addr); A=ror(A); break;
      case 0xcb: { // AXS
        unsigned char v=read(addr);
        int r=(A&X)-v;
        P=(P&~1)|((r>=0)?1:0);
        X=(unsigned char)r;
        setNZ(X);
        break;
      }
      default:
        // NOPs (official and not), reading their operand if any
        if (mode!=0 && mode!=12 && mode!=11) {
          if (pageCross) cyc++;
        }
        break;
    }
    // stores and RMW don't get the page cross cycle (rd() adds it only for reads)
    cycles+=cyc;
  }

  // calls a routine like JSR and runs it until it returns.
  // returns false if it doesn't return in time.
  bool call(unsigned short addr, long long maxCycles) {
    // return address: $3FF0, an address nothing else uses
    unsigned short ret=retAddr-1;
    push(ret>>8);
    push(ret&0xff);
    PC=addr;
    long long limit=cycles+maxCycles;
    while (PC!=retAddr) {
      if (cycles>=limit) {
        // fix the stack so the next call starts clean
        S=0xfd;
        return false;
      }
      step();
    }
    return true;
  }
};

unsigned long long nsfHash(const unsigned char* p, size_t len, unsigned long long h=1469598103934665603ULL) {
  for (size_t i=0; i<len; i++) {
    h^=p[i];
    h*=1099511628211ULL;
  }
  return h;
}

// runs one NSF track and fills the log
bool nsfRunTrack(const NSFFile& nsf, int track, int maxSeconds, VGMNESLog& log, std::vector<String>& warns, String& err, int& expWrites) {
  NSFMachine* m=new NSFMachine;
  m->nsf=&nsf;
  m->log=&log;
  bool pal=nsf.pal;
  m->clock=pal?1662607.0:1789773.0;
  double speed=pal?nsf.palSpeed:nsf.ntscSpeed;
  double period=speed*44100.0/1000000.0;
  log.clock=m->clock;
  log.pal=pal;
  log.fixedPeriod=period;
  log.fixedT0=0;
  m->setup();

  // the usual APU init
  m->callTime=0;
  m->callCycles=m->cycles;
  for (int i=0; i<0x14; i++) m->write(0x4000+i,(i==0x10)?0x10:0);
  m->write(0x4015,0x00);
  m->write(0x4015,0x0f);
  m->write(0x4017,0x40);

  m->A=(unsigned char)(track-1);
  m->X=pal?1:0;
  m->Y=0;
  m->S=0xfd;
  if (!m->call(nsf.initAddr,(long long)(m->clock*2))) {
    warns.push_back("the INIT routine of this NSF did not return; the result may be wrong");
  }

  int maxFrames=(int)((double)maxSeconds*1000000.0/speed);
  if (maxFrames<60) maxFrames=60;
  std::map<unsigned long long,int> stateSeen;
  std::vector<unsigned long long> stateHash, writeHash, soundHash;
  // per channel, what the program has set it to (register image) each frame
  // (5 for the 2A03, then 3 for the 5B and 3 for the VRC6)
  const int g5B=5;
  const int gVRC6=(nsf.expansion&0x20)?8:5;
  const int nGroups=gVRC6+((nsf.expansion&1)?3:0);
  std::vector<std::vector<unsigned long long>> chanHash(nGroups);
  unsigned char apuImg[0x18];
  memset(apuImg,0,sizeof(apuImg));
  std::vector<bool> audible;
  VGMNESApu* apu=new VGMNESApu;
  apu->pal=pal;
  apu->clock=m->clock;
  size_t apuEv=0;
  unsigned char ayRegs[16];
  memset(ayRegs,0,16);
  ayRegs[7]=0x3f;
  unsigned char vrcRegs[12];
  memset(vrcRegs,0,12);
  int loopStart=-1, loopEnd=-1;
  int stuckAt=-1;
  int playFails=0;
  for (int f=0; f<maxFrames; f++) {
    m->callTime=(double)f*period;
    m->callCycles=m->cycles;
    m->frameWrites.clear();
    if (!m->call(nsf.playAddr,(long long)(m->clock/30.0))) playFails++;

    // what can be heard at the end of this frame
    double snap=((double)f+0.5)*period;
    while (apuEv<log.events.size() && log.events[apuEv].time<=snap) {
      VGMEvent& e=log.events[apuEv++];
      if (e.reg>=0x500) {
        vrcRegs[e.reg&15]=e.val;
        continue;
      }
      if (e.reg>=0x100) {
        ayRegs[e.reg&15]=e.val;
        continue;
      }
      apu->advance(e.time*m->clock/44100.0);
      if (e.reg>=0) apu->write(e.reg,e.val);
    }
    apu->advance(snap*m->clock/44100.0);
    VGMChanTick st[4];
    apu->snapshot(st);
    bool aud=(st[0].aud || st[1].aud || st[2].aud || st[3].aud);
    for (int i=0; i<3; i++) {
      bool on=!(ayRegs[7]&(1<<i)) || !(ayRegs[7]&(8<<i));
      if (on && (ayRegs[8+i]&31)) aud=true;
    }
    for (int i=0; i<3; i++) {
      if ((vrcRegs[i*4+2]&0x80) && !(vrcRegs[3]&1) && (vrcRegs[i*4]&((i<2)?15:63))) aud=true;
    }
    audible.push_back(aud);

    // what is heard in this frame: pitch, volume and duty of every channel,
    // the 5B registers and DMC starts. drivers don't always write the same
    // way twice, but a repeating song sounds the same.
    {
      unsigned char sb[64];
      int sp=0;
      for (int c=0; c<4; c++) {
        sb[sp++]=st[c].aud?1:0;
        sb[sp++]=st[c].aud?(st[c].period&0xff):0;
        sb[sp++]=st[c].aud?(st[c].period>>8):0;
        sb[sp++]=st[c].aud?st[c].vol:0;
        sb[sp++]=st[c].aud?st[c].duty:0;
      }
      for (int r=0; r<14; r++) sb[sp++]=ayRegs[r];
      for (int r=0; r<12; r++) sb[sp++]=vrcRegs[r];
      unsigned char dmc=0;
      for (auto& w: m->frameWrites) {
        if (w.first==0x15 && (w.second&0x10)) dmc++;
        if (w.first==0x11) dmc+=0x10;
      }
      sb[sp++]=dmc;
      soundHash.push_back(nsfHash(sb,sp));
    }

    unsigned long long wh=1469598103934665603ULL;
    for (auto& w: m->frameWrites) {
      unsigned char b2[3]={(unsigned char)w.first,(unsigned char)(w.first>>8),(unsigned char)w.second};
      wh=nsfHash(b2,3,wh);
    }
    writeHash.push_back(wh);
    {
      int dmcStarts=0;
      for (auto& w: m->frameWrites) {
        if (w.first<0x18) apuImg[w.first]=w.second;
        if (w.first==0x15 && (w.second&0x10)) dmcStarts++;
      }
      static const int grp[4][4]={{0,1,2,3},{4,5,6,7},{8,10,11,-1},{12,14,15,-1}};
      for (int g=0; g<4; g++) {
        unsigned char b[6];
        int n=0;
        for (int k=0; k<4; k++) if (grp[g][k]>=0) b[n++]=apuImg[grp[g][k]];
        b[n++]=(apuImg[0x15]>>g)&1;
        chanHash[g].push_back(nsfHash(b,n));
      }
      unsigned char db[5]={(unsigned char)dmcStarts,apuImg[0x10],apuImg[0x12],apuImg[0x13],(unsigned char)(apuImg[0x15]&0x10)};
      chanHash[4].push_back(nsfHash(db,dmcStarts?5:1));
      if (gVRC6>5) {
        for (int i=0; i<3; i++) {
          unsigned char b[6]={ayRegs[i*2],ayRegs[i*2+1],ayRegs[8+i],(unsigned char)(ayRegs[7]&(9<<i)),ayRegs[6],ayRegs[13]};
          chanHash[g5B+i].push_back(nsfHash(b,6));
        }
      }
      if (nGroups>gVRC6) {
        for (int i=0; i<3; i++) {
          unsigned char b[4]={vrcRegs[i*4],vrcRegs[i*4+1],vrcRegs[i*4+2],(unsigned char)(vrcRegs[3]&1)};
          chanHash[gVRC6+i].push_back(nsfHash(b,4));
        }
      }
    }

    // program state: RAM, work RAM and banks
    unsigned long long h=nsfHash(m->ram,sizeof(m->ram));
    h=nsfHash(m->hi,0x2000,h);
    h=nsfHash(m->bankReg,sizeof(m->bankReg),h);
    h=nsfHash(m->exram,sizeof(m->exram),h);
    stateHash.push_back(h);
    auto seen=stateSeen.find(h);
    if (seen!=stateSeen.end()) {
      int j=seen->second;
      if (f-j<30) {
        // the program stopped changing: the track is over (jingle, sound effect)
        stuckAt=j;
      } else {
        loopStart=j;
        loopEnd=f;
      }
      break;
    }
    stateSeen[h]=f;
  }
  delete apu;
  expWrites=m->expWrites;
  delete m;
  if (playFails>0) warns.push_back(fmt::sprintf("the PLAY routine did not return in time %d times",playFails));

  int frames=(int)writeHash.size();
  if (loopStart<0 && stuckAt<0) {
    // the memory never repeated (a frame counter, for example).
    // look for the sound repeating instead: the shortest cycle that keeps
    // repeating until the end, allowing a few frames to differ (drivers
    // sometimes land a write a frame earlier or later).
    // compare each channel with itself one loop later. drivers with a
    // fractional tempo land notes a frame earlier or later on each pass (so
    // a neighbor frame counts as a match), and some channels never repeat
    // exactly (a noise effect that changes every pass, for example), so the
    // loop is the shortest period where the channels repeat almost always.
    auto chanSame=[&](int g, int x, int y) -> bool {
      for (int d=-1; d<=1; d++) {
        int yy=y+d;
        if (yy>=0 && yy<frames && chanHash[g][x]==chanHash[g][yy]) return true;
      }
      return false;
    };
    // fraction of channel-frames that differ, comparing the last part of the
    // log (already looping) with one period later
    auto scoreL=[&](int L, double giveUp) -> double {
      int w=std::min(frames-L-1,std::max(L,2400));
      if (w<L) return 2.0;
      long long bad=0, total=0;
      for (int i=frames-L-1-w; i<frames-L-1; i+=2) {
        for (int g=0; g<nGroups; g++) {
          if (!chanSame(g,i,i+L)) bad++;
        }
        total+=nGroups;
        // give up early on hopeless periods
        if (total>=400 && (double)bad/(double)total>giveUp) return 2.0;
      }
      return total?((double)bad/(double)total):2.0;
    };
    double bestScore=2.0;
    std::vector<std::pair<int,double>> scores;
    for (int L=60; L*2<=frames; L++) {
      double sc=scoreL(L,std::min(0.5,bestScore*2.0+0.05));
      if (sc<=1.0) scores.push_back(std::make_pair(L,sc));
      if (sc<bestScore) bestScore=sc;
    }
    if (bestScore<=0.12) {
      // the shortest period that is about as good as the best one (twice the
      // loop scores the same as the loop itself)
      int L=0;
      for (auto& sc: scores) {
        if (sc.second<=bestScore+0.02) {
          L=sc.first;
          break;
        }
      }
      // the loop starts where the channels begin to repeat: walk back while
      // 2-second windows stay mostly matching
      const int win=120;
      int j=frames-L-1;
      while (j-win>=0) {
        int bad=0;
        for (int i=j-win; i<j; i++) {
          for (int g=0; g<nGroups; g++) if (!chanSame(g,i,i+L)) bad++;
        }
        if ((double)bad/(double)(win*nGroups)>std::max(0.25,bestScore*3.0)) break;
        j-=win;
      }
      // then frame by frame, as long as every channel matches
      while (j>0) {
        bool ok=true;
        for (int g=0; g<nGroups && ok; g++) if (!chanSame(g,j-1,j-1+L)) ok=false;
        if (!ok) break;
        j--;
      }
      if (frames-L-j>=L) {
        loopStart=j;
        loopEnd=j+L;
        logI("NSF loop by channels: period %d frames, %.1f%% differing",L,bestScore*100.0);
      }
    }
  }

  if (loopStart>=0) {
    // the memory may differ for a while (a pass counter, for example) even
    // though the music already repeats: move the loop back while the writes
    // to the APU stay the same
    int L=loopEnd-loopStart;
    // the memory can take several passes of the music to repeat (when
    // channels loop separately, for example): try shorter loops that the
    // writes agree with
    for (int d=8; d>=2; d--) {
      if (L%d!=0 || L/d<60) continue;
      int Ld=L/d;
      bool same=true;
      for (int i=loopStart; i+Ld<loopEnd && same; i++) {
        if (soundHash[i]!=soundHash[i+Ld] && writeHash[i]!=writeHash[i+Ld]) same=false;
      }
      if (same) {
        L=Ld;
        loopEnd=loopStart+L;
        break;
      }
    }
    // (same writes, or the same sound: either means the same music)
    while (loopStart>0 && (soundHash[loopStart-1]==soundHash[loopStart-1+L] || writeHash[loopStart-1]==writeHash[loopStart-1+L])) {
      loopStart--;
      loopEnd--;
    }
    // a loop that is all silence is an ending, not a loop
    bool anySound=false;
    for (int f=loopStart; f<loopEnd && f<(int)audible.size(); f++) {
      if (audible[f]) {
        anySound=true;
        break;
      }
    }
    if (!anySound) {
      stuckAt=loopStart;
      loopStart=-1;
    }
  }

  int endFrame=frames;
  if (loopStart>=0) {
    log.loopTime=(double)loopStart*period;
    endFrame=loopEnd;
    logI("NSF track %d: loop at %.2fs, %.2fs long",track,(double)loopStart*speed/1000000.0,(double)(loopEnd-loopStart)*speed/1000000.0);
  } else {
    log.loopTime=-1;
    // no loop: end shortly after the last sound
    int last=-1;
    for (int f=(int)audible.size()-1; f>=0; f--) {
      if (audible[f]) {
        last=f;
        break;
      }
    }
    if (stuckAt>=0 && stuckAt>last) last=stuckAt;
    if (last<0) {
      err="this track makes no sound";
      return false;
    }
    endFrame=std::min(frames,last+30);
    if (stuckAt<0 && frames>=maxFrames) {
      warns.push_back(fmt::sprintf("track %d: no loop was found within %d seconds, so it was cut there",track,maxSeconds));
    }
    logI("NSF track %d: no loop, %.2fs",track,(double)endFrame*speed/1000000.0);
  }
  log.endTime=(double)endFrame*period-1.0;
  // drop what comes after the end
  while (!log.events.empty() && log.events.back().time>log.endTime) log.events.pop_back();
  while (!log.dacWrites.empty() && log.dacWrites.back().time>log.endTime) log.dacWrites.pop_back();
  return true;
}

}

// --- SID files (C64 music: PSID/RSID) ---

struct SIDFile {
  bool rsid=false;
  int version=1;
  unsigned short loadAddr=0, initAddr=0, playAddr=0;
  int songs=1, start=1;
  unsigned int speed=0;
  unsigned short flags=0;
  String title, artist, released;
  std::vector<unsigned char> data;
  bool pal=true;
  bool is8580=false;
  bool multiSID=false;
};

bool sidParse(const unsigned char* f, size_t len, SIDFile& sid, String& err) {
  if (len<0x7c && len<0x76) {
    err="file too short";
    return false;
  }
  if (memcmp(f,"PSID",4)!=0 && memcmp(f,"RSID",4)!=0) {
    err="not a SID file";
    return false;
  }
  auto r16=[&](size_t p) -> unsigned short {
    return (unsigned short)((f[p]<<8)|f[p+1]);
  };
  sid.rsid=(f[0]=='R');
  sid.version=r16(4);
  unsigned short dataOff=r16(6);
  sid.loadAddr=r16(8);
  sid.initAddr=r16(10);
  sid.playAddr=r16(12);
  sid.songs=r16(14);
  sid.start=r16(16);
  sid.speed=((unsigned int)f[18]<<24)|((unsigned int)f[19]<<16)|((unsigned int)f[20]<<8)|f[21];
  sid.title=nsfString(f+0x16,32);
  sid.artist=nsfString(f+0x36,32);
  sid.released=nsfString(f+0x56,32);
  if (sid.version>=2 && len>=0x78) {
    sid.flags=r16(0x76);
    int clk=(sid.flags>>2)&3;
    sid.pal=(clk!=2);
    sid.is8580=(((sid.flags>>4)&3)==2);
    if (sid.version>=3 && len>=0x7c && (f[0x7a]!=0 || f[0x7b]!=0)) sid.multiSID=true;
  }
  if (sid.flags&1) {
    err="this SID file is in Compute!'s Sidplayer format (MUS), which can't be imported";
    return false;
  }
  if (dataOff>=len) {
    err="the SID file has no data";
    return false;
  }
  size_t p=dataOff;
  if (sid.loadAddr==0) {
    if (p+2>len) {
      err="the SID file has no data";
      return false;
    }
    sid.loadAddr=f[p]|(f[p+1]<<8);
    p+=2;
  }
  sid.data.assign(f+p,f+len);
  if ((size_t)sid.loadAddr+sid.data.size()>0x10000) sid.data.resize(0x10000-sid.loadAddr);
  if (sid.initAddr==0) sid.initAddr=sid.loadAddr;
  if (sid.songs<1) sid.songs=1;
  if (sid.songs>256) sid.songs=256;
  if (sid.start<1 || sid.start>sid.songs) sid.start=1;
  return true;
}

// the shortest period after which the channels keep repeating (allowing a
// few frames to differ), for programs whose memory never repeats
static bool vgmSoundLoop(const std::vector<std::vector<unsigned long long>>& chanHash, int frames, int& loopStart, int& loopEnd, double& score) {
  const int nGroups=(int)chanHash.size();
  auto chanSame=[&](int g, int x, int y) -> bool {
    for (int d=-1; d<=1; d++) {
      int yy=y+d;
      if (yy>=0 && yy<frames && chanHash[g][x]==chanHash[g][yy]) return true;
    }
    return false;
  };
  auto scoreL=[&](int L, double giveUp) -> double {
    int w=std::min(frames-L-1,std::max(L,2400));
    if (w<L) return 2.0;
    long long bad=0, total=0;
    for (int i=frames-L-1-w; i<frames-L-1; i+=2) {
      for (int g=0; g<nGroups; g++) {
        if (!chanSame(g,i,i+L)) bad++;
      }
      total+=nGroups;
      if (total>=400 && (double)bad/(double)total>giveUp) return 2.0;
    }
    return total?((double)bad/(double)total):2.0;
  };
  double bestScore=2.0;
  std::vector<std::pair<int,double>> scores;
  for (int L=60; L*2<=frames; L++) {
    double sc=scoreL(L,std::min(0.5,bestScore*2.0+0.05));
    if (sc<=1.0) scores.push_back(std::make_pair(L,sc));
    if (sc<bestScore) bestScore=sc;
  }
  score=bestScore;
  if (bestScore>0.12) return false;
  int L=0;
  for (auto& sc: scores) {
    if (sc.second<=bestScore+0.02) {
      L=sc.first;
      break;
    }
  }
  const int win=120;
  int j=frames-L-1;
  while (j-win>=0) {
    int bad=0;
    for (int i=j-win; i<j; i++) {
      for (int g=0; g<nGroups; g++) if (!chanSame(g,i,i+L)) bad++;
    }
    if ((double)bad/(double)(win*nGroups)>std::max(0.25,bestScore*3.0)) break;
    j-=win;
  }
  while (j>0) {
    bool ok=true;
    for (int g=0; g<nGroups && ok; g++) if (!chanSame(g,j-1,j-1+L)) ok=false;
    if (!ok) break;
    j--;
  }
  if (frames-L-j<L) return false;
  loopStart=j;
  loopEnd=j+L;
  return true;
}

bool sidRunTrack(const SIDFile& sid, int track, int maxSeconds, VGMNESLog& log, std::vector<String>& warns, String& err) {
  NSFMachine* m=new NSFMachine;
  std::vector<unsigned char> mem(0x10000,0);
  m->c64=true;
  m->mem=mem.data();
  m->log=&log;
  m->retAddr=0xdf00;
  m->clock=sid.pal?985248.0:1022727.0;
  m->linesPerFrame=sid.pal?312:263;
  m->ciaLatch=sid.pal?0x4025:0x4295;
  m->c64Setup();
  memcpy(&mem[sid.loadAddr],sid.data.data(),sid.data.size());
  unsigned int loadEnd=sid.loadAddr+(unsigned int)sid.data.size();
  // memory map: the tune's own memory is RAM
  if (sid.rsid) {
    mem[1]=0x37;
  } else if (loadEnd>0xe000) {
    mem[1]=0x35;
  } else if (loadEnd>0xa000 && sid.loadAddr<0xc000) {
    mem[1]=0x36;
  } else {
    mem[1]=0x37;
  }
  mem[0]=0x2f;
  log.clock=m->clock;
  log.pal=sid.pal;
  log.hasNES=false;
  log.hasSID=true;
  log.sid8580=sid.is8580;
  log.sidClock=m->clock;

  m->callTime=0;
  m->callCycles=m->cycles;
  m->A=(unsigned char)(track-1);
  m->X=0;
  m->Y=0;
  m->S=0xfd;
  m->P=0x24;
  bool initReturned=m->call(sid.initAddr,(long long)(m->clock*4));
  if (!initReturned && !sid.rsid) {
    warns.push_back("the init routine of this SID did not return; the result may be wrong");
  }
  m->S=0xfd;
  // how the player is called: the play routine (PSID), or the interrupt the
  // init routine set up (RSID, or a PSID without a play address)
  bool useIrq=sid.rsid || sid.playAddr==0;
  bool cia;
  if (useIrq) {
    cia=!(m->vicIrqMask&1) && (m->ciaMask&1);
  } else {
    int bit=std::min(track-1,31);
    cia=(sid.speed>>bit)&1;
  }
  double frameCycles=cia?(double)(m->ciaLatch+1):(sid.pal?19656.0:17095.0);
  if (frameCycles<1000) frameCycles=1000;
  double period=frameCycles*44100.0/m->clock;
  double hz=m->clock/frameCycles;
  logI("SID track %d: %s, %.3f Hz",track,useIrq?(cia?"CIA timer interrupt":"raster interrupt"):(cia?"CIA timer":"once per frame"),hz);
  log.fixedPeriod=period;
  log.fixedT0=0;
  log.headerRate=(unsigned int)round(hz);
  // whatever init wrote counts as the start of the first frame
  for (VGMEvent& e: log.events) e.time=0;

  int maxFrames=(int)((double)maxSeconds*hz);
  if (maxFrames<60) maxFrames=60;
  std::map<unsigned long long,int> stateSeen;
  std::vector<unsigned long long> writeHash, soundHash;
  std::vector<std::vector<unsigned long long>> chanHash(4);
  std::vector<bool> audible;
  memset(m->pageDirty,0,sizeof(m->pageDirty));
  int loopStart=-1, loopEnd=-1, stuckAt=-1;
  int playFails=0, digiFrames=0;
  long long frameStart=m->cycles;
  for (int f=0; f<maxFrames; f++) {
    m->callTime=(double)f*period;
    // the frame starts on time, whatever the last call took
    m->cycles=frameStart+(long long)((double)f*frameCycles);
    m->callCycles=m->cycles;
    m->frameWrites.clear();
    m->volWrites=0;
    bool ok;
    if (useIrq) {
      ok=m->irq((long long)(frameCycles*2));
    } else {
      ok=m->call(sid.playAddr,(long long)(frameCycles*2));
    }
    if (!ok) playFails++;
    if (m->volWrites>8) digiFrames++;
    const unsigned char* sr=m->sidRegs;
    bool aud=false;
    for (int v=0; v<3; v++) if (sr[v*7+4]&1) aud=true;
    audible.push_back(aud);
    soundHash.push_back(nsfHash(sr,25));
    unsigned long long wh=1469598103934665603ULL;
    for (auto& w: m->frameWrites) {
      unsigned char b2[3]={(unsigned char)w.first,(unsigned char)(w.first>>8),(unsigned char)w.second};
      wh=nsfHash(b2,3,wh);
    }
    writeHash.push_back(wh);
    for (int v=0; v<3; v++) chanHash[v].push_back(nsfHash(sr+v*7,7));
    chanHash[3].push_back(nsfHash(sr+0x15,4));
    // program state: the memory it writes, and the SID
    unsigned long long h=nsfHash(sr,25);
    for (int pg=0; pg<256; pg++) {
      if (pg==1 || !m->pageDirty[pg]) continue;
      h=nsfHash(&mem[pg<<8],256,h);
    }
    auto seen=stateSeen.find(h);
    if (seen!=stateSeen.end()) {
      int j=seen->second;
      if (f-j<30) {
        stuckAt=j;
      } else {
        loopStart=j;
        loopEnd=f;
      }
      break;
    }
    stateSeen[h]=f;
  }
  delete m;
  if (playFails>0) warns.push_back(fmt::sprintf("the player did not return in time %d times",playFails));
  if (digiFrames>10) warns.push_back("this tune plays samples through the volume register (digi), which can't be imported; they are left out");
  log.sidDigi=(digiFrames>10);
  int frames=(int)writeHash.size();
  if (loopStart<0 && stuckAt<0) {
    double sc=0;
    if (vgmSoundLoop(chanHash,frames,loopStart,loopEnd,sc)) {
      logI("SID loop by channels: period %d frames, %.1f%% differing",loopEnd-loopStart,sc*100.0);
    }
  }
  if (loopStart>=0) {
    int L=loopEnd-loopStart;
    for (int d=8; d>=2; d--) {
      if (L%d!=0 || L/d<60) continue;
      int Ld=L/d;
      bool same=true;
      for (int i=loopStart; i+Ld<loopEnd && same; i++) {
        if (soundHash[i]!=soundHash[i+Ld] && writeHash[i]!=writeHash[i+Ld]) same=false;
      }
      if (same) {
        L=Ld;
        loopEnd=loopStart+L;
        break;
      }
    }
    while (loopStart>0 && (soundHash[loopStart-1]==soundHash[loopStart-1+L] || writeHash[loopStart-1]==writeHash[loopStart-1+L])) {
      loopStart--;
      loopEnd--;
    }
    bool anySound=false;
    for (int f=loopStart; f<loopEnd && f<(int)audible.size(); f++) {
      if (audible[f]) {
        anySound=true;
        break;
      }
    }
    if (!anySound) {
      stuckAt=loopStart;
      loopStart=-1;
    }
  }
  int endFrame=frames;
  if (loopStart>=0) {
    log.loopTime=(double)loopStart*period;
    endFrame=loopEnd;
    logI("SID track %d: loop at %.2fs, %.2fs long",track,(double)loopStart/hz,(double)(loopEnd-loopStart)/hz);
  } else {
    log.loopTime=-1;
    int last=-1;
    for (int f=(int)audible.size()-1; f>=0; f--) {
      if (audible[f]) {
        last=f;
        break;
      }
    }
    if (stuckAt>=0 && stuckAt>last) last=stuckAt;
    if (last<0) {
      err="this tune makes no sound";
      return false;
    }
    // (and let the last notes ring out)
    endFrame=std::min(frames,last+(int)(hz*2));
    if (stuckAt<0 && frames>=maxFrames) {
      warns.push_back(fmt::sprintf("track %d: no loop was found within %d seconds, so it was cut there",track,maxSeconds));
    }
    logI("SID track %d: no loop, %.2fs",track,(double)endFrame/hz);
  }
  log.endTime=(double)endFrame*period-1.0;
  while (!log.events.empty() && log.events.back().time>log.endTime) log.events.pop_back();
  return true;
}

bool gbsInfo(const unsigned char* file, size_t len, DivNSFInfo& info);

bool DivEngine::getNSFInfo(const unsigned char* file, size_t len, DivNSFInfo& info) {
  NSFFile nsf;
  String err;
  if (len>=4 && memcmp(file,"GBS",3)==0) return gbsInfo(file,len,info);
  if (len>=4 && (memcmp(file,"PSID",4)==0 || memcmp(file,"RSID",4)==0)) {
    SIDFile sid;
    if (!sidParse(file,len,sid,err)) return false;
    info.title=sid.title;
    info.artist=sid.artist;
    info.copyright=sid.released;
    info.songs=sid.songs;
    info.start=sid.start;
    info.pal=sid.pal;
    info.chips=sid.is8580?"SID 8580":"SID 6581";
    return true;
  }
  if (!nsfParse(file,len,nsf,err)) return false;
  info.title=nsf.title;
  info.artist=nsf.artist;
  info.copyright=nsf.copyright;
  info.songs=nsf.songs;
  info.start=nsf.start;
  info.trackNames=nsf.trackNames;
  info.trackLengths=nsf.trackLengths;
  info.pal=nsf.pal;
  info.chips="2A03";
  static const char* expNames[6]={"VRC6","VRC7","FDS","MMC5","N163","5B"};
  for (int i=0; i<6; i++) {
    if (nsf.expansion&(1<<i)) info.chips+=String(" + ")+expNames[i];
  }
  return true;
}

bool DivEngine::loadNSF(unsigned char* file, size_t len) {
  bool success=false;
  warnings="";
  try {
    NSFFile nsf;
    String err;
    if (!nsfParse(file,len,nsf,err)) {
      lastError=err;
      throw VGMInvalidException();
    }
    std::vector<int> tracks=nsfImportOptions.tracks;
    if (tracks.empty()) tracks.push_back(nsf.start);
    int maxSeconds=CLAMP(nsfImportOptions.maxSeconds,10,1800);

    DivSong ds;
    ds.version=DIV_VERSION_NSF;
    // a chord arpeggio (00xy) starts over with every note, like the game's
    ds.compatFlags.resetArpPhaseOnNewNote=true;
    ds.systemLen=1;
    ds.system[0]=DIV_SYSTEM_NES;
    ds.systemVol[0]=1.0f;
    ds.systemPan[0]=0.0f;
    ds.systemFlags[0].set("clockSel",nsf.pal?1:0);
    const bool has5B=nsf.expansion&0x20;
    if (has5B) {
      // Famicom + Sunsoft 5B, clocked by the CPU like on the cartridge
      ds.systemLen=2;
      ds.system[1]=DIV_SYSTEM_AY8910;
      // mix like the hardware: a 5B square at volume 12 measures 1.3 dB below
      // a full 2A03 square (NESdev audio level survey, Gimmick! cartridge).
      // in Furnace it comes out 8.3 dB below at 100%/100%, so the 5B gets
      // +7 dB (x2.24). the 2A03 goes down a bit to leave headroom.
      ds.systemVol[0]=0.6f;
      ds.systemVol[1]=0.6f*2.24f;
      ds.systemPan[1]=0.0f;
      ds.systemFlags[1].set("chipType",2);
      ds.systemFlags[1].set("clockSel",nsf.pal?7:0);
    }
    const bool hasVRC6=nsf.expansion&1;
    if (hasVRC6) {
      // Famicom + Konami VRC6, at the levels of Furnace's own preset
      int si=ds.systemLen++;
      ds.system[si]=DIV_SYSTEM_VRC6;
      ds.systemVol[si]=has5B?0.6f:1.0f;
      ds.systemPan[si]=0.0f;
      ds.systemFlags[si].set("clockSel",nsf.pal?1:0);
    }
    ds.initDefaultSystemChans();
    ds.name=nsf.title;
    ds.author=nsf.artist;
    ds.category=nsf.title;
    ds.notes=fmt::sprintf("Imported from NSF.\nGame: %s\nArtist: %s\nCopyright: %s",nsf.title,nsf.artist,nsf.copyright);

    VGMNESContext ctx;
    int expWritesTotal=0;
    int imported=0;
    for (size_t ti=0; ti<tracks.size(); ti++) {
      int track=tracks[ti];
      if (track<1 || track>nsf.songs) continue;
      // share the instruments fairly between the tracks still to import
      int tracksLeft=(int)(tracks.size()-ti);
      ctx.shareLimit=(tracks.size()>1)?std::max(24,(256-(int)ds.ins.size())/tracksLeft):0;
      VGMNESLog log;
      log.has5B=has5B;
      log.hasVRC6=hasVRC6;
      std::vector<String> warns;
      int expWrites=0;
      if (!nsfRunTrack(nsf,track,maxSeconds,log,warns,err,expWrites)) {
        addWarning(fmt::sprintf("track %d: %s",track,err));
        continue;
      }
      expWritesTotal+=expWrites;
      DivSubSong* sub;
      if (imported==0) {
        sub=ds.subsong[0];
      } else {
        sub=new DivSubSong;
        ds.subsong.push_back(sub);
      }
      String trackName=(track-1<(int)nsf.trackNames.size() && !nsf.trackNames[track-1].empty())?nsf.trackNames[track-1]:fmt::sprintf("Track %d",track);
      sub->name=trackName;
      ctx.gridMode=vgmPickGrid(log,ctx,ds);
      if (!vgmConvertNES(log,ctx,ds,sub,warns,err)) {
        addWarning(fmt::sprintf("track %d: %s",track,err));
        if (imported>0) {
          ds.subsong.pop_back();
          delete sub;
        }
        continue;
      }
      for (String& w: warns) {
        String full=(tracks.size()>1)?fmt::sprintf("%s: %s",trackName,w):w;
        addWarning(full);
      }
      imported++;
    }
    if (imported==0) {
      lastError=err.empty()?String("no track could be imported"):err;
      throw VGMInvalidException();
    }
    if (tracks.size()==1) {
      int t=tracks[0];
      if (t-1<(int)nsf.trackNames.size() && !nsf.trackNames[t-1].empty()) {
        ds.name=nsf.trackNames[t-1];
      } else {
        ds.name=fmt::sprintf("%s - Track %d",nsf.title,t);
      }
    }

    ds.insLen=(int)ds.ins.size();
    ds.sampleLen=(int)ds.sample.size();
    ds.recalcChans();

    if (ctx.instrumentsOver>0) addWarning(fmt::sprintf("%d DPCM/PCM hits needed more than 256 instruments and were left out",ctx.instrumentsOver));
    if ((nsf.expansion&~0x21) || expWritesTotal>0) {
      addWarning("this NSF uses an expansion sound chip that can't be imported yet (VRC7, FDS, MMC5 or N163), so that part is missing. (the VRC6 and the Sunsoft 5B are imported.)");
    }

    if (active) quitDispatch();
    BUSY_BEGIN_SOFT;
    saveLock.lock();
    song.unload();
    song=ds;
    hasLoadedSomething=true;
    changeSong(0);
    saveLock.unlock();
    BUSY_END;
    if (active) {
      initDispatch();
      BUSY_BEGIN;
      renderSamples();
      reset();
      BUSY_END;
    }
    success=true;
  } catch (VGMInvalidException& e) {
  }
  delete[] file;
  return success;
}

bool DivEngine::loadSID(unsigned char* file, size_t len) {
  bool success=false;
  warnings="";
  try {
    SIDFile sid;
    String err;
    if (!sidParse(file,len,sid,err)) {
      lastError=err;
      throw VGMInvalidException();
    }
    std::vector<int> tracks=nsfImportOptions.tracks;
    if (tracks.empty()) tracks.push_back(sid.start);
    int maxSeconds=CLAMP(nsfImportOptions.maxSeconds,10,1800);

    DivSong ds;
    ds.version=DIV_VERSION_NSF;
    ds.compatFlags.resetArpPhaseOnNewNote=true;
    ds.systemLen=1;
    ds.system[0]=sid.is8580?DIV_SYSTEM_C64_8580:DIV_SYSTEM_C64_6581;
    ds.systemVol[0]=1.0f;
    ds.systemPan[0]=0.0f;
    ds.systemFlags[0].set("clockSel",sid.pal?1:0);
    // no hard reset before notes: the player's own register writes are
    // imported as they are
    ds.systemFlags[0].set("initResetTime",0);
    // a cutoff macro only writes while it runs (once over, the pattern or
    // another note may change the cutoff)
    ds.systemFlags[0].set("macroRace",true);
    // (the volume macro sets the volume of the whole chip: once over it must
    // stop writing it, like the other macros)
    ds.compatFlags.volMacroLinger=false;
    ds.initDefaultSystemChans();
    ds.name=sid.title;
    ds.author=sid.artist;
    ds.category=sid.title;
    ds.notes=fmt::sprintf("Imported from SID.\nTitle: %s\nAuthor: %s\nReleased: %s",sid.title,sid.artist,sid.released);

    VGMNESContext ctx;
    int imported=0;
    for (size_t ti=0; ti<tracks.size(); ti++) {
      int track=tracks[ti];
      if (track<1 || track>sid.songs) continue;
      int tracksLeft=(int)(tracks.size()-ti);
      ctx.shareLimit=(tracks.size()>1)?std::max(24,(256-(int)ds.ins.size())/tracksLeft):0;
      VGMNESLog log;
      std::vector<String> warns;
      if (!sidRunTrack(sid,track,maxSeconds,log,warns,err)) {
        addWarning(fmt::sprintf("track %d: %s",track,err));
        continue;
      }
      DivSubSong* sub;
      if (imported==0) {
        sub=ds.subsong[0];
      } else {
        sub=new DivSubSong;
        ds.subsong.push_back(sub);
      }
      String trackName=fmt::sprintf("Track %d",track);
      sub->name=trackName;
      ctx.gridMode=0;
      ctx.sidFreePW=vgmPickGrid(log,ctx,ds,true);
      ctx.gridMode=vgmPickGrid(log,ctx,ds);
      if (!vgmConvertNES(log,ctx,ds,sub,warns,err)) {
        addWarning(fmt::sprintf("track %d: %s",track,err));
        if (imported>0) {
          ds.subsong.pop_back();
          delete sub;
        }
        continue;
      }
      for (String& w: warns) {
        String full=(tracks.size()>1)?fmt::sprintf("%s: %s",trackName,w):w;
        addWarning(full);
      }
      imported++;
    }
    if (imported==0) {
      lastError=err.empty()?String("no track could be imported"):err;
      throw VGMInvalidException();
    }
    if (tracks.size()==1 && sid.songs>1) ds.name=fmt::sprintf("%s - Track %d",sid.title,tracks[0]);

    ds.insLen=(int)ds.ins.size();
    ds.sampleLen=(int)ds.sample.size();
    ds.recalcChans();
    if (sid.multiSID) addWarning("this tune uses more than one SID; only the first one is imported");

    if (active) quitDispatch();
    BUSY_BEGIN_SOFT;
    saveLock.lock();
    song.unload();
    song=ds;
    hasLoadedSomething=true;
    changeSong(0);
    saveLock.unlock();
    BUSY_END;
    if (active) {
      initDispatch();
      BUSY_BEGIN;
      renderSamples();
      reset();
      BUSY_END;
    }
    success=true;
  } catch (VGMInvalidException& e) {
  }
  delete[] file;
  return success;
}

namespace {

// ---------------------------------------------------------------------------
// SPC import
//
// an SPC is a snapshot of the SNES sound chip: the SPC700's 64 KB of RAM (with
// the game's sound driver in it), its registers, and the DSP's. to know what
// it plays, the driver is run on an SPC700 (with its timers and the DSP, which
// the driver reads back), and every write to the DSP is logged with its time.
// ---------------------------------------------------------------------------

struct SPCFile {
  unsigned char ram[65536];
  unsigned char dsp[128];
  unsigned char iplRam[64]; // the RAM under the IPL ROM
  unsigned short pc;
  unsigned char a, x, y, psw, sp;
  String title, game, artist, dumper, comment;
  int lengthSec=0, fadeMs=0;
};

// the IPL ROM ($FFC0-$FFFF while $F1 bit 7 is set)
static const unsigned char spcIPL[64]={
  0xcd,0xef,0xbd,0xe8,0x00,0xc6,0x1d,0xd0,0xfc,0x8f,0xaa,0xf4,0x8f,0xbb,0xf5,0x78,
  0xcc,0xf4,0xd0,0xfb,0x2f,0x19,0xeb,0xf4,0xd0,0xfc,0x7e,0xf4,0xd0,0x0b,0xe4,0xf5,
  0xcb,0xf4,0xd7,0x00,0xfc,0xd0,0xf3,0xab,0x01,0x10,0xef,0x7e,0xf4,0x10,0xeb,0xba,
  0xf6,0xda,0x00,0xba,0xf4,0xc4,0xf4,0xdd,0x5d,0xd0,0xdb,0x1f,0x00,0x00,0xc0,0xff
};

// cycles of each instruction (taken branches add 2)
static const unsigned char spcCycles[256]={
  2,8,4,5,3,4,3,6,2,6,5,4,5,4,6,8,
  2,8,4,5,4,5,5,6,5,5,6,5,2,2,4,6,
  2,8,4,5,3,4,3,6,2,6,5,4,5,4,5,4,
  2,8,4,5,4,5,5,6,5,5,6,5,2,2,3,8,
  2,8,4,5,3,4,3,6,2,6,4,4,5,4,6,6,
  2,8,4,5,4,5,5,6,5,5,4,5,2,2,4,3,
  2,8,4,5,3,4,3,6,2,6,4,4,5,4,5,5,
  2,8,4,5,4,5,5,6,5,5,5,5,2,2,3,6,
  2,8,4,5,3,4,3,6,2,6,5,4,5,2,4,5,
  2,8,4,5,4,5,5,6,5,5,5,5,2,2,12,5,
  3,8,4,5,3,4,3,6,2,6,4,4,5,2,4,4,
  2,8,4,5,4,5,5,6,5,5,5,5,2,2,3,4,
  3,8,4,5,4,5,4,7,2,5,6,4,5,2,4,9,
  2,8,4,5,5,6,6,7,4,5,5,5,2,2,6,3,
  2,8,4,5,3,4,3,6,2,4,5,3,4,3,4,1,
  2,8,4,5,4,5,5,6,3,4,5,4,2,2,4,1
};

static String spcText(const unsigned char* p, size_t n) {
  size_t l=0;
  while (l<n && p[l]!=0) l++;
  String r((const char*)p,l);
  while (!r.empty() && (r.back()==' ')) r.pop_back();
  return r;
}

bool spcParse(const unsigned char* f, size_t len, SPCFile& spc, String& err) {
  if (len<0x10180 || memcmp(f,"SNES-SPC700 Sound File Data",27)!=0) {
    err="not an SPC file";
    return false;
  }
  spc.pc=f[0x25]|(f[0x26]<<8);
  spc.a=f[0x27];
  spc.x=f[0x28];
  spc.y=f[0x29];
  spc.psw=f[0x2a];
  spc.sp=f[0x2b];
  memcpy(spc.ram,f+0x100,65536);
  memcpy(spc.dsp,f+0x10100,128);
  if (len>=0x10200) {
    memcpy(spc.iplRam,f+0x101c0,64);
  } else {
    memcpy(spc.iplRam,spc.ram+0xffc0,64);
  }
  // ID666 tag (text or binary: the length field tells them apart)
  if (f[0x23]==26) {
    spc.title=spcText(f+0x2e,32);
    spc.game=spcText(f+0x4e,32);
    spc.dumper=spcText(f+0x6e,16);
    spc.comment=spcText(f+0x7e,32);
    bool text=true;
    for (int i=0; i<3; i++) {
      unsigned char c=f[0xa9+i];
      if (c!=0 && (c<'0' || c>'9')) text=false;
    }
    for (int i=0; i<5; i++) {
      unsigned char c=f[0xac+i];
      if (c!=0 && (c<'0' || c>'9')) text=false;
    }
    if (text) {
      spc.lengthSec=atoi(spcText(f+0xa9,3).c_str());
      spc.fadeMs=atoi(spcText(f+0xac,5).c_str());
      spc.artist=spcText(f+0xb1,32);
    } else {
      spc.lengthSec=f[0xa9]|(f[0xaa]<<8);
      spc.fadeMs=f[0xac]|(f[0xad]<<8)|(f[0xae]<<16)|(f[0xaf]<<24);
      spc.artist=spcText(f+0xb0,32);
    }
  }
  return true;
}

// the SPC700 with its timers, I/O ports and the DSP
struct SPCMachine {
  unsigned char ram[65536];
  unsigned char iplRam[64];
  SPC_DSP dsp;
  unsigned short pc;
  unsigned char a, x, y, sp, psw;
  bool stopped=false;
  // cycles since the start (1.024 MHz), and the DSP's
  unsigned long long cyc=0, dspCyc=0;
  // I/O
  unsigned char dspAddr=0;
  unsigned char inPort[4];
  bool iplOn=false;
  // timers: 0 and 1 at 8 kHz, 2 at 64 kHz
  struct Timer {
    bool on=false;
    int target=256;
    int inner=0, out=0;
    int stage=0;
    int period=128;
    // (when it last ticked, for the song's tick)
    long long lastTick=-1;
  } timer[3];
  // the DSP writes, in the order they happen
  struct DSPWrite {
    unsigned long long cyc;
    unsigned char reg, val;
    // (the driver's frame for each timer: see frameCyc)
    unsigned int fr[3];
  };
  std::vector<DSPWrite> writes;
  // the driver's frames: when it reads a timer's counter and finds it
  // ticked (the cycle of each such read)
  std::vector<unsigned long long> frameCyc[3];

  bool noEchoDbg=false;
  void load(const SPCFile& f) {
    memcpy(ram,f.ram,65536);
    memcpy(iplRam,f.iplRam,64);
    pc=f.pc;
    a=f.a;
    x=f.x;
    y=f.y;
    sp=f.sp;
    psw=f.psw;
    dsp.init(ram);
    dsp.load(f.dsp);
    noEchoDbg=getenv("FURNACE_SPC_NOECHO")!=NULL;
    if (noEchoDbg) {
      dsp.write(0x2c,0);
      dsp.write(0x3c,0);
    }
    // the I/O registers as the snapshot left them
    unsigned char ctrl=ram[0xf1];
    for (int i=0; i<3; i++) {
      timer[i].on=ctrl&(1<<i);
      timer[i].target=ram[0xfa+i]?ram[0xfa+i]:256;
      timer[i].out=ram[0xfd+i]&15;
      timer[i].period=(i==2)?16:128;
    }
    iplOn=ctrl&0x80;
    dspAddr=ram[0xf2];
    for (int i=0; i<4; i++) inPort[i]=ram[0xf4+i];
  }

  void runDSP() {
    if (cyc>dspCyc) {
      unsigned long long d=cyc-dspCyc;
      while (d>0) {
        int n=(d>(1u<<20))?(1<<20):(int)d;
        dsp.run(n);
        d-=n;
      }
      dspCyc=cyc;
    }
  }

  void tickTimers(int c) {
    for (int i=0; i<3; i++) {
      Timer& t=timer[i];
      t.stage+=c;
      while (t.stage>=t.period) {
        t.stage-=t.period;
        if (!t.on) continue;
        if (++t.inner>=t.target) {
          t.inner=0;
          t.out=(t.out+1)&15;
          t.lastTick=(long long)(cyc+c)-t.stage;
        }
      }
    }
  }

  unsigned char read(unsigned short addr) {
    if (addr>=0xf0 && addr<=0xff) {
      switch (addr) {
        case 0xf2: return dspAddr;
        case 0xf3:
          runDSP();
          return dsp.read(dspAddr&0x7f);
        case 0xf4: case 0xf5: case 0xf6: case 0xf7:
          return inPort[addr-0xf4];
        case 0xf8: case 0xf9:
          return ram[addr];
        case 0xfd: case 0xfe: case 0xff: {
          int r=timer[addr-0xfd].out;
          timer[addr-0xfd].out=0;
          for (int i=0; i<r; i++) frameCyc[addr-0xfd].push_back(cyc);
          return r;
        }
        default:
          return 0;
      }
    }
    if (addr>=0xffc0 && iplOn) return spcIPL[addr-0xffc0];
    return ram[addr];
  }

  void write(unsigned short addr, unsigned char v) {
    if (addr>=0xf0 && addr<=0xff) {
      switch (addr) {
        case 0xf1:
          for (int i=0; i<3; i++) {
            bool on=v&(1<<i);
            if (on && !timer[i].on) {
              timer[i].inner=0;
              timer[i].out=0;
            }
            timer[i].on=on;
          }
          if (v&0x10) inPort[0]=inPort[1]=0;
          if (v&0x20) inPort[2]=inPort[3]=0;
          iplOn=v&0x80;
          break;
        case 0xf2:
          dspAddr=v;
          break;
        case 0xf3:
          if (dspAddr<0x80) {
            runDSP();
            // DEBUGREF: the original without its echo
            if (noEchoDbg && (dspAddr==0x2c || dspAddr==0x3c)) {
              dsp.write(dspAddr,0);
            } else {
              dsp.write(dspAddr,v);
            }
            DSPWrite w;
            w.cyc=cyc;
            w.reg=dspAddr;
            w.val=v;
            for (int i=0; i<3; i++) w.fr[i]=(unsigned int)frameCyc[i].size();
            writes.push_back(w);
          }
          break;
        case 0xfa: case 0xfb: case 0xfc:
          timer[addr-0xfa].target=v?v:256;
          break;
        default:
          break;
      }
      ram[addr]=v;
      return;
    }
    ram[addr]=v;
  }

  // flags
  enum { FN=0x80, FV=0x40, FP=0x20, FB=0x10, FH=0x08, FI=0x04, FZ=0x02, FC=0x01 };
  inline void setNZ(unsigned char v) {
    psw=(psw&~(FN|FZ))|(v&0x80)|(v?0:FZ);
  }
  inline unsigned short dp(unsigned char a8) {
    return ((psw&FP)?0x100:0)|a8;
  }
  inline unsigned char fetch() {
    return read(pc++);
  }
  inline unsigned short fetch16() {
    unsigned char l=fetch();
    unsigned char h=fetch();
    return l|(h<<8);
  }
  inline unsigned short readDP16(unsigned char a8) {
    unsigned char l=read(dp(a8));
    unsigned char h=read(dp((unsigned char)(a8+1)));
    return l|(h<<8);
  }
  inline void push(unsigned char v) {
    ram[0x100|sp]=v;
    sp--;
  }
  inline unsigned char pop() {
    sp++;
    return ram[0x100|sp];
  }
  inline unsigned char adc(unsigned char p, unsigned char q) {
    int r=p+q+(psw&FC);
    psw&=~(FV|FH|FC);
    if (~(p^q)&(p^r)&0x80) psw|=FV;
    if ((p^q^r)&0x10) psw|=FH;
    if (r>0xff) psw|=FC;
    setNZ(r&0xff);
    return r&0xff;
  }
  inline unsigned char sbc(unsigned char p, unsigned char q) {
    return adc(p,~q);
  }
  inline void cmp(unsigned char p, unsigned char q) {
    int r=p-q;
    psw=(psw&~FC)|((r>=0)?FC:0);
    setNZ(r&0xff);
  }
  inline unsigned char asl(unsigned char v) {
    psw=(psw&~FC)|((v&0x80)?FC:0);
    v<<=1;
    setNZ(v);
    return v;
  }
  inline unsigned char lsr(unsigned char v) {
    psw=(psw&~FC)|(v&1);
    v>>=1;
    setNZ(v);
    return v;
  }
  inline unsigned char rol(unsigned char v) {
    int c=psw&FC;
    psw=(psw&~FC)|((v&0x80)?FC:0);
    v=(v<<1)|c;
    setNZ(v);
    return v;
  }
  inline unsigned char ror(unsigned char v) {
    int c=(psw&FC)?0x80:0;
    psw=(psw&~FC)|(v&1);
    v=(v>>1)|c;
    setNZ(v);
    return v;
  }
  inline void branch(bool cond, int& c) {
    signed char r=(signed char)fetch();
    if (cond) {
      pc+=r;
      c+=2;
    }
  }

  // one instruction; returns its cycles
  int step() {
    if (stopped) return 2;
    unsigned char op=fetch();
    int c=spcCycles[op];
    const int hi=op>>4;
    switch (op&15) {
      case 0x0: {
        switch (op) {
          case 0x00: break;
          case 0x10: branch(!(psw&FN),c); break;
          case 0x20: psw&=~FP; break;
          case 0x30: branch(psw&FN,c); break;
          case 0x40: psw|=FP; break;
          case 0x50: branch(!(psw&FV),c); break;
          case 0x60: psw&=~FC; break;
          case 0x70: branch(psw&FV,c); break;
          case 0x80: psw|=FC; break;
          case 0x90: branch(!(psw&FC),c); break;
          case 0xa0: psw|=FI; break;
          case 0xb0: branch(psw&FC,c); break;
          case 0xc0: psw&=~FI; break;
          case 0xd0: branch(!(psw&FZ),c); break;
          case 0xe0: psw&=~(FV|FH); break;
          case 0xf0: branch(psw&FZ,c); break;
        }
        break;
      }
      case 0x1: {
        // TCALL n
        push(pc>>8);
        push(pc&0xff);
        unsigned short v=0xffde - (hi<<1);
        pc=read(v)|(read(v+1)<<8);
        break;
      }
      case 0x2: {
        // SET1/CLR1 d.b
        unsigned short ad=dp(fetch());
        unsigned char v=read(ad);
        int bit=hi>>1;
        if (hi&1) {
          v&=~(1<<bit);
        } else {
          v|=(1<<bit);
        }
        write(ad,v);
        break;
      }
      case 0x3: {
        // BBS/BBC d.b,r
        unsigned char v=read(dp(fetch()));
        int bit=hi>>1;
        bool set=v&(1<<bit);
        branch((hi&1)?!set:set,c);
        break;
      }
      case 0x4: case 0x5: case 0x6: case 0x7: case 0x8: {
        // the ALU group: OR, AND, EOR, CMP, ADC, SBC (hi 0-11), and the
        // MOVs (hi 12-15)
        const int lo=op&15;
        unsigned short ad=0;
        bool imm=false;
        unsigned char immV=0;
        // (x8 with odd hi: "op d,#i"; with even hi: "op A,#i")
        if (lo==0x8 && hi<12) {
          if (hi&1) {
            unsigned char i=fetch();
            ad=dp(fetch());
            unsigned char m=read(ad);
            unsigned char r=0;
            switch (hi>>1) {
              case 0: r=m|i; setNZ(r); break;
              case 1: r=m&i; setNZ(r); break;
              case 2: r=m^i; setNZ(r); break;
              case 3: cmp(m,i); r=m; break;
              case 4: r=adc(m,i); break;
              case 5: r=sbc(m,i); break;
            }
            if ((hi>>1)!=3) write(ad,r);
            break;
          }
          imm=true;
          immV=fetch();
        } else if (lo==0x8) {
          switch (op) {
            case 0xc8: cmp(x,fetch()); break;
            case 0xd8: write(dp(fetch()),x); break;
            case 0xe8: a=fetch(); setNZ(a); break;
            case 0xf8: x=read(dp(fetch())); setNZ(x); break;
          }
          break;
        } else {
          // the address
          switch (lo) {
            case 0x4:
              ad=(hi&1)?dp((unsigned char)(fetch()+x)):dp(fetch());
              break;
            case 0x5:
              ad=fetch16();
              if (hi&1) ad+=x;
              break;
            case 0x6:
              if (hi&1) {
                ad=fetch16()+y;
              } else {
                ad=dp(x);
              }
              break;
            case 0x7:
              if (hi&1) {
                ad=readDP16(fetch())+y;
              } else {
                ad=readDP16((unsigned char)(fetch()+x));
              }
              break;
          }
        }
        if (hi>=12) {
          // MOV
          if (hi<14) {
            write(ad,a);
          } else {
            a=read(ad);
            setNZ(a);
          }
          break;
        }
        unsigned char m=imm?immV:read(ad);
        switch (hi>>1) {
          case 0: a|=m; setNZ(a); break;
          case 1: a&=m; setNZ(a); break;
          case 2: a^=m; setNZ(a); break;
          case 3: cmp(a,m); break;
          case 4: a=adc(a,m); break;
          case 5: a=sbc(a,m); break;
        }
        break;
      }
      case 0x9: {
        switch (op) {
          case 0xc9: write(fetch16(),x); break;
          case 0xd9: write(dp((unsigned char)(fetch()+y)),x); break;
          case 0xe9: x=read(fetch16()); setNZ(x); break;
          case 0xf9: x=read(dp((unsigned char)(fetch()+y))); setNZ(x); break;
          default: {
            // op dd,ds (even hi) or op (X),(Y) (odd hi)
            unsigned short da, sa;
            if (hi&1) {
              sa=dp(y);
              da=dp(x);
            } else {
              sa=dp(fetch());
              da=dp(fetch());
            }
            unsigned char s=read(sa);
            unsigned char d=read(da);
            unsigned char r=0;
            switch (hi>>1) {
              case 0: r=d|s; setNZ(r); break;
              case 1: r=d&s; setNZ(r); break;
              case 2: r=d^s; setNZ(r); break;
              case 3: cmp(d,s); r=d; break;
              case 4: r=adc(d,s); break;
              case 5: r=sbc(d,s); break;
            }
            if ((hi>>1)!=3) write(da,r);
            break;
          }
        }
        break;
      }
      case 0xa: {
        switch (op) {
          case 0x0a: case 0x2a: case 0x4a: case 0x6a: case 0x8a: case 0xaa: case 0xca: case 0xea: {
            unsigned short w=fetch16();
            unsigned short ad=w&0x1fff;
            int bit=w>>13;
            unsigned char m=read(ad);
            bool b=(m>>bit)&1;
            bool cf=psw&FC;
            switch (op) {
              case 0x0a: cf=cf||b; break;
              case 0x2a: cf=cf||!b; break;
              case 0x4a: cf=cf&&b; break;
              case 0x6a: cf=cf&&!b; break;
              case 0x8a: cf=cf!=b; break;
              case 0xaa: cf=b; break;
              case 0xca:
                m=cf?(m|(1<<bit)):(m&~(1<<bit));
                write(ad,m);
                break;
              case 0xea:
                m^=(1<<bit);
                write(ad,m);
                break;
            }
            psw=(psw&~FC)|(cf?FC:0);
            break;
          }
          case 0x1a: case 0x3a: {
            // DECW/INCW d
            unsigned char a8=fetch();
            unsigned short v=readDP16(a8);
            v+=(op==0x3a)?1:-1;
            write(dp(a8),v&0xff);
            write(dp((unsigned char)(a8+1)),v>>8);
            psw=(psw&~(FN|FZ))|((v&0x8000)?FN:0)|(v?0:FZ);
            break;
          }
          case 0x5a: {
            // CMPW YA,d
            unsigned short v=readDP16(fetch());
            int ya=(y<<8)|a;
            int r=ya-v;
            psw=(psw&~(FN|FZ|FC))|((r&0x8000)?FN:0)|((r&0xffff)?0:FZ)|((r>=0)?FC:0);
            break;
          }
          case 0x7a: case 0x9a: {
            // ADDW/SUBW YA,d
            unsigned short v=readDP16(fetch());
            int ya=(y<<8)|a;
            int r;
            psw&=~(FV|FH|FC);
            if (op==0x7a) {
              r=ya+v;
              if (~(ya^v)&(ya^r)&0x8000) psw|=FV;
              if ((ya^v^r)&0x1000) psw|=FH;
              if (r>0xffff) psw|=FC;
            } else {
              r=ya-v;
              if ((ya^v)&(ya^r)&0x8000) psw|=FV;
              if (!((ya^v^r)&0x1000)) psw|=FH;
              if (r>=0) psw|=FC;
            }
            r&=0xffff;
            a=r&0xff;
            y=r>>8;
            psw=(psw&~(FN|FZ))|((r&0x8000)?FN:0)|(r?0:FZ);
            break;
          }
          case 0xba: {
            // MOVW YA,d
            unsigned short v=readDP16(fetch());
            a=v&0xff;
            y=v>>8;
            psw=(psw&~(FN|FZ))|((v&0x8000)?FN:0)|(v?0:FZ);
            break;
          }
          case 0xda: {
            // MOVW d,YA
            unsigned char a8=fetch();
            write(dp(a8),a);
            write(dp((unsigned char)(a8+1)),y);
            break;
          }
          case 0xfa: {
            // MOV dd,ds
            unsigned char s=read(dp(fetch()));
            write(dp(fetch()),s);
            break;
          }
        }
        break;
      }
      case 0xb: case 0xc: {
        const bool absA=((op&15)==0xc);
        switch (op) {
          case 0xcb: write(dp(fetch()),y); break;
          case 0xdb: write(dp((unsigned char)(fetch()+x)),y); break;
          case 0xeb: y=read(dp(fetch())); setNZ(y); break;
          case 0xfb: y=read(dp((unsigned char)(fetch()+x))); setNZ(y); break;
          case 0xcc: write(fetch16(),y); break;
          case 0xdc: y--; setNZ(y); break;
          case 0xec: y=read(fetch16()); setNZ(y); break;
          case 0xfc: y++; setNZ(y); break;
          case 0x1c: a=asl(a); break;
          case 0x3c: a=rol(a); break;
          case 0x5c: a=lsr(a); break;
          case 0x7c: a=ror(a); break;
          case 0x9c: a--; setNZ(a); break;
          case 0xbc: a++; setNZ(a); break;
          default: {
            // ASL/ROL/LSR/ROR/DEC/INC on memory: d, d+X (xB) or !a (xC)
            unsigned short ad;
            if (absA) {
              ad=fetch16();
            } else {
              ad=(hi&1)?dp((unsigned char)(fetch()+x)):dp(fetch());
            }
            unsigned char v=read(ad);
            switch (hi>>1) {
              case 0: v=asl(v); break;
              case 1: v=rol(v); break;
              case 2: v=lsr(v); break;
              case 3: v=ror(v); break;
              case 4: v--; setNZ(v); break;
              case 5: v++; setNZ(v); break;
            }
            write(ad,v);
            break;
          }
        }
        break;
      }
      case 0xd: {
        switch (op) {
          case 0x0d: push(psw); break;
          case 0x1d: x--; setNZ(x); break;
          case 0x2d: push(a); break;
          case 0x3d: x++; setNZ(x); break;
          case 0x4d: push(x); break;
          case 0x5d: x=a; setNZ(x); break;
          case 0x6d: push(y); break;
          case 0x7d: a=x; setNZ(a); break;
          case 0x8d: y=fetch(); setNZ(y); break;
          case 0x9d: x=sp; setNZ(x); break;
          case 0xad: cmp(y,fetch()); break;
          case 0xbd: sp=x; break;
          case 0xcd: x=fetch(); setNZ(x); break;
          case 0xdd: a=y; setNZ(a); break;
          case 0xed: psw^=FC; break;
          case 0xfd: y=a; setNZ(y); break;
        }
        break;
      }
      case 0xe: {
        switch (op) {
          case 0x0e: case 0x4e: {
            // TSET1/TCLR1 !a
            unsigned short ad=fetch16();
            unsigned char m=read(ad);
            setNZ((unsigned char)(a-m));
            write(ad,(op==0x0e)?(m|a):(m&~a));
            break;
          }
          case 0x1e: cmp(x,read(fetch16())); break;
          case 0x3e: cmp(x,read(dp(fetch()))); break;
          case 0x5e: cmp(y,read(fetch16())); break;
          case 0x7e: cmp(y,read(dp(fetch()))); break;
          case 0x2e: case 0xde: {
            // CBNE d,r / CBNE d+X,r
            unsigned char a8=fetch();
            unsigned char m=read((op==0xde)?dp((unsigned char)(a8+x)):dp(a8));
            branch(a!=m,c);
            break;
          }
          case 0x6e: {
            // DBNZ d,r
            unsigned short ad=dp(fetch());
            unsigned char m=read(ad)-1;
            write(ad,m);
            branch(m!=0,c);
            break;
          }
          case 0xfe: {
            y--;
            branch(y!=0,c);
            break;
          }
          case 0x8e: psw=pop(); break;
          case 0xae: a=pop(); break;
          case 0xce: x=pop(); break;
          case 0xee: y=pop(); break;
          case 0x9e: {
            // DIV YA,X
            unsigned int ya=(y<<8)|a;
            psw&=~(FV|FH);
            if (y>=x) psw|=FV;
            if ((y&15)>=(x&15)) psw|=FH;
            if (y<(x<<1)) {
              a=x?(ya/x):0xff;
              y=x?(ya%x):(ya&0xff);
            } else {
              a=255-(ya-(x<<9))/(256-x);
              y=x+(ya-(x<<9))%(256-x);
            }
            setNZ(a);
            break;
          }
          case 0xbe: {
            // DAS
            if (!(psw&FC) || a>0x99) {
              a-=0x60;
              psw&=~FC;
            }
            if (!(psw&FH) || (a&15)>9) a-=6;
            setNZ(a);
            break;
          }
        }
        break;
      }
      case 0xf: {
        switch (op) {
          case 0x0f: {
            // BRK
            push(pc>>8);
            push(pc&0xff);
            push(psw);
            psw|=FB;
            psw&=~FI;
            pc=read(0xffde)|(read(0xffdf)<<8);
            break;
          }
          case 0x1f: {
            unsigned short ad=fetch16()+x;
            pc=read(ad)|(read(ad+1)<<8);
            break;
          }
          case 0x2f: branch(true,c); break;
          case 0x3f: {
            unsigned short ad=fetch16();
            push(pc>>8);
            push(pc&0xff);
            pc=ad;
            break;
          }
          case 0x4f: {
            unsigned char u=fetch();
            push(pc>>8);
            push(pc&0xff);
            pc=0xff00|u;
            break;
          }
          case 0x5f: pc=fetch16(); break;
          case 0x6f: {
            unsigned char l=pop();
            unsigned char h=pop();
            pc=l|(h<<8);
            break;
          }
          case 0x7f: {
            psw=pop();
            unsigned char l=pop();
            unsigned char h=pop();
            pc=l|(h<<8);
            break;
          }
          case 0x8f: {
            unsigned char i=fetch();
            write(dp(fetch()),i);
            break;
          }
          case 0x9f: a=(a>>4)|(a<<4); setNZ(a); break;
          case 0xaf: write(dp(x),a); x++; break;
          case 0xbf: a=read(dp(x)); x++; setNZ(a); break;
          case 0xcf: {
            unsigned int r=y*a;
            a=r&0xff;
            y=r>>8;
            setNZ(y);
            break;
          }
          case 0xdf: {
            // DAA
            if ((psw&FC) || a>0x99) {
              a+=0x60;
              psw|=FC;
            }
            if ((psw&FH) || (a&15)>9) a+=6;
            setNZ(a);
            break;
          }
          case 0xef: case 0xff:
            stopped=true;
            break;
        }
        break;
      }
    }
    cyc+=c;
    tickTimers(c);
    return c;
  }
};


// runs the SPC's driver and fills the log: the DSP's registers (events with
// reg 0x800+register) and, at each key on, the sample each voice starts
// (events with reg 0x900+voice, value: an index into log.snesSamples), taken
// from the RAM as it is then
bool spcRunTrack(const SPCFile& spc, int maxSeconds, VGMNESLog& log, std::vector<String>& warns, String& err) {
  SPCMachine* m=new SPCMachine;
  m->load(spc);
  // (the length in the tags: where the song ends if it doesn't loop)
  int secs=(spc.lengthSec>0)?spc.lengthSec:180;
  if (secs>maxSeconds) secs=maxSeconds;
  if (secs<1) secs=1;
  const double clock=1024000.0;
  // the driver runs for up to maxSeconds, looking for the song's loop: the
  // point from which the DSP writes repeat, tick by tick
  const unsigned long long endCyc=(unsigned long long)((double)maxSeconds*clock);
  unsigned char regs[128];
  memcpy(regs,spc.dsp,128);
  log.hasNES=false;
  log.hasSNES=true;
  memcpy(log.snesInit,regs,128);
  // the DSP as the snapshot left it (not KON, ENDX and the read-only ones)
  for (int r=0; r<128; r++) {
    if (r==0x4c || r==0x7c || (r&15)==8 || (r&15)==9) continue;
    VGMEvent e;
    e.time=0;
    e.reg=0x800|r;
    e.val=regs[r];
    log.events.push_back(e);
  }
  // the samples: BRR blocks from the start address until the end flag. a
  // loop inside the sample is a loop point; one somewhere else is played
  // after it (its own blocks, up to their end)
  std::map<String,int> smpMap;
  auto sampleOf=[&](int v) -> int {
    const unsigned char* ram=m->ram;
    unsigned int dir=regs[0x5d]<<8;
    unsigned int e=(dir+regs[v*16+4]*4)&0xffff;
    unsigned int start=ram[e]|(ram[(e+1)&0xffff]<<8);
    unsigned int loop=ram[(e+2)&0xffff]|(ram[(e+3)&0xffff]<<8);
    std::vector<unsigned char> brr;
    unsigned int p=start;
    bool loopFlag=false;
    for (int b=0; b<7281; b++) {
      for (int k=0; k<9; k++) brr.push_back(ram[(p+k)&0xffff]);
      unsigned char h=ram[p&0xffff];
      p=(p+9)&0xffff;
      if (h&1) {
        loopFlag=h&2;
        break;
      }
    }
    int loopBlock=-1;
    if (loopFlag) {
      unsigned int endAddr=start+(unsigned int)brr.size();
      if (loop>=start && loop<endAddr && ((loop-start)%9)==0) {
        loopBlock=(loop-start)/9;
      } else {
        // (the loop plays other blocks: they go after the sample, with no
        // end flags in the middle)
        for (size_t k=0; k<brr.size(); k+=9) brr[k]&=~3;
        loopBlock=(int)(brr.size()/9);
        unsigned int q=loop;
        for (int b=0; b<7281 && brr.size()<65536; b++) {
          for (int k=0; k<9; k++) brr.push_back(ram[(q+k)&0xffff]);
          unsigned char h=ram[q&0xffff];
          q=(q+9)&0xffff;
          if (h&1) break;
        }
      }
    }
    String key=fmt::sprintf("%d:",loopBlock);
    key.append((const char*)brr.data(),brr.size());
    auto it=smpMap.find(key);
    if (it!=smpMap.end()) return it->second;
    VGMSNESSample s;
    s.brr=brr;
    s.loopBlock=loopBlock;
    s.addr=start;
    log.snesSamples.push_back(s);
    smpMap[key]=(int)log.snesSamples.size()-1;
    return smpMap[key];
  };
  // DEBUGREF: the original's sound, to compare with the import
  const char* refPath=getenv("FURNACE_SPC_REF");
  std::vector<short> refOut;
  std::vector<SPC_DSP::sample_t> refBuf(8192);
  if (refPath) m->dsp.set_output(refBuf.data(),(int)refBuf.size());
  if (refPath && getenv("FURNACE_SPC_REFVOICE")) m->dsp.mute_voices(0xff&~(1<<atoi(getenv("FURNACE_SPC_REFVOICE"))));
  int refCount=0;
  bool initSet=false;
  size_t wi=0;
  int keyOns=0;
  unsigned long long nextEnvCyc=0;
  int lastEnvx[8]={-1,-1,-1,-1,-1,-1,-1,-1};
  std::map<int,int> globalChanges;
  // the loop: what the music does on each voice (its key ons, with the
  // sample, the note to the nearest semitone, envelope and rough volume,
  // and its key offs), with the driver's tick it happens in (counted from
  // the first note). not the pitch tick by tick: drivers often run their
  // vibrato on its own, out of step with the song's loop
  std::vector<int> vEvT[8];
  std::vector<unsigned long long> vEvH[8];
  std::vector<unsigned char> tickKon;
  // (the ticks go by the driver's timer as the driver reads it: the writes
  // of one tick can take most of it, so going by time would split them)
  int firstTick=-1;
  int loopS=-1, loopP=-1;
  int tickTimer=-1;
  long long fr0=-1;
  auto framesNow=[&]() -> int {
    if (tickTimer<0 || fr0<0) return 0;
    return (int)((long long)m->frameCyc[tickTimer].size()-fr0);
  };
  // the loop: the earliest tick s, with the shortest period T, such that
  // from s to the end every event of every voice has the same event T ticks
  // before or after it, over at least one whole period (more than that
  // unless this is the end). an event may land a few ticks off its repeat:
  // drivers with a fractional tempo land their notes a tick earlier or
  // later on each pass, each voice on its own
  auto findLoop=[&](bool last, int N) -> bool {
    if (firstTick<0 || log.fixedPeriod<=0) return false;
    double tps=44100.0/log.fixedPeriod;
    int minP=std::max(8,(int)(2.0*tps));
    const int slack=3;
    // (the events before tick N)
    int cnt[8];
    for (int v=0; v<8; v++) {
      cnt[v]=(int)(std::lower_bound(vEvT[v].begin(),vEvT[v].end(),N)-vEvT[v].begin());
    }
    // does voice v have event h within slack of tick t?
    auto hasEv=[&](int v, int t, unsigned long long h) -> bool {
      const std::vector<int>& et=vEvT[v];
      int i=(int)(std::lower_bound(et.begin(),et.begin()+cnt[v],t-slack)-et.begin());
      for (; i<cnt[v] && et[i]<=t+slack; i++) {
        if (vEvH[v][i]==h) return true;
      }
      return false;
    };
    int bestS=-1, bestP=-1;
    for (int T=minP; firstTick+2*T<=N; T++) {
      int st=firstTick;
      // backwards: each event from the end needs its repeat T ticks before
      for (int v=0; v<8; v++) {
        for (int i=cnt[v]-1; i>=0; i--) {
          int t=vEvT[v][i];
          if (t-T<st) break;
          if (!hasEv(v,t-T,vEvH[v][i])) {
            st=std::max(st,t-T+slack+1);
            break;
          }
        }
        if (N-st<T) break;
      }
      if (N-st<T) continue;
      // forwards: each event from s on needs its repeat T ticks after (the
      // start moves on past any that don't, until none is left)
      bool moved=true;
      while (moved && N-st>=T) {
        moved=false;
        for (int v=0; v<8 && !moved; v++) {
          int i=(int)(std::lower_bound(vEvT[v].begin(),vEvT[v].begin()+cnt[v],st)-vEvT[v].begin());
          for (; i<cnt[v]; i++) {
            int t=vEvT[v][i];
            if (t+T+slack>=N) break;
            if (!hasEv(v,t+T,vEvH[v][i])) {
              st=t+1;
              moved=true;
              break;
            }
          }
        }
      }
      // (before the end: two whole periods, and 45 seconds more than one,
      // so a part that repeats inside the song isn't taken for the loop)
      int need=last?T:std::max(2*T,T+(int)(45.0*tps));
      if (N-st<need) continue;
      if (bestS>=0 && st>=bestS) continue;
      // (a loop of silence is an ending, not a loop)
      bool kon=false;
      for (int k=st; k<st+T && k<(int)tickKon.size() && !kon; k++) if (tickKon[k]) kon=true;
      if (!kon) continue;
      bestS=st;
      bestP=T;
    }
    if (bestS<0) return false;
    // the loop starts at its first event (and ends at that event's repeat)
    int s0=-1;
    for (int k=bestS; k<bestS+bestP && k<(int)tickKon.size(); k++) {
      if (tickKon[k]) {
        s0=k;
        break;
      }
    }
    if (s0<0) s0=bestS;
    loopS=s0;
    loopP=bestP;
    return true;
  };
  // key ons that the snapshot left pending (in KON): the DSP plays them as
  // it starts, without the driver writing them
  if (spc.dsp[0x4c]) {
    for (int v=0; v<8; v++) {
      if (!(spc.dsp[0x4c]&(1<<v))) continue;
      VGMEvent e;
      e.time=0;
      e.reg=0x900|v;
      e.val=sampleOf(v);
      log.events.push_back(e);
      keyOns++;
    }
    VGMEvent e;
    e.time=0;
    e.reg=0x84c;
    e.val=spc.dsp[0x4c];
    log.events.push_back(e);
    logI("SPC import: the snapshot starts with key ons pending: %.2X",spc.dsp[0x4c]);
  }
  unsigned long long nextLoopCheck=(unsigned long long)(30.0*clock);
  while (m->cyc<endCyc && !m->stopped) {
    m->step();
    if (refPath && ++refCount>=64) {
      refCount=0;
      m->runDSP();
      int n=m->dsp.sample_count();
      if (n>=4096) {
        refOut.insert(refOut.end(),refBuf.begin(),refBuf.begin()+n);
        m->dsp.set_output(refBuf.data(),(int)refBuf.size());
      }
    }
    while (wi<m->writes.size()) {
      const SPCMachine::DSPWrite& w=m->writes[wi++];
      double t=(double)w.cyc*44100.0/clock;
      // (the global registers changing after the song starts)
      if (initSet && regs[w.reg]!=w.val) {
        int r=w.reg;
        if (r==0x0c || r==0x1c || r==0x2c || r==0x3c || r==0x0d || r==0x4d || r==0x7d || r==0x6c || r==0x2d || r==0x3d || r==0x5d || (r&15)==0x0f) globalChanges[r]++;
      }
      regs[w.reg]=w.val;
      if (w.reg==0x4c && w.val) {
        // (the echo and the volumes as the song sets them: at its first note)
        if (!initSet) {
          memcpy(log.snesInit,regs,128);
          initSet=true;
          // the song's tick: the timer the driver runs on (the first one
          // that is on and ticks between 20 and 500 times a second)
          for (int i=0; i<3; i++) {
            const SPCMachine::Timer& tm=m->timer[i];
            if (!tm.on || tm.lastTick<0) continue;
            double hz=clock/((double)tm.period*(double)tm.target);
            if (hz<20.0 || hz>500.0) continue;
            double per=44100.0/hz;
            double t0=(double)tm.lastTick*44100.0/clock;
            while (t0>=per) t0-=per;
            log.fixedPeriod=per;
            log.fixedT0=t0;
            tickTimer=i;
            fr0=w.fr[i];
            logI("SPC import: the driver ticks on timer %d at %.3f Hz",i,hz);
            break;
          }
        }
        for (int v=0; v<8; v++) {
          if (!(w.val&(1<<v))) continue;
          VGMEvent e;
          e.time=t;
          e.reg=0x900|v;
          e.val=sampleOf(v);
          log.events.push_back(e);
          keyOns++;
        }
      }
      VGMEvent e;
      e.time=t;
      e.reg=0x800|w.reg;
      e.val=w.val;
      log.events.push_back(e);
      if (initSet && tickTimer>=0 && (long long)w.fr[tickTimer]>=fr0) {
        int k=(int)((long long)w.fr[tickTimer]-fr0);
        if (firstTick<0) firstTick=k;
        if ((int)tickKon.size()<=k) tickKon.resize(k+1,0);
        if (w.reg==0x4c && w.val) {
          tickKon[k]=1;
          for (int v=0; v<8; v++) {
            if (!(w.val&(1<<v))) continue;
            const unsigned char* vr=&regs[v<<4];
            int P=(vr[2]|(vr[3]<<8))&0x3fff;
            int note=(P>0)?(int)floor(12.0*log2((double)P/4096.0)+0.5):-999;
            unsigned long long h=1469598103934665603ULL;
            unsigned long long parts[5]={
              (unsigned long long)vr[4],
              (unsigned long long)(note+1000),
              (unsigned long long)((vr[5]<<8)|vr[6]),
              (unsigned long long)vr[7],
              (unsigned long long)(((vr[0]>>4)<<4)|(vr[1]>>4))
            };
            for (int i=0; i<5; i++) h=(h^parts[i])*1099511628211ULL;
            vEvT[v].push_back(k);
            vEvH[v].push_back(h);
          }
        } else if (w.reg==0x5c && w.val) {
          for (int v=0; v<8; v++) {
            if (!(w.val&(1<<v))) continue;
            vEvT[v].push_back(k);
            vEvH[v].push_back(0x5cULL);
          }
        }
      }
    }
    if (m->cyc>=nextLoopCheck) {
      nextLoopCheck=m->cyc+(unsigned long long)(10.0*clock);
      // (the ticks so far, up to the current one, which isn't over yet)
      if (tickTimer>=0) {
        int cur=framesNow();
        // (only whole ticks: not the current one)
        if (findLoop(false,cur)) break;
      }
    }
    if (wi>=4096) {
      m->writes.clear();
      wi=0;
    }
    // the envelopes' levels (some drivers change the envelope during a note:
    // the import follows the level there)
    if (m->cyc>=nextEnvCyc) {
      nextEnvCyc=m->cyc+2048;
      m->runDSP();
      double t=(double)m->cyc*44100.0/clock;
      for (int v=0; v<8; v++) {
        int ex=m->dsp.read((v<<4)|8)&0x7f;
        if (ex==lastEnvx[v]) continue;
        lastEnvx[v]=ex;
        VGMEvent e;
        e.time=t;
        e.reg=0xa00|v;
        e.val=ex;
        log.events.push_back(e);
      }
    }
  }
  if (refPath) {
    m->runDSP();
    int n=m->dsp.sample_count();
    refOut.insert(refOut.end(),refBuf.begin(),refBuf.begin()+n);
    FILE* rf=fopen(refPath,"wb");
    if (rf) {
      unsigned int dataLen=refOut.size()*2;
      unsigned char h[44]={'R','I','F','F',0,0,0,0,'W','A','V','E','f','m','t',' ',16,0,0,0,1,0,2,0,0x00,0x7d,0,0,0x00,0xf4,0x01,0,4,0,16,0,'d','a','t','a',0,0,0,0};
      unsigned int rl=dataLen+36;
      memcpy(h+4,&rl,4);
      memcpy(h+40,&dataLen,4);
      fwrite(h,1,44,rf);
      fwrite(refOut.data(),2,refOut.size(),rf);
      fclose(rf);
    }
  }
  if (m->stopped) warns.push_back("the sound driver stopped (SLEEP/STOP) before the end");
  // (the end of the run: one more look, with one repeat being enough)
  if (loopS<0 && tickTimer>=0) {
    int cur=framesNow();
    findLoop(true,cur);
  }
  double ranSecs=(double)m->cyc/clock;
  // (where a frame starts: when the driver saw its tick)
  auto frameTime=[&](int k) -> double {
    long long i=fr0+k-1;
    if (i<0) i=0;
    const std::vector<unsigned long long>& fc=m->frameCyc[tickTimer];
    if (fc.empty()) return 0;
    if (i>=(long long)fc.size()) i=fc.size()-1;
    return (double)fc[i]*44100.0/clock;
  };
  double loopT=-1, endT=-1;
  if (loopS>=0) {
    loopT=frameTime(loopS);
    endT=frameTime(loopS+loopP);
  }
  // (how far the DSP's sums went: see snesVolGain)
  log.snesPeak=m->dsp.peak_sum;
  delete m;
  if (keyOns==0) {
    err="the sound driver played no notes";
    return false;
  }
  if (loopS>=0) {
    log.loopTime=loopT;
    log.endTime=endT-1.0;
    logI("SPC loop: at %.2fs, %.2fs long (%d frames)",loopT/44100.0,(endT-loopT)/44100.0,loopP);
  } else {
    log.loopTime=-1;
    log.endTime=(double)secs*44100.0;
    if (spc.lengthSec<=0 && ranSecs>=(double)maxSeconds-1.0) {
      warns.push_back(fmt::sprintf("no loop was found within %d seconds, so the song was cut at %d seconds",maxSeconds,secs));
    }
    logI("SPC loop: none, %d seconds",secs);
  }
  // drop what comes after the end
  {
    size_t keep=0;
    for (size_t i=0; i<log.events.size(); i++) {
      if (log.events[i].time<=log.endTime) log.events[keep++]=log.events[i];
    }
    log.events.resize(keep);
  }
  logI("SPC import: %.1f seconds, %d events, %d key ons, %d samples",log.endTime/44100.0,(int)log.events.size(),keyOns,(int)log.snesSamples.size());
  for (auto& g: globalChanges) logI("SPC import: DSP register $%.2X changes %d times after the start",g.first,g.second);
  return true;
}

}

bool DivEngine::loadSPC(unsigned char* file, size_t len) {
  bool success=false;
  warnings="";
  try {
    SPCFile spc;
    String err;
    if (!spcParse(file,len,spc,err)) {
      lastError=err;
      throw VGMInvalidException();
    }
    int maxSeconds=CLAMP(nsfImportOptions.maxSeconds,10,1800);
    VGMNESLog log;
    std::vector<String> warns;
    if (!spcRunTrack(spc,maxSeconds,log,warns,err)) {
      lastError=err;
      throw VGMInvalidException();
    }

    // the voices' volumes up: many drivers keep them low and the main volume
    // high, so the song shows volumes like 06 or 1F. on the DSP a voice's
    // volume times the main volume (and times the echo volume, for the echo)
    // is what is heard: the voices' volumes times a whole number, and the
    // main and echo volumes divided by it, sound the same. the number is
    // the largest that keeps every voice volume within 127, the DSP's sums
    // (which it clips at 16 bits before the main volume) within their range,
    // and the main and echo volumes the song uses dividing with less than 2%
    // of difference
    {
      int maxVol=0;
      std::vector<int> outVols;
      auto voiceVol=[&](int reg, int val) {
        if ((reg&0x0f)<=1 && (reg&0x7f)<0x80) maxVol=std::max(maxVol,abs((int)(signed char)val));
        if (reg==0x0c || reg==0x1c || reg==0x2c || reg==0x3c) outVols.push_back((signed char)val);
      };
      for (int i=0; i<128; i++) voiceVol(i,log.snesInit[i]);
      for (const VGMEvent& e: log.events) {
        if (e.reg>=0x800 && e.reg<0x880) voiceVol(e.reg-0x800,e.val);
      }
      int gain=1;
      for (int g=8; g>=2; g--) {
        if (maxVol*g>127) continue;
        if ((long long)log.snesPeak*g>32000) continue;
        bool ok=true;
        for (int v: outVols) {
          if (v==0) continue;
          double q=(double)v/(double)g;
          if (fabs(q-round(q))*g>fabs((double)v)*0.02) {
            ok=false;
            break;
          }
        }
        if (!ok) continue;
        gain=g;
        break;
      }
      log.snesVolGain=gain;
      logI("SPC import: loudest voice volume %d, DSP peak %d: voice volumes x%d",maxVol,log.snesPeak,gain);
    }

    DivSong ds;
    ds.version=DIV_VERSION_VGM;
    ds.compatFlags.resetArpPhaseOnNewNote=true;
    ds.systemLen=1;
    ds.system[0]=DIV_SYSTEM_SNES;
    ds.systemVol[0]=1.0f;
    ds.systemPan[0]=0.0f;
    // (the key ons as the driver writes them, without Furnace's anti-click
    // delay)
    ds.systemFlags[0].set("antiClick",false);
    // the echo and the main volume as the song sets them up
    {
      const unsigned char* r=log.snesInit;
      bool echoOn=!(r[0x6c]&0x20) && !getenv("FURNACE_SPC_NOECHO");
      ds.systemFlags[0].set("echo",echoOn);
      ds.systemFlags[0].set("echoDelay",r[0x7d]&15);
      ds.systemFlags[0].set("echoFeedback",(int)(signed char)r[0x0d]);
      for (int i=0; i<8; i++) ds.systemFlags[0].set(fmt::sprintf("echoFilter%d",i).c_str(),(int)(signed char)r[(i<<4)|0x0f]);
      const double gain=(double)log.snesVolGain;
      auto byGain=[&](unsigned char b) -> int {
        return std::max(-128,std::min(127,(int)round((double)(signed char)b/gain)));
      };
      ds.systemFlags[0].set("echoVolL",byGain(r[0x2c]));
      ds.systemFlags[0].set("echoVolR",byGain(r[0x3c]));
      ds.systemFlags[0].set("echoMask",(int)r[0x4d]);
      // the main volume goes to the chip's own register (MVOL), as it is,
      // sign and all: the voices keep their volumes, and the echo takes them
      // before the main volume, like on the real chip (scaling the voices
      // instead would turn the echo down with it)
      ds.systemFlags[0].set("dryVolL",byGain(r[0x0c]));
      ds.systemFlags[0].set("dryVolR",byGain(r[0x1c]));
    }
    ds.initDefaultSystemChans();
    ds.name=spc.title;
    ds.author=spc.artist;
    ds.category=spc.game;
    ds.notes=fmt::sprintf("Imported from SPC.\nGame: %s\nTitle: %s\nArtist: %s\nDumper: %s\n%s",spc.game,spc.title,spc.artist,spc.dumper,spc.comment);

    VGMNESContext ctx;
    ctx.gridMode=vgmPickGrid(log,ctx,ds);
    if (!vgmConvertNES(log,ctx,ds,ds.subsong[0],warns,err)) {
      lastError=err;
      throw VGMInvalidException();
    }
    for (String& w: warns) {
      logW("SPC import: %s",w);
      addWarning(w);
    }

    ds.insLen=(int)ds.ins.size();
    ds.sampleLen=(int)ds.sample.size();
    ds.recalcChans();

    if (active) quitDispatch();
    BUSY_BEGIN_SOFT;
    saveLock.lock();
    song.unload();
    song=ds;
    hasLoadedSomething=true;
    changeSong(0);
    saveLock.unlock();
    BUSY_END;
    if (active) {
      initDispatch();
      BUSY_BEGIN;
      renderSamples();
      reset();
      BUSY_END;
    }
    success=true;
  } catch (VGMInvalidException& e) {
  }
  delete[] file;
  return success;
}

// ---------------------------------------------------------------------------
// GBS import
//
// a GBS is the sound program of a Game Boy game: its code and data, with an
// init routine (called with the song number) and a play routine (called at
// the rate of the timer, or of the screen). the program is run on a Game Boy
// CPU (LR35902) and every write to the sound registers is logged with its
// time, like an NSF.
// ---------------------------------------------------------------------------

namespace {

struct GBSFile {
  int songs=0, first=1;
  unsigned short loadAddr=0, initAddr=0, playAddr=0, sp=0;
  unsigned char tma=0, tac=0;
  String title, author, copyright;
  std::vector<unsigned char> data;
};

String gbsString(const unsigned char* p, size_t n) {
  String r;
  for (size_t i=0; i<n && p[i]; i++) r+=(char)p[i];
  return r;
}

bool gbsParse(const unsigned char* f, size_t len, GBSFile& gbs, String& err) {
  if (len<0x71 || memcmp(f,"GBS",3)!=0) {
    err="not a GBS file";
    return false;
  }
  if (f[3]!=1) {
    err=fmt::sprintf("unsupported GBS version %d",f[3]);
    return false;
  }
  gbs.songs=f[4];
  gbs.first=f[5];
  if (gbs.songs<1) gbs.songs=1;
  if (gbs.first<1 || gbs.first>gbs.songs) gbs.first=1;
  gbs.loadAddr=f[6]|(f[7]<<8);
  gbs.initAddr=f[8]|(f[9]<<8);
  gbs.playAddr=f[10]|(f[11]<<8);
  gbs.sp=f[12]|(f[13]<<8);
  gbs.tma=f[14];
  gbs.tac=f[15];
  gbs.title=gbsString(f+0x10,32);
  gbs.author=gbsString(f+0x30,32);
  gbs.copyright=gbsString(f+0x50,32);
  gbs.data.assign(f+0x70,f+len);
  return true;
}

struct GBSMachine {
  std::vector<unsigned char> rom;
  unsigned char mem[0x10000];
  int bank=1;
  unsigned short loadAddr=0;
  unsigned char a=0, f=0, b=0, c=0, d=0, e=0, h=0, l=0;
  unsigned short sp=0xfffe, pc=0;
  bool ime=false, halted=false, bad=false;
  long long cycles=0;
  double clock=4194304.0;
  // the call being run: when it started (samples) and at which cycle
  double callTime=0;
  long long callCycles=0;
  VGMNESLog* log=NULL;
  VGMGBApu apu;
  bool pageDirty[256];
  std::vector<std::pair<unsigned short,unsigned char>> frameWrites;
  static const unsigned short SENTINEL=0xf00d;

  GBSMachine() {
    memset(mem,0,sizeof(mem));
    memset(pageDirty,0,sizeof(pageDirty));
  }
  double now() const {
    return callTime+(double)(cycles-callCycles)*44100.0/clock;
  }
  unsigned char rd(unsigned short addr) {
    if (addr<0x4000) return (addr<rom.size())?rom[addr]:0xff;
    if (addr<0x8000) {
      size_t o=(size_t)bank*0x4000+(addr-0x4000);
      return (o<rom.size())?rom[o]:0xff;
    }
    if (addr>=0xe000 && addr<0xfe00) addr-=0x2000;
    if (addr>=0xff00 && addr<0xff80) {
      switch (addr) {
        case 0xff04: return (unsigned char)(cycles>>8);
        case 0xff44: return (unsigned char)((cycles/456)%154);
        case 0xff26: {
          apu.advance(now());
          unsigned char r=apu.powered?0x80:0;
          for (int i=0; i<4; i++) if (apu.ch[i].on && apu.dac(i)) r|=1<<i;
          return r|0x70;
        }
      }
      return mem[addr];
    }
    return mem[addr];
  }
  void wr(unsigned short addr, unsigned char v) {
    if (addr<0x8000) {
      // (the bank of $4000-$7FFF)
      if (addr>=0x2000 && addr<0x4000) {
        bank=v?v:1;
        if ((size_t)bank*0x4000>=rom.size()) bank=1;
      }
      return;
    }
    if (addr>=0xe000 && addr<0xfe00) addr-=0x2000;
    mem[addr]=v;
    pageDirty[addr>>8]=true;
    if (addr>=0xff10 && addr<0xff40) {
      double t=now();
      apu.advance(t);
      apu.write(addr&0xff,v);
      if (log!=NULL) {
        VGMEvent ev;
        ev.time=t;
        ev.reg=0xc00|(addr&0xff);
        ev.val=v;
        log->events.push_back(ev);
      }
      frameWrites.push_back(std::make_pair(addr,v));
    }
  }
  unsigned short bc() const { return (b<<8)|c; }
  unsigned short de() const { return (d<<8)|e; }
  unsigned short hl() const { return (h<<8)|l; }
  void setBC(unsigned short v) { b=v>>8; c=v&0xff; }
  void setDE(unsigned short v) { d=v>>8; e=v&0xff; }
  void setHL(unsigned short v) { h=v>>8; l=v&0xff; }
  unsigned char fetch() { return rd(pc++); }
  unsigned short fetch16() {
    unsigned char lo=fetch();
    return lo|(fetch()<<8);
  }
  void push(unsigned short v) {
    wr(--sp,v>>8);
    wr(--sp,v&0xff);
  }
  unsigned short pop() {
    unsigned char lo=rd(sp++);
    return lo|(rd(sp++)<<8);
  }
  enum { FZ=0x80, FN=0x40, FH=0x20, FC=0x10 };
  unsigned char getR(int i) {
    switch (i) {
      case 0: return b;
      case 1: return c;
      case 2: return d;
      case 3: return e;
      case 4: return h;
      case 5: return l;
      case 6: return rd(hl());
      default: return a;
    }
  }
  void setR(int i, unsigned char v) {
    switch (i) {
      case 0: b=v; break;
      case 1: c=v; break;
      case 2: d=v; break;
      case 3: e=v; break;
      case 4: h=v; break;
      case 5: l=v; break;
      case 6: wr(hl(),v); break;
      default: a=v; break;
    }
  }
  void alu(int op, unsigned char v) {
    int r;
    switch (op) {
      case 0: case 1: { // ADD, ADC
        int cy=(op==1 && (f&FC))?1:0;
        r=a+v+cy;
        f=((r&0xff)?0:FZ)|(((a&15)+(v&15)+cy)>15?FH:0)|(r>0xff?FC:0);
        a=r;
        break;
      }
      case 2: case 3: case 7: { // SUB, SBC, CP
        int cy=(op==3 && (f&FC))?1:0;
        r=a-v-cy;
        f=((r&0xff)?0:FZ)|FN|(((a&15)-(v&15)-cy)<0?FH:0)|(r<0?FC:0);
        if (op!=7) a=r;
        break;
      }
      case 4: a&=v; f=(a?0:FZ)|FH; break;
      case 5: a^=v; f=a?0:FZ; break;
      case 6: a|=v; f=a?0:FZ; break;
    }
  }
  bool cond(int cc) {
    switch (cc&3) {
      case 0: return !(f&FZ);
      case 1: return f&FZ;
      case 2: return !(f&FC);
      default: return f&FC;
    }
  }
  unsigned short getRR(int i) {
    switch (i) {
      case 0: return bc();
      case 1: return de();
      case 2: return hl();
      default: return sp;
    }
  }
  void setRR(int i, unsigned short v) {
    switch (i) {
      case 0: setBC(v); break;
      case 1: setDE(v); break;
      case 2: setHL(v); break;
      default: sp=v; break;
    }
  }
  int cb() {
    unsigned char op=fetch();
    int r=op&7;
    int x=op>>6, y=(op>>3)&7;
    unsigned char v=getR(r);
    int cyc=(r==6)?16:8;
    if (x==0) {
      unsigned char res=v;
      bool cy=false;
      switch (y) {
        case 0: cy=v&0x80; res=(v<<1)|(v>>7); break; // RLC
        case 1: cy=v&1; res=(v>>1)|(v<<7); break; // RRC
        case 2: cy=v&0x80; res=(v<<1)|((f&FC)?1:0); break; // RL
        case 3: cy=v&1; res=(v>>1)|((f&FC)?0x80:0); break; // RR
        case 4: cy=v&0x80; res=v<<1; break; // SLA
        case 5: cy=v&1; res=(v>>1)|(v&0x80); break; // SRA
        case 6: res=(v<<4)|(v>>4); break; // SWAP
        case 7: cy=v&1; res=v>>1; break; // SRL
      }
      f=(res?0:FZ)|(cy?FC:0);
      setR(r,res);
    } else if (x==1) {
      f=(f&FC)|FH|((v&(1<<y))?0:FZ);
      if (r==6) cyc=12;
    } else if (x==2) {
      setR(r,v&~(1<<y));
    } else {
      setR(r,v|(1<<y));
    }
    return cyc;
  }
  // one instruction; returns its cycles
  int step() {
    if (halted) return 4;
    unsigned char op=fetch();
    switch (op) {
      case 0x00: return 4;
      case 0x01: case 0x11: case 0x21: case 0x31:
        setRR(op>>4,fetch16());
        return 12;
      case 0x02: wr(bc(),a); return 8;
      case 0x12: wr(de(),a); return 8;
      case 0x22: wr(hl(),a); setHL(hl()+1); return 8;
      case 0x32: wr(hl(),a); setHL(hl()-1); return 8;
      case 0x0a: a=rd(bc()); return 8;
      case 0x1a: a=rd(de()); return 8;
      case 0x2a: a=rd(hl()); setHL(hl()+1); return 8;
      case 0x3a: a=rd(hl()); setHL(hl()-1); return 8;
      case 0x03: case 0x13: case 0x23: case 0x33:
        setRR(op>>4,getRR(op>>4)+1);
        return 8;
      case 0x0b: case 0x1b: case 0x2b: case 0x3b:
        setRR(op>>4,getRR(op>>4)-1);
        return 8;
      case 0x04: case 0x0c: case 0x14: case 0x1c: case 0x24: case 0x2c: case 0x34: case 0x3c: {
        int r=(op>>3)&7;
        unsigned char v=getR(r)+1;
        setR(r,v);
        f=(f&FC)|(v?0:FZ)|((v&15)==0?FH:0);
        return (r==6)?12:4;
      }
      case 0x05: case 0x0d: case 0x15: case 0x1d: case 0x25: case 0x2d: case 0x35: case 0x3d: {
        int r=(op>>3)&7;
        unsigned char v=getR(r)-1;
        setR(r,v);
        f=(f&FC)|FN|(v?0:FZ)|((v&15)==15?FH:0);
        return (r==6)?12:4;
      }
      case 0x06: case 0x0e: case 0x16: case 0x1e: case 0x26: case 0x2e: case 0x36: case 0x3e: {
        int r=(op>>3)&7;
        setR(r,fetch());
        return (r==6)?12:8;
      }
      case 0x07: { bool cy=a&0x80; a=(a<<1)|(a>>7); f=cy?FC:0; return 4; }
      case 0x0f: { bool cy=a&1; a=(a>>1)|(a<<7); f=cy?FC:0; return 4; }
      case 0x17: { bool cy=a&0x80; a=(a<<1)|((f&FC)?1:0); f=cy?FC:0; return 4; }
      case 0x1f: { bool cy=a&1; a=(a>>1)|((f&FC)?0x80:0); f=cy?FC:0; return 4; }
      case 0x08: {
        unsigned short ad=fetch16();
        wr(ad,sp&0xff);
        wr(ad+1,sp>>8);
        return 20;
      }
      case 0x09: case 0x19: case 0x29: case 0x39: {
        unsigned int x=hl(), y=getRR(op>>4);
        unsigned int r=x+y;
        f=(f&FZ)|(((x&0xfff)+(y&0xfff))>0xfff?FH:0)|(r>0xffff?FC:0);
        setHL(r);
        return 8;
      }
      case 0x10: fetch(); bad=true; return 4; // STOP
      case 0x18: { signed char o=(signed char)fetch(); pc+=o; return 12; }
      case 0x20: case 0x28: case 0x30: case 0x38: {
        signed char o=(signed char)fetch();
        if (cond((op>>3)&3)) {
          pc+=o;
          return 12;
        }
        return 8;
      }
      case 0x27: {
        int v=a;
        if (!(f&FN)) {
          if ((f&FC) || v>0x99) { v+=0x60; f|=FC; }
          if ((f&FH) || (v&15)>9) v+=6;
        } else {
          if (f&FC) v-=0x60;
          if (f&FH) v-=6;
        }
        a=v;
        f=(f&(FN|FC))|(a?0:FZ);
        return 4;
      }
      case 0x2f: a=~a; f|=FN|FH; return 4;
      case 0x37: f=(f&FZ)|FC; return 4;
      case 0x3f: f=(f&FZ)|((f&FC)?0:FC); return 4;
      case 0x76: halted=true; return 4;
      case 0xc0: case 0xc8: case 0xd0: case 0xd8:
        if (cond((op>>3)&3)) {
          pc=pop();
          return 20;
        }
        return 8;
      case 0xc9: pc=pop(); return 16;
      case 0xd9: pc=pop(); ime=true; return 16;
      case 0xc1: case 0xd1: case 0xe1: {
        unsigned short v=pop();
        setRR((op>>4)-0xc,v);
        return 12;
      }
      case 0xf1: { unsigned short v=pop(); a=v>>8; f=v&0xf0; return 12; }
      case 0xc5: case 0xd5: case 0xe5:
        push(getRR((op>>4)-0xc));
        return 16;
      case 0xf5: push((a<<8)|f); return 16;
      case 0xc2: case 0xca: case 0xd2: case 0xda: {
        unsigned short ad=fetch16();
        if (cond((op>>3)&3)) {
          pc=ad;
          return 16;
        }
        return 12;
      }
      case 0xc3: pc=fetch16(); return 16;
      case 0xe9: pc=hl(); return 4;
      case 0xc4: case 0xcc: case 0xd4: case 0xdc: {
        unsigned short ad=fetch16();
        if (cond((op>>3)&3)) {
          push(pc);
          pc=ad;
          return 24;
        }
        return 12;
      }
      case 0xcd: {
        unsigned short ad=fetch16();
        push(pc);
        pc=ad;
        return 24;
      }
      case 0xc6: case 0xce: case 0xd6: case 0xde: case 0xe6: case 0xee: case 0xf6: case 0xfe:
        alu((op>>3)&7,fetch());
        return 8;
      case 0xc7: case 0xcf: case 0xd7: case 0xdf: case 0xe7: case 0xef: case 0xf7: case 0xff:
        // (a GBS's RST vectors are at its load address)
        push(pc);
        pc=loadAddr+(op&0x38);
        return 16;
      case 0xcb: return cb();
      case 0xe0: wr(0xff00|fetch(),a); return 12;
      case 0xf0: a=rd(0xff00|fetch()); return 12;
      case 0xe2: wr(0xff00|c,a); return 8;
      case 0xf2: a=rd(0xff00|c); return 8;
      case 0xe8: case 0xf8: {
        signed char o=(signed char)fetch();
        unsigned int r=(sp+o)&0xffff;
        f=(((sp&15)+(o&15))>15?FH:0)|(((sp&0xff)+(o&0xff))>0xff?FC:0);
        if (op==0xe8) {
          sp=r;
          return 16;
        }
        setHL(r);
        return 12;
      }
      case 0xf9: sp=hl(); return 8;
      case 0xea: wr(fetch16(),a); return 16;
      case 0xfa: a=rd(fetch16()); return 16;
      case 0xf3: ime=false; return 4;
      case 0xfb: ime=true; return 4;
      default:
        if (op>=0x40 && op<0x80) {
          int dst=(op>>3)&7, src=op&7;
          setR(dst,getR(src));
          return (dst==6 || src==6)?8:4;
        }
        if (op>=0x80 && op<0xc0) {
          int src=op&7;
          alu((op>>3)&7,getR(src));
          return (src==6)?8:4;
        }
        // (an opcode the CPU doesn't have: it locks up)
        bad=true;
        return 4;
    }
  }
  // call a routine: it returns to the sentinel. false if it doesn't in time
  bool call(unsigned short addr, long long maxCycles) {
    push(SENTINEL);
    pc=addr;
    halted=false;
    bad=false;
    long long end=cycles+maxCycles;
    while (cycles<end) {
      if (pc==SENTINEL) return true;
      if (halted || bad) break;
      cycles+=step();
    }
    if (pc==SENTINEL) return true;
    // (a routine that waits with HALT counts as done)
    return halted;
  }
};

bool gbsRunTrack(const GBSFile& gbs, int track, int maxSeconds, VGMNESLog& log, std::vector<String>& warns, String& err) {
  GBSMachine* m=new GBSMachine;
  m->loadAddr=gbs.loadAddr;
  m->rom.assign((size_t)gbs.loadAddr+gbs.data.size(),0);
  memcpy(&m->rom[gbs.loadAddr],gbs.data.data(),gbs.data.size());
  // (whole banks)
  m->rom.resize((m->rom.size()+0x3fff)&~(size_t)0x3fff,0);
  if (m->rom.size()<0x8000) m->rom.resize(0x8000,0);
  m->log=&log;
  // the play rate: the timer (with the GBC's double speed), or the screen
  const bool dbl=gbs.tac&0x80;
  if (dbl) m->clock=8388608.0;
  double hz;
  if (gbs.tac&4) {
    static const double div[4]={1024.0,16.0,64.0,256.0};
    hz=(4194304.0/div[gbs.tac&3])/(256.0-(double)gbs.tma);
    if (dbl) hz*=2.0;
  } else {
    hz=4194304.0/70224.0;
  }
  double frameCycles=m->clock/hz;
  double period=44100.0/hz;
  log.hasNES=false;
  log.hasGB=true;
  log.clock=m->clock;
  log.fixedPeriod=period;
  log.fixedT0=0;
  log.headerRate=(unsigned int)round(hz);
  logI("GBS track %d: %s, %.3f Hz",track,(gbs.tac&4)?"timer":"screen",hz);

  // the state before init: the sound on, everything panned to both sides
  m->wr(0xff26,0x80);
  m->wr(0xff25,0xff);
  m->wr(0xff24,0x77);
  m->mem[0xff06]=gbs.tma;
  m->mem[0xff07]=gbs.tac;
  m->mem[0xffff]=(gbs.tac&4)?4:1;
  m->sp=gbs.sp;
  m->a=(unsigned char)(track-1);
  m->callTime=0;
  m->callCycles=m->cycles;
  if (!m->call(gbs.initAddr,(long long)(m->clock*4))) {
    warns.push_back("the init routine of this GBS did not return; the result may be wrong");
  }
  // (whatever init wrote counts as the start of the first frame)
  for (VGMEvent& e: log.events) e.time=0;

  int maxFrames=(int)((double)maxSeconds*hz);
  if (maxFrames<60) maxFrames=60;
  std::map<unsigned long long,int> stateSeen;
  std::vector<unsigned long long> writeHash, soundHash;
  std::vector<std::vector<unsigned long long>> chanHash(4);
  std::vector<bool> audible;
  memset(m->pageDirty,0,sizeof(m->pageDirty));
  int loopStart=-1, loopEnd=-1, stuckAt=-1;
  int playFails=0;
  long long frameStart=m->cycles;
  for (int fr=0; fr<maxFrames; fr++) {
    m->callTime=(double)fr*period;
    m->cycles=frameStart+(long long)((double)fr*frameCycles);
    m->callCycles=m->cycles;
    m->frameWrites.clear();
    m->sp=gbs.sp;
    if (!m->call(gbs.playAddr,(long long)(frameCycles*2))) playFails++;
    m->apu.advance(m->callTime+period);
    // (what the channels are doing: their registers, and whether they play)
    unsigned char sb[0x40];
    for (int i=0; i<0x30; i++) sb[i]=m->apu.regs[i];
    for (int i=0; i<16; i++) sb[0x30+i]=m->apu.waveRam[i];
    bool aud=false;
    for (int i=0; i<4; i++) if (m->apu.ch[i].on && m->apu.dac(i)) aud=true;
    audible.push_back(aud);
    soundHash.push_back(nsfHash(sb,0x40));
    unsigned long long wh=1469598103934665603ULL;
    for (auto& w: m->frameWrites) {
      unsigned char b2[3]={(unsigned char)w.first,(unsigned char)(w.first>>8),w.second};
      wh=nsfHash(b2,3,wh);
    }
    writeHash.push_back(wh);
    for (int i=0; i<4; i++) {
      unsigned char cs[7];
      memcpy(cs,&m->apu.regs[i*5],5);
      cs[5]=m->apu.ch[i].on;
      cs[6]=(unsigned char)m->apu.ch[i].vol;
      unsigned long long chh=nsfHash(cs,7);
      if (i==2) chh=nsfHash(m->apu.waveRam,16,chh);
      chanHash[i].push_back(chh);
    }
    // the program's state: the memory it writes, and the sound registers
    unsigned long long h=nsfHash(sb,0x40);
    for (int pg=0x80; pg<256; pg++) {
      if (!m->pageDirty[pg]) continue;
      // (the I/O page: only the high RAM)
      if (pg==0xff) {
        h=nsfHash(&m->mem[0xff80],0x7f,h);
        continue;
      }
      h=nsfHash(&m->mem[pg<<8],256,h);
    }
    auto seen=stateSeen.find(h);
    if (seen!=stateSeen.end()) {
      int j=seen->second;
      if (fr-j<30) {
        stuckAt=j;
      } else {
        loopStart=j;
        loopEnd=fr;
      }
      break;
    }
    stateSeen[h]=fr;
  }
  delete m;
  if (playFails>0) warns.push_back(fmt::sprintf("the player did not return in time %d times",playFails));
  int frames=(int)writeHash.size();
  if (loopStart<0 && stuckAt<0) {
    double sc=0;
    if (vgmSoundLoop(chanHash,frames,loopStart,loopEnd,sc)) {
      logI("GBS loop by channels: period %d frames, %.1f%% differing",loopEnd-loopStart,sc*100.0);
    }
  }
  if (loopStart>=0) {
    int L=loopEnd-loopStart;
    for (int dv=8; dv>=2; dv--) {
      if (L%dv!=0 || L/dv<60) continue;
      int Ld=L/dv;
      bool same=true;
      for (int i=loopStart; i+Ld<loopEnd && same; i++) {
        if (soundHash[i]!=soundHash[i+Ld] && writeHash[i]!=writeHash[i+Ld]) same=false;
      }
      if (same) {
        L=Ld;
        loopEnd=loopStart+L;
        break;
      }
    }
    while (loopStart>0 && (soundHash[loopStart-1]==soundHash[loopStart-1+L] || writeHash[loopStart-1]==writeHash[loopStart-1+L])) {
      loopStart--;
      loopEnd--;
    }
    bool anySound=false;
    for (int fr=loopStart; fr<loopEnd && fr<(int)audible.size(); fr++) {
      if (audible[fr]) {
        anySound=true;
        break;
      }
    }
    if (!anySound) {
      stuckAt=loopStart;
      loopStart=-1;
    }
  }
  int endFrame=frames;
  if (loopStart>=0) {
    log.loopTime=(double)loopStart*period;
    endFrame=loopEnd;
    logI("GBS track %d: loop at %.2fs, %.2fs long",track,(double)loopStart/hz,(double)(loopEnd-loopStart)/hz);
  } else {
    log.loopTime=-1;
    int last=-1;
    for (int fr=(int)audible.size()-1; fr>=0; fr--) {
      if (audible[fr]) {
        last=fr;
        break;
      }
    }
    if (stuckAt>=0 && stuckAt>last) last=stuckAt;
    if (last<0) {
      err="this track makes no sound";
      return false;
    }
    endFrame=std::min(frames,last+(int)(hz*2));
    if (stuckAt<0 && frames>=maxFrames) {
      warns.push_back(fmt::sprintf("track %d: no loop was found within %d seconds, so it was cut there",track,maxSeconds));
    }
    logI("GBS track %d: no loop, %.2fs",track,(double)endFrame/hz);
  }
  log.endTime=(double)endFrame*period-1.0;
  while (!log.events.empty() && log.events.back().time>log.endTime) log.events.pop_back();
  // (some channel panned to one side only, at some point)
  for (VGMEvent& e: log.events) {
    if (e.reg==0xc25 && ((e.val>>4)&15)!=(e.val&15)) {
      log.gbStereo=true;
      break;
    }
  }
  return true;
}

}

bool gbsInfo(const unsigned char* file, size_t len, DivNSFInfo& info) {
  GBSFile gbs;
  String err;
  if (!gbsParse(file,len,gbs,err)) return false;
  info.title=gbs.title;
  info.artist=gbs.author;
  info.copyright=gbs.copyright;
  info.songs=gbs.songs;
  info.start=gbs.first;
  info.pal=false;
  info.chips="Game Boy";
  return true;
}

bool DivEngine::loadGBS(unsigned char* file, size_t len) {
  bool success=false;
  warnings="";
  try {
    GBSFile gbs;
    String err;
    if (!gbsParse(file,len,gbs,err)) {
      lastError=err;
      throw VGMInvalidException();
    }
    std::vector<int> tracks=nsfImportOptions.tracks;
    if (tracks.empty()) tracks.push_back(gbs.first);
    int maxSeconds=CLAMP(nsfImportOptions.maxSeconds,10,1800);

    DivSong ds;
    ds.version=DIV_VERSION_NSF;
    ds.compatFlags.resetArpPhaseOnNewNote=true;
    ds.systemLen=1;
    ds.system[0]=DIV_SYSTEM_GB;
    ds.systemVol[0]=1.0f;
    ds.systemPan[0]=0.0f;
    ds.initDefaultSystemChans();
    ds.name=gbs.title;
    ds.author=gbs.author;
    ds.category=gbs.title;
    ds.notes=fmt::sprintf("Imported from GBS.\nGame: %s\nArtist: %s\nCopyright: %s",gbs.title,gbs.author,gbs.copyright);

    VGMNESContext ctx;
    int imported=0;
    for (size_t ti=0; ti<tracks.size(); ti++) {
      int track=tracks[ti];
      if (track<1 || track>gbs.songs) continue;
      int tracksLeft=(int)(tracks.size()-ti);
      ctx.shareLimit=(tracks.size()>1)?std::max(24,(256-(int)ds.ins.size())/tracksLeft):0;
      VGMNESLog log;
      std::vector<String> warns;
      if (!gbsRunTrack(gbs,track,maxSeconds,log,warns,err)) {
        addWarning(fmt::sprintf("track %d: %s",track,err));
        continue;
      }
      DivSubSong* sub;
      if (imported==0) {
        sub=ds.subsong[0];
      } else {
        sub=new DivSubSong;
        ds.subsong.push_back(sub);
      }
      String trackName=fmt::sprintf("Track %d",track);
      sub->name=trackName;
      ctx.gridMode=vgmPickGrid(log,ctx,ds);
      if (!vgmConvertNES(log,ctx,ds,sub,warns,err)) {
        addWarning(fmt::sprintf("track %d: %s",track,err));
        if (imported>0) {
          ds.subsong.pop_back();
          delete sub;
        }
        continue;
      }
      for (String& w: warns) {
        String full=(tracks.size()>1)?fmt::sprintf("%s: %s",trackName,w):w;
        addWarning(full);
      }
      imported++;
    }
    if (imported==0) {
      lastError=err.empty()?String("no track could be imported"):err;
      throw VGMInvalidException();
    }
    if (tracks.size()==1 && gbs.songs>1) ds.name=fmt::sprintf("%s - Track %d",gbs.title,tracks[0]);

    ds.insLen=(int)ds.ins.size();
    ds.sampleLen=(int)ds.sample.size();
    ds.waveLen=(int)ds.wave.size();
    ds.recalcChans();

    if (active) quitDispatch();
    BUSY_BEGIN_SOFT;
    saveLock.lock();
    song.unload();
    song=ds;
    hasLoadedSomething=true;
    changeSong(0);
    saveLock.unlock();
    BUSY_END;
    if (active) {
      initDispatch();
      BUSY_BEGIN;
      renderSamples();
      reset();
      BUSY_END;
    }
    success=true;
  } catch (VGMInvalidException& e) {
  }
  delete[] file;
  return success;
}
