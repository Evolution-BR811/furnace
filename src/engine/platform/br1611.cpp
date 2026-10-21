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

// BR1611 (Ompreza 2) - por Evolution BR811

#include "br1611.h"
#include "../engine.h"
#include <math.h>

// o BR1611 roda com o clock do RS16 (10,272 MHz) / 321 = 32.000 Hz
#define BR1611_CLOCK 10272000
#define BR1611_RATE_DIVIDER 321
// frequencia de um canal WMS: f = W * 32000 / 2^21   (W = fnum << bloco)
// (a tabela de afinacao do Furnace trabalha com a frequencia x32, por isso o /32)
#define BR1611_TONE_DIVIDER (2097152.0*BR1611_RATE_DIVIDER/32.0)
// PCM: W = 17146 toca o sample a 32.000 Hz
#define BR1611_PCM_DIVIDER (17146.0*BR1611_RATE_DIVIDER)
#define BR1611_MAX_W 0x3ffff
#define BR1611_ROM_SIZE (4*1024*1024)

// formatos de PCM do chip
#define FMT_PCM16 0
#define FMT_PCM8 1
#define FMT_PCM4 2
#define FMT_CP 3

const char* regCheatSheetBR1611[]={
  "LFO", "000",
  "KeyOn", "001",
  "PCMMode", "002",
  "WaveSlot", "004",
  "WaveIndex", "005",
  "WaveData", "006",
  "FreqLow_x", "008",
  "Block_FreqHigh_x", "010",
  "Volume_x", "018",
  "FMS_Pan_x", "020",
  "PCM6_Start", "030",
  "PCM6_Loop", "033",
  "PCM6_End", "036",
  "PCM6_Control", "039",
  "PCM7_Start", "040",
  "PCM7_Loop", "043",
  "PCM7_End", "046",
  "PCM7_Control", "049",
  "DT_MULT_xy", "100",
  "TL_xy", "120",
  "RS_AR_xy", "140",
  "D1R_xy", "160",
  "D2R_xy", "180",
  "SL_RR_xy", "1A0",
  "SSG_xy", "1C0",
  NULL
};

const char** DivPlatformBR1611::getRegisterSheet() {
  return regCheatSheetBR1611;
}

void DivPlatformBR1611::writeReg(int page, int addr, unsigned char val) {
  regPool[((page&1)<<8)|(addr&0xff)]=val;
  chip.write(page,addr,val);
}

// manda uma wavetable da musica para o slot de onda do operador (256 pontos de 8 bits)
void DivPlatformBR1611::uploadWave(int ch, int op, int waveIndex) {
  chan[ch].waves[op]=waveIndex;
  unsigned char data[256];
  DivWavetable* w=NULL;
  if (parent->song.waveLen>0) w=parent->getWave(waveIndex);
  if (w==NULL || w->len<1 || w->max<1) {
    // sem wavetable na musica: usa uma senoide
    for (int i=0; i<256; i++) {
      data[i]=(unsigned char)round(128.0+127.0*sin(2.0*3.14159265358979323846*(double)i/256.0));
    }
  } else {
    for (int i=0; i<256; i++) {
      // reamostra a onda para 256 pontos (interpolacao linear, em ciclo)
      double pos=(double)i*(double)w->len/256.0;
      int a=(int)pos;
      double frac=pos-(double)a;
      int b=(a+1)%w->len;
      a%=w->len;
      double v=(double)w->data[a]*(1.0-frac)+(double)w->data[b]*frac;
      int out=(int)round(v*255.0/(double)w->max);
      if (out<0) out=0;
      if (out>255) out=255;
      data[i]=out;
    }
  }
  writeReg(0,0x04,ch*4+op);
  writeReg(0,0x05,0);
  for (int i=0; i<256; i++) {
    writeReg(0,0x06,data[i]);
  }
}

// escreve os 4 operadores do canal (pagina 1)
void DivPlatformBR1611::commitOps(int ch) {
  for (int j=0; j<4; j++) {
    DivInstrumentFM::Operator& op=chan[ch].state.op[j];
    int i=j*8+ch;
    writeReg(1,0x00+i,((op.dt&7)<<4)|(op.mult&15));
    writeReg(1,0x20+i,(isMuted[ch] || !op.enable)?127:(op.tl&127));
    writeReg(1,0x40+i,((op.rs&3)<<6)|(op.ar&31));
    writeReg(1,0x60+i,op.dr&31);
    writeReg(1,0x80+i,op.d2r&31);
    writeReg(1,0xa0+i,((op.sl&15)<<4)|(op.rr&15));
    writeReg(1,0xc0+i,op.ssgEnv&15);
  }
}

// volume, vibrato e posicao estereo
void DivPlatformBR1611::commitChannel(int ch) {
  int vol=isMuted[ch]?0:chan[ch].outVol;
  if (vol<0) vol=0;
  if (vol>127) vol=127;
  int pan=chan[ch].pan;
  if (pan<0 || pan>4) pan=2;
  writeReg(0,0x18+ch,vol);
  writeReg(0,0x20+ch,((chan[ch].fms&7)<<4)|pan);
}

// converte W (fnum << bloco) para fnum de 11 bits + bloco de 3 bits
void DivPlatformBR1611::commitFreq(int ch) {
  int w=chan[ch].freq;
  if (w<0) w=0;
  if (w>BR1611_MAX_W) w=BR1611_MAX_W;
  int block=0;
  while (w>2047 && block<7) {
    w>>=1;
    block++;
  }
  if (w>2047) w=2047;
  writeReg(0,0x08+ch,w&0xff);
  writeReg(0,0x10+ch,(block<<3)|((w>>8)&7));
}

// programa os enderecos do sample (canais 6 e 7)
void DivPlatformBR1611::setupPcm(int ch) {
  int base=(ch==5)?0x30:0x40;
  int s=chan[ch].sample;
  unsigned int start=0, loop=0, end=0;
  unsigned char ctrl=0;
  if (s>=0 && s<parent->song.sampleLen && sampleLoaded[s]) {
    DivSample* smp=parent->getSample(s);
    start=sampleOff[s];
    end=sampleEnd[s];
    loop=sampleLoopOff[s];
    ctrl=sampleFmt[s]&3;
    if (smp->isLoopable()) ctrl|=4;
  }
  for (int k=0; k<3; k++) {
    writeReg(0,base+k,(start>>(k*8))&0xff);
    writeReg(0,base+3+k,(loop>>(k*8))&0xff);
    writeReg(0,base+6+k,(end>>(k*8))&0xff);
  }
  writeReg(0,base+9,ctrl);
}

void DivPlatformBR1611::keyOn(int ch) {
  int mask=0;
  if (chan[ch].pcm) {
    mask=1; // no PCM, o operador 1 dispara o sample e da o envelope
  } else {
    for (int j=0; j<4; j++) {
      if (chan[ch].state.op[j].enable) mask|=(1<<j);
    }
  }
  // solta antes, para o envelope recomecar do ataque
  writeReg(0,0x01,ch);
  writeReg(0,0x01,(mask<<4)|ch);
}

void DivPlatformBR1611::keyOff(int ch) {
  writeReg(0,0x01,ch);
}

void DivPlatformBR1611::acquire(short** buf, size_t len) {
  for (int i=0; i<7; i++) {
    oscBuf[i]->begin(len);
  }
  for (size_t h=0; h<len; h++) {
    int16_t l=0, r=0;
    chip.render(&l,&r,1);
    buf[0][h]=l;
    buf[1][h]=r;
    for (int i=0; i<7; i++) {
      int o=chip.chanOut(i)*4;
      if (o<-32768) o=-32768;
      if (o>32767) o=32767;
      oscBuf[i]->putSample(h,o);
    }
  }
  for (int i=0; i<7; i++) {
    oscBuf[i]->end(len);
  }
}

void DivPlatformBR1611::tick(bool sysTick) {
  for (int i=0; i<7; i++) {
    chan[i].std.next();
    bool opsDirty=false;
    bool chanDirty=false;

    if (chan[i].std.vol.had) {
      chan[i].outVol=VOL_SCALE_LINEAR(chan[i].vol&127,MIN(127,chan[i].std.vol.val),127);
      chanDirty=true;
    }
    if (NEW_ARP_STRAT) {
      chan[i].handleArp();
    } else if (chan[i].std.arp.had && !chan[i].rawFreq) {
      if (!chan[i].inPorta) {
        chan[i].baseFreq=chan[i].calcBaseFreq(parent->calcArp(chan[i].note,chan[i].std.arp.val));
      }
      chan[i].freqChanged=true;
    }
    if (chan[i].std.pitch.had) {
      if (chan[i].std.pitch.mode) {
        chan[i].pitch2+=chan[i].std.pitch.val;
        CLAMP_VAR(chan[i].pitch2,-32768,32767);
      } else {
        chan[i].pitch2=chan[i].std.pitch.val;
      }
      chan[i].freqChanged=true;
    }
    if (chan[i].std.panL.had) {
      chan[i].pan=chan[i].std.panL.val;
      if (chan[i].pan<0 || chan[i].pan>4) chan[i].pan=2;
      chanDirty=true;
    }
    if (chan[i].std.fms.had) {
      chan[i].fms=chan[i].std.fms.val&7;
      chanDirty=true;
    }
    if (chan[i].std.phaseReset.had) {
      if (chan[i].std.phaseReset.val==1 && chan[i].active) {
        chan[i].keyOn=true;
      }
    }

    // macros dos operadores
    for (int j=0; j<4; j++) {
      DivInstrumentFM::Operator& op=chan[i].state.op[j];
      DivMacroInt::IntOp& m=chan[i].std.op[j];
      if (m.tl.had) { op.tl=m.tl.val; opsDirty=true; }
      if (m.ar.had) { op.ar=m.ar.val; opsDirty=true; }
      if (m.dr.had) { op.dr=m.dr.val; opsDirty=true; }
      if (m.d2r.had) { op.d2r=m.d2r.val; opsDirty=true; }
      if (m.rr.had) { op.rr=m.rr.val; opsDirty=true; }
      if (m.sl.had) { op.sl=m.sl.val; opsDirty=true; }
      if (m.rs.had) { op.rs=m.rs.val; opsDirty=true; }
      if (m.mult.had) { op.mult=m.mult.val; opsDirty=true; }
      if (m.dt.had) { op.dt=m.dt.val; opsDirty=true; }
      if (m.ssg.had) { op.ssgEnv=m.ssg.val; opsDirty=true; }
      if (m.ws.had && !chan[i].pcm) {
        if (chan[i].waves[j]!=m.ws.val) uploadWave(i,j,m.ws.val);
      }
    }

    if (opsDirty) commitOps(i);
    if (chanDirty) commitChannel(i);

    if (chan[i].freqChanged || chan[i].keyOn || chan[i].keyOff) {
      chan[i].freq=chan[i].calcFreq();
      if (chan[i].freq<0) chan[i].freq=0;
      if (chan[i].freq>BR1611_MAX_W) chan[i].freq=BR1611_MAX_W;
      commitFreq(i);
      if (chan[i].keyOn) {
        if (chan[i].pcm) setupPcm(i);
        keyOn(i);
      }
      if (chan[i].keyOff) keyOff(i);
      chan[i].keyOn=false;
      chan[i].keyOff=false;
      chan[i].freqChanged=false;
    }
  }
}

int DivPlatformBR1611::dispatch(DivCommand c) {
  switch (c.cmd) {
    case DIV_CMD_NOTE_ON: {
      DivInstrument* ins=parent->getIns(chan[c.chan].ins,DIV_INS_BR1611);
      // canais 6 e 7: instrumento com "Use sample" (ou instrumento de sample) = PCM
      bool wantPcm=(c.chan>=5 && (ins->type==DIV_INS_AMIGA || ins->amiga.useSample));
      if (wantPcm!=chan[c.chan].pcm) {
        chan[c.chan].pcm=wantPcm;
        chan[c.chan].insChanged=true;
        int mode=0;
        if (chan[5].pcm) mode|=1;
        if (chan[6].pcm) mode|=2;
        writeReg(0,0x02,mode);
      }
      if (chan[c.chan].pcm) {
        if (c.value!=DIV_NOTE_NULL) {
          chan[c.chan].sample=ins->amiga.getSample(c.value);
          chan[c.chan].pitchTable=samplePitchTable.get(chan[c.chan].sample);
          chan[c.chan].sampleNote=c.value;
          c.value=ins->amiga.getFreq(c.value);
          chan[c.chan].sampleNoteDelta=c.value-chan[c.chan].sampleNote;
        }
      } else {
        chan[c.chan].sample=-1;
        chan[c.chan].sampleNote=DIV_NOTE_NULL;
        chan[c.chan].sampleNoteDelta=0;
        chan[c.chan].pitchTable=&pitchTable;
      }

      if (chan[c.chan].insChanged) {
        if (ins->type==DIV_INS_BR1611) {
          chan[c.chan].state=ins->fm;
          if (chan[c.chan].pcm) {
            // PCM: envelope proprio (aba Sample), gravado nos registradores do operador 1
            DivInstrumentFM::Operator& op=chan[c.chan].state.op[0];
            op.ar=ins->br1611.pcmAR;
            op.dr=ins->br1611.pcmD1R;
            op.sl=ins->br1611.pcmSL;
            op.d2r=ins->br1611.pcmD2R;
            op.rr=ins->br1611.pcmRR;
            op.tl=ins->br1611.pcmTL;
            op.rs=0;
            op.ssgEnv=0;
            op.enable=true;
          }
        } else {
          // instrumento de sample sem envelope proprio: volume cheio enquanto a nota durar
          chan[c.chan].state=DivInstrumentFM();
          DivInstrumentFM::Operator& op=chan[c.chan].state.op[0];
          op.tl=0; op.ar=31; op.dr=0; op.sl=0; op.d2r=0; op.rr=15; op.ssgEnv=0; op.rs=0;
        }
        chan[c.chan].fms=chan[c.chan].state.fms&7;
        if (!chan[c.chan].pcm) {
          for (int j=0; j<4; j++) {
            uploadWave(c.chan,j,(ins->type==DIV_INS_BR1611)?ins->br1611.wave[j]:0);
          }
        }
        commitOps(c.chan);
        chan[c.chan].insChanged=false;
      }

      if (c.value!=DIV_NOTE_NULL) {
        chan[c.chan].baseFreq=chan[c.chan].calcBaseFreq(c.value);
        chan[c.chan].freqChanged=true;
        chan[c.chan].note=c.value;
      }
      chan[c.chan].active=true;
      chan[c.chan].keyOn=true;
      chan[c.chan].macroInit(ins);
      if (!parent->song.compatFlags.brokenOutVol && !chan[c.chan].std.vol.will) {
        chan[c.chan].outVol=chan[c.chan].vol;
      }
      commitChannel(c.chan);
      break;
    }
    case DIV_CMD_NOTE_OFF:
      chan[c.chan].active=false;
      chan[c.chan].keyOff=true;
      chan[c.chan].macroInit(NULL);
      break;
    case DIV_CMD_NOTE_OFF_ENV:
    case DIV_CMD_ENV_RELEASE:
      chan[c.chan].std.release();
      break;
    case DIV_CMD_INSTRUMENT:
      if (chan[c.chan].ins!=c.value || c.value2==1) {
        chan[c.chan].insChanged=true;
        chan[c.chan].ins=c.value;
      }
      break;
    case DIV_CMD_VOLUME:
      if (chan[c.chan].vol!=c.value) {
        chan[c.chan].vol=c.value;
        if (!chan[c.chan].std.vol.has) {
          chan[c.chan].outVol=c.value;
        }
        commitChannel(c.chan);
      }
      break;
    case DIV_CMD_GET_VOLUME:
      return chan[c.chan].vol;
      break;
    case DIV_CMD_PANNING: {
      // efeito 08xy: converte esquerda/direita para as 5 posicoes do BR1611
      int l=c.value, r=c.value2;
      int pan=2;
      if (l>0 && r==0) {
        pan=0;
      } else if (l==0 && r>0) {
        pan=4;
      } else if (l>r) {
        pan=1;
      } else if (r>l) {
        pan=3;
      }
      chan[c.chan].pan=pan;
      commitChannel(c.chan);
      break;
    }
    case DIV_CMD_FM_LFO: // efeito 10xx: 0 desliga; 1 a 8 liga com a velocidade 0 a 7
      if (c.value<=0) {
        lfo=0;
      } else {
        lfo=8|((c.value-1)&7);
      }
      writeReg(0,0x00,lfo);
      break;
    case DIV_CMD_FM_FMS: // efeito 11xx: profundidade do vibrato (0-7)
      chan[c.chan].fms=c.value&7;
      commitChannel(c.chan);
      break;
    case DIV_CMD_PITCH:
      chan[c.chan].pitch=c.value;
      chan[c.chan].freqChanged=true;
      break;
    case DIV_CMD_NOTE_PORTA: {
      int destFreq=chan[c.chan].calcBaseFreq(c.value2+chan[c.chan].sampleNoteDelta);
      bool return2=false;
      if (destFreq>chan[c.chan].baseFreq) {
        chan[c.chan].baseFreq+=c.value;
        if (chan[c.chan].baseFreq>=destFreq) {
          chan[c.chan].baseFreq=destFreq;
          return2=true;
        }
      } else {
        chan[c.chan].baseFreq-=c.value;
        if (chan[c.chan].baseFreq<=destFreq) {
          chan[c.chan].baseFreq=destFreq;
          return2=true;
        }
      }
      chan[c.chan].freqChanged=true;
      if (return2) {
        chan[c.chan].inPorta=false;
        return 2;
      }
      break;
    }
    case DIV_CMD_LEGATO:
      chan[c.chan].baseFreq=chan[c.chan].calcBaseFreq(c.value+chan[c.chan].sampleNoteDelta+((HACKY_LEGATO_MESS)?(chan[c.chan].std.arp.val):(0)));
      chan[c.chan].freqChanged=true;
      chan[c.chan].note=c.value;
      break;
    case DIV_CMD_PRE_PORTA:
      if (chan[c.chan].active && c.value2) {
        if (parent->song.compatFlags.resetMacroOnPorta) chan[c.chan].macroInit(parent->getIns(chan[c.chan].ins,DIV_INS_BR1611));
      }
      if (!chan[c.chan].inPorta && c.value && !parent->song.compatFlags.brokenPortaArp && chan[c.chan].std.arp.will && !NEW_ARP_STRAT) chan[c.chan].baseFreq=chan[c.chan].calcBaseFreq(chan[c.chan].note);
      chan[c.chan].inPorta=c.value;
      break;
    case DIV_CMD_GET_VOLMAX:
      return 127;
      break;
    case DIV_CMD_MACRO_OFF:
      chan[c.chan].std.mask(c.value,true);
      break;
    case DIV_CMD_MACRO_ON:
      chan[c.chan].std.mask(c.value,false);
      break;
    case DIV_CMD_MACRO_RESTART:
      chan[c.chan].std.restart(c.value);
      break;
    default:
      break;
  }
  return 1;
}

void DivPlatformBR1611::muteChannel(int ch, bool mute) {
  isMuted[ch]=mute;
  commitOps(ch);
  commitChannel(ch);
}

bool DivPlatformBR1611::keyOffAffectsArp(int ch) {
  return false;
}

void DivPlatformBR1611::forceIns() {
  writeReg(0,0x00,lfo);
  int mode=0;
  if (chan[5].pcm) mode|=1;
  if (chan[6].pcm) mode|=2;
  writeReg(0,0x02,mode);
  for (int i=0; i<7; i++) {
    chan[i].insChanged=true;
    chan[i].freqChanged=true;
    for (int j=0; j<4; j++) {
      if (chan[i].waves[j]>=0) uploadWave(i,j,chan[i].waves[j]);
    }
    commitOps(i);
    commitChannel(i);
  }
}

void DivPlatformBR1611::notifyInsChange(int ins) {
  for (int i=0; i<7; i++) {
    if (chan[i].ins==ins) {
      chan[i].insChanged=true;
    }
  }
}

void DivPlatformBR1611::notifyWaveChange(int wave) {
  for (int i=0; i<7; i++) {
    if (chan[i].pcm) continue;
    for (int j=0; j<4; j++) {
      if (chan[i].waves[j]==wave) uploadWave(i,j,wave);
    }
  }
}

SharedChannel* DivPlatformBR1611::getChanState(int ch) {
  return &chan[ch];
}

DivMacroInt* DivPlatformBR1611::getChanMacroInt(int ch) {
  return &chan[ch].std;
}

DivDispatchOscBuffer* DivPlatformBR1611::getOscBuffer(int ch) {
  return oscBuf[ch];
}

unsigned char* DivPlatformBR1611::getRegisterPool() {
  return regPool;
}

int DivPlatformBR1611::getRegisterPoolSize() {
  return 512;
}

void DivPlatformBR1611::reset() {
  memset(regPool,0,512);
  chip.reset();
  chip.setSampleMemory(sampleMem,BR1611_ROM_SIZE);
  lfo=0;
  for (int i=0; i<7; i++) {
    chan[i]=Channel(parent->song.compatFlags.linearPitch);
    chan[i].pitchTable=&pitchTable;
    chan[i].std.setEngine(parent);
    chan[i].vol=127;
    chan[i].outVol=127;
    commitChannel(i);
  }
}

int DivPlatformBR1611::getOutputCount() {
  return 2;
}

void DivPlatformBR1611::notifyInsDeletion(void* ins) {
  for (int i=0; i<7; i++) {
    chan[i].std.notifyInsDeletion((DivInstrument*)ins);
  }
}

void DivPlatformBR1611::notifyPitchTable(int sample) {
  pitchTable.init(parent->song.tuning,chipClock,BR1611_TONE_DIVIDER,BR1611_MAX_W,false,parent->song.compatFlags.linearPitch);
  samplePitchTable.update<Channel>(chan,7,parent->song.tuning,chipClock,BR1611_PCM_DIVIDER,BR1611_MAX_W,false,parent->song.compatFlags.linearPitch,sample);
}

unsigned int DivPlatformBR1611::getMaxFreq(int ch) {
  return BR1611_MAX_W;
}

void DivPlatformBR1611::setFlags(const DivConfig& flags) {
  // clock fixo: o mesmo da CPU RS16 do Ompreza 2 (10,272 MHz)
  chipClock=BR1611_CLOCK;
  rate=chipClock/BR1611_RATE_DIVIDER;
  for (int i=0; i<7; i++) {
    oscBuf[i]->setRate(rate);
  }
  notifyPitchTable();
}

void DivPlatformBR1611::poke(unsigned int addr, unsigned short val) {
  if (addr>=512) return;
  writeReg(addr>>8,addr&0xff,val);
}

void DivPlatformBR1611::poke(std::vector<DivRegWrite>& wlist) {
  for (DivRegWrite& i: wlist) poke(i.addr,i.val);
}

int DivPlatformBR1611::init(DivEngine* p, int channels, int sugRate, const DivConfig& flags) {
  parent=p;
  dumpWrites=false;
  skipRegisterWrites=false;
  for (int i=0; i<7; i++) {
    isMuted[i]=false;
    oscBuf[i]=new DivDispatchOscBuffer;
  }
  samplePitchTable.init(parent);
  sampleMemLen=0;
  memset(sampleMem,0,BR1611_ROM_SIZE);
  setFlags(flags);
  reset();
  return 7;
}

void DivPlatformBR1611::quit() {
  samplePitchTable.destroy<Channel>(chan,7);
  for (int i=0; i<7; i++) {
    delete oscBuf[i];
  }
}

const void* DivPlatformBR1611::getSampleMem(int index) {
  return index==0?sampleMem:NULL;
}

size_t DivPlatformBR1611::getSampleMemCapacity(int index) {
  return index==0?BR1611_ROM_SIZE:0;
}

size_t DivPlatformBR1611::getSampleMemUsage(int index) {
  return index==0?sampleMemLen:0;
}

bool DivPlatformBR1611::isSampleLoaded(int index, int sample) {
  if (index!=0) return false;
  if (sample<0 || sample>=getMaxSamples(index)) return false;
  return sampleLoaded[sample];
}

const DivMemoryComposition* DivPlatformBR1611::getMemCompo(int index) {
  if (index!=0) return NULL;
  return &memCompo;
}

// monta a ROM do cartucho: cada sample no formato escolhido no editor de samples
// (16 bits, 8 bits, 4 bits ou BRR, que o BR1611 toca como CP)
void DivPlatformBR1611::renderSamples(int sysID) {
  memset(sampleMem,0,BR1611_ROM_SIZE);
  memset(sampleLoaded,0,32768*sizeof(bool));

  memCompo=DivMemoryComposition();
  memCompo.name="Cartridge ROM";

  size_t memPos=0;
  for (int i=0; i<parent->song.sampleLen; i++) {
    DivSample* s=parent->song.sample[i];
    sampleOff[i]=0;
    sampleEnd[i]=0;
    sampleLoopOff[i]=0;
    sampleFmt[i]=FMT_PCM16;
    if (!s->renderOn[0][sysID]) continue;

    unsigned char fmt=FMT_PCM16;
    switch (s->depth) {
      case DIV_SAMPLE_DEPTH_8BIT: fmt=FMT_PCM8; break;
      case DIV_SAMPLE_DEPTH_4BIT: fmt=FMT_PCM4; break;
      case DIV_SAMPLE_DEPTH_BRR: fmt=FMT_CP; break;
      default: fmt=FMT_PCM16; break;
    }

    const unsigned char* src=NULL;
    size_t len=0;
    size_t loopByte=0;
    bool extraBlock=false;
    switch (fmt) {
      case FMT_PCM16:
        src=(const unsigned char*)s->data16;
        len=(size_t)s->samples*2;
        if (s->isLoopable()) {
          len=(size_t)s->loopEnd*2;
          loopByte=(size_t)s->loopStart*2;
        }
        break;
      case FMT_PCM8:
        src=(const unsigned char*)s->data8;
        len=s->samples;
        if (s->isLoopable()) {
          len=s->loopEnd;
          loopByte=s->loopStart;
        }
        break;
      case FMT_PCM4:
        src=s->data4;
        len=(s->samples+1)/2;
        if (s->isLoopable()) {
          len=(s->loopEnd+1)/2;
          loopByte=s->loopStart/2;
        }
        break;
      case FMT_CP:
        src=s->dataBRR;
        // o Furnace coloca um bloco extra no começo quando converte com loop
        extraBlock=(s->loop && s->depth!=DIV_SAMPLE_DEPTH_BRR);
        len=s->lengthBRR+(extraBlock?9:0);
        len=(len/9)*9;
        if (s->isLoopable()) {
          loopByte=(extraBlock?9:0)+(s->loopStart/16)*9;
        }
        break;
    }
    if (src==NULL || len==0) continue;
    if (memPos+len>BR1611_ROM_SIZE) {
      logW("BR1611: out of cartridge ROM for sample %d!",i);
      break;
    }

    if (fmt==FMT_PCM16) {
      // little-endian, com sinal
      for (size_t k=0; k+1<len; k+=2) {
        short v=s->data16[k>>1];
        sampleMem[memPos+k]=v&0xff;
        sampleMem[memPos+k+1]=(v>>8)&0xff;
      }
    } else if (fmt==FMT_PCM4) {
      // o Furnace guarda 4 bits sem sinal; o BR1611 usa com sinal (inverte o bit mais alto)
      for (size_t k=0; k<len; k++) {
        sampleMem[memPos+k]=src[k]^0x88;
      }
    } else {
      memcpy(&sampleMem[memPos],src,len);
    }

    sampleOff[i]=memPos;
    sampleEnd[i]=memPos+len;
    sampleLoopOff[i]=memPos+loopByte;
    sampleFmt[i]=fmt;
    sampleLoaded[i]=true;
    memCompo.entries.push_back(DivMemoryEntry(DIV_MEMORY_SAMPLE,"Sample",i,memPos,memPos+len));
    memPos+=len;
    // alinha em 2 bytes
    if (memPos&1) memPos++;
  }
  sampleMemLen=memPos;

  memCompo.capacity=BR1611_ROM_SIZE;
  memCompo.used=sampleMemLen;
}

DivPlatformBR1611::DivPlatformBR1611() {
  sampleMem=new unsigned char[BR1611_ROM_SIZE];
  sampleOff=new unsigned int[32768];
  sampleEnd=new unsigned int[32768];
  sampleLoopOff=new unsigned int[32768];
  sampleFmt=new unsigned char[32768];
  sampleLoaded=new bool[32768];
  memset(sampleOff,0,32768*sizeof(unsigned int));
  memset(sampleEnd,0,32768*sizeof(unsigned int));
  memset(sampleLoopOff,0,32768*sizeof(unsigned int));
  memset(sampleFmt,0,32768);
  memset(sampleLoaded,0,32768*sizeof(bool));
}

DivPlatformBR1611::~DivPlatformBR1611() {
  delete[] sampleMem;
  delete[] sampleOff;
  delete[] sampleEnd;
  delete[] sampleLoopOff;
  delete[] sampleFmt;
  delete[] sampleLoaded;
}
