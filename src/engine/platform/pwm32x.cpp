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

#include "pwm32x.h"
#include "../engine.h"
#include <math.h>

// the mixer steps each sample with a 16.16 counter, once per PWM output
#define CHIP_FREQBASE 65536

// the SH2's clock, which the PWM counts (NTSC and PAL 32X)
static const int pwmClocks[2]={23011361,22801467};

void DivPlatformPWM32X::acquire(short** buf, size_t len) {
  if (chan==NULL) {
    for (size_t h=0; h<len; h++) {
      buf[0][h]=0;
      buf[1][h]=0;
    }
    return;
  }

  for (int i=0; i<chans; i++) {
    oscBuf[i].begin(len);
  }

  // the PWM's range: 0 to the cycle register, silence in the middle
  const double half=(double)cycle*0.5;

  for (size_t h=0; h<len; h++) {
    int mixL=0, mixR=0;
    for (int i=0; i<chans; i++) {
      Channel& ch=chan[i];
      int output=0;
      if (ch.active && ch.sample>=0 && ch.sample<parent->song.sampleLen) {
        DivSample* s=parent->getSample(ch.sample);
        if (s->samples>0 && ch.audPos>=0 && ch.audPos<(int)s->samples) {
          // (the value at the position: no interpolation, like the drivers)
          output=s->data16[ch.audPos];
        }
        // the step to the next output
        ch.audSub+=ch.freq;
        while (ch.audSub>=0x10000 && ch.sample>=0) {
          ch.audSub-=0x10000;
          ch.audPos+=ch.audDir?-1:1;
          if (s->samples<=0) {
            ch.sample=-1;
            break;
          }
          if (s->isLoopable()) {
            if (ch.audDir) {
              if (ch.audPos<s->loopStart) {
                switch (s->loopMode) {
                  case DIV_SAMPLE_LOOP_BACKWARD:
                    ch.audPos=s->loopEnd-1-(s->loopStart-ch.audPos);
                    break;
                  default:
                    ch.audPos=s->loopStart+(s->loopStart-ch.audPos);
                    ch.audDir=false;
                    break;
                }
              }
            } else if (ch.audPos>=s->loopEnd) {
              switch (s->loopMode) {
                case DIV_SAMPLE_LOOP_FORWARD:
                  ch.audPos=(ch.audPos+s->loopStart)-s->loopEnd;
                  break;
                default:
                  ch.audPos=s->loopEnd-1-(ch.audPos-s->loopEnd);
                  ch.audDir=true;
                  break;
              }
            }
          } else if (ch.audPos>=(int)s->samples || ch.audPos<0) {
            ch.sample=-1;
          }
        }
      }
      if (isMuted[i]) {
        output=0;
      } else {
        output=(output*MIN(255,ch.outVol)*MIN(ch.envVol,64))/(255*64);
      }
      oscBuf[i].putSample(h,output>>1);
      mixL+=(output*ch.panL)/255;
      mixR+=(output*ch.panR)/255;
    }

    // the sum goes to the PWM: a whole number of clocks in its range
    // (clipped at its ends), which is what is heard
    int pwmL=(int)round(half+(double)mixL*half/32767.0);
    int pwmR=(int)round(half+(double)mixR*half/32767.0);
    if (pwmL<0) pwmL=0;
    if (pwmL>cycle) pwmL=cycle;
    if (pwmR<0) pwmR=0;
    if (pwmR>cycle) pwmR=cycle;
    lastL=pwmL;
    lastR=pwmR;
    double outL=((double)pwmL-half)*32767.0/half;
    double outR=((double)pwmR-half)*32767.0/half;
    if (outL<-32768.0) outL=-32768.0;
    if (outL>32767.0) outL=32767.0;
    if (outR<-32768.0) outR=-32768.0;
    if (outR>32767.0) outR=32767.0;
    buf[0][h]=(short)outL;
    buf[1][h]=(short)outR;
  }

  for (int i=0; i<chans; i++) {
    oscBuf[i].end(len);
  }
}

void DivPlatformPWM32X::tick(bool sysTick) {
  for (int i=0; i<chans; i++) {
    chan[i].std.next();
    if (chan[i].std.vol.had) {
      chan[i].envVol=chan[i].std.vol.val;
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
      chan[i].panL=(chan[i].std.panL.val&0x7f)*2;
    }
    if (chan[i].std.panR.had) {
      chan[i].panR=(chan[i].std.panR.val&0x7f)*2;
    }
    if (chan[i].std.phaseReset.had) {
      if (chan[i].std.phaseReset.val==1) {
        chan[i].audDir=false;
        chan[i].audPos=0;
      }
    }
    if (chan[i].freqChanged || chan[i].keyOn || chan[i].keyOff) {
      chan[i].freq=chan[i].calcFreq();
      if (!chan[i].rawFreq) {
        if (chan[i].freq>16777215) chan[i].freq=16777215;
      }
      if (chan[i].keyOn) {
        if (!chan[i].std.vol.had) {
          chan[i].envVol=64;
        }
        chan[i].keyOn=false;
      }
      if (chan[i].keyOff) {
        chan[i].keyOff=false;
      }
      chan[i].freqChanged=false;
    }
    chan[i].outVol=chan[i].vol;
  }
}

int DivPlatformPWM32X::dispatch(DivCommand c) {
  if (c.chan>=chans) return 0;
  switch (c.cmd) {
    case DIV_CMD_NOTE_ON: {
      DivInstrument* ins=parent->getIns(chan[c.chan].ins,DIV_INS_AMIGA);
      if (c.value!=DIV_NOTE_NULL) {
        chan[c.chan].sample=ins->amiga.getSample(c.value);
        chan[c.chan].pitchTable=samplePitchTable.get(chan[c.chan].sample);
        chan[c.chan].sampleNote=c.value;
        c.value=ins->amiga.getFreq(c.value);
        chan[c.chan].sampleNoteDelta=c.value-chan[c.chan].sampleNote;
      } else if (chan[c.chan].sampleNote!=DIV_NOTE_NULL) {
        chan[c.chan].sample=ins->amiga.getSample(chan[c.chan].sampleNote);
        chan[c.chan].pitchTable=samplePitchTable.get(chan[c.chan].sample);
        c.value=ins->amiga.getFreq(chan[c.chan].sampleNote);
      }
      if (c.value!=DIV_NOTE_NULL) {
        chan[c.chan].baseFreq=chan[c.chan].calcBaseFreq(c.value);
      }
      if (chan[c.chan].sample<0 || chan[c.chan].sample>=parent->song.sampleLen) {
        chan[c.chan].sample=-1;
        chan[c.chan].sampleNote=DIV_NOTE_NULL;
        chan[c.chan].sampleNoteDelta=0;
      }
      if (chan[c.chan].setPos) {
        chan[c.chan].setPos=false;
      } else {
        chan[c.chan].audDir=false;
        chan[c.chan].audPos=0;
      }
      chan[c.chan].audSub=0;
      if (c.value!=DIV_NOTE_NULL) {
        chan[c.chan].freqChanged=true;
        chan[c.chan].note=c.value;
      }
      chan[c.chan].active=true;
      chan[c.chan].keyOn=true;
      chan[c.chan].macroInit(ins);
      if (!parent->song.compatFlags.brokenOutVol && !chan[c.chan].std.vol.will) {
        chan[c.chan].envVol=64;
      }
      chan[c.chan].insChanged=false;
      break;
    }
    case DIV_CMD_NOTE_OFF:
      chan[c.chan].sample=-1;
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
        chan[c.chan].ins=c.value;
        chan[c.chan].insChanged=true;
      }
      break;
    case DIV_CMD_VOLUME:
      if (chan[c.chan].vol!=c.value) {
        chan[c.chan].vol=c.value;
        chan[c.chan].outVol=c.value;
        if (!chan[c.chan].std.vol.has) {
          chan[c.chan].envVol=64;
        }
      }
      break;
    case DIV_CMD_GET_VOLUME:
      return chan[c.chan].vol;
      break;
    case DIV_CMD_PANNING:
      chan[c.chan].panL=c.value;
      chan[c.chan].panR=c.value2;
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
    case DIV_CMD_LEGATO: {
      chan[c.chan].baseFreq=chan[c.chan].calcBaseFreq(c.value+chan[c.chan].sampleNoteDelta+((HACKY_LEGATO_MESS)?(chan[c.chan].std.arp.val):(0)));
      chan[c.chan].freqChanged=true;
      chan[c.chan].note=c.value;
      break;
    }
    case DIV_CMD_PRE_PORTA:
      if (chan[c.chan].active && c.value2) {
        if (parent->song.compatFlags.resetMacroOnPorta) chan[c.chan].macroInit(parent->getIns(chan[c.chan].ins,DIV_INS_AMIGA));
      }
      chan[c.chan].inPorta=c.value;
      break;
    case DIV_CMD_SAMPLE_POS:
      chan[c.chan].audPos=c.value;
      chan[c.chan].setPos=true;
      break;
    case DIV_CMD_GET_VOLMAX:
      return 255;
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

void DivPlatformPWM32X::muteChannel(int ch, bool mute) {
  if (ch>=chans) return;
  isMuted[ch]=mute;
}

void DivPlatformPWM32X::forceIns() {
  for (int i=0; i<chans; i++) {
    chan[i].insChanged=true;
    chan[i].freqChanged=true;
    chan[i].audDir=false;
    chan[i].audPos=0;
    chan[i].sample=-1;
  }
}

SharedChannel* DivPlatformPWM32X::getChanState(int ch) {
  if (ch>=chans) return NULL;
  return &chan[ch];
}

DivDispatchOscBuffer* DivPlatformPWM32X::getOscBuffer(int ch) {
  if (ch>=chans) return NULL;
  return &oscBuf[ch];
}

void DivPlatformPWM32X::reset() {
  for (int i=0; i<chans; i++) {
    chan[i]=DivPlatformPWM32X::Channel(parent->song.compatFlags.linearPitch);
    chan[i].pitchTable=samplePitchTable.get(-1);
    chan[i].std.setEngine(parent);
  }
  lastL=lastR=cycle/2;
}

int DivPlatformPWM32X::getOutputCount() {
  return 2;
}

bool DivPlatformPWM32X::hasSoftPan(int ch) {
  return true;
}

DivMacroInt* DivPlatformPWM32X::getChanMacroInt(int ch) {
  if (ch>=chans) return NULL;
  return &chan[ch].std;
}

unsigned short DivPlatformPWM32X::getPan(int ch) {
  if (ch>=chans) return 0;
  return (chan[ch].panL<<8)|chan[ch].panR;
}

DivSamplePos DivPlatformPWM32X::getSamplePos(int ch) {
  if (ch>=chans) return DivSamplePos();
  return DivSamplePos(
    chan[ch].sample,
    chan[ch].audPos,
    chan[ch].freq
  );
}

void DivPlatformPWM32X::notifyInsChange(int ins) {
  for (int i=0; i<chans; i++) {
    if (chan[i].ins==ins) {
      chan[i].insChanged=true;
    }
  }
}

void DivPlatformPWM32X::notifyInsDeletion(void* ins) {
  for (int i=0; i<chans; i++) {
    chan[i].std.notifyInsDeletion((DivInstrument*)ins);
  }
}

void DivPlatformPWM32X::notifyPitchTable(int sample) {
  // (the step counts PWM outputs: the pitch goes by the PWM's rate)
  samplePitchTable.update<Channel>(chan,chans,parent->song.tuning,rate,CHIP_FREQBASE,0xffffff,false,parent->song.compatFlags.linearPitch,sample);
}

unsigned int DivPlatformPWM32X::getMaxFreq(int ch) {
  return 0xffffff;
}

void DivPlatformPWM32X::setFlags(const DivConfig& flags) {
  int clockSel=flags.getInt("clockSel",0);
  if (clockSel<0 || clockSel>1) clockSel=0;
  chipClock=pwmClocks[clockSel];
  // the cycle register: a PWM output every cycle-1 clocks
  cycle=flags.getInt("cycle",1047);
  if (cycle<256) cycle=256;
  if (cycle>4095) cycle=4095;
  rate=(int)round((double)chipClock/(double)(cycle-1));
  if (rate<1000) rate=1000;
  for (int i=0; i<chans; i++) {
    oscBuf[i].setRate(rate);
  }
  notifyPitchTable();
}

int DivPlatformPWM32X::init(DivEngine* p, int channels, int sugRate, const DivConfig& flags) {
  parent=p;
  samplePitchTable.init(parent);
  dumpWrites=false;
  skipRegisterWrites=false;
  oscBuf=new DivDispatchOscBuffer[channels];
  chan=new Channel[channels];
  isMuted=new bool[channels];
  chans=channels;
  for (int i=0; i<channels; i++) {
    isMuted[i]=false;
  }
  setFlags(flags);
  reset();
  return 1;
}

void DivPlatformPWM32X::quit() {
  samplePitchTable.destroy<Channel>(chan,chans);
  delete[] chan;
  delete[] isMuted;
  delete[] oscBuf;
  chan=NULL;
  isMuted=NULL;
  oscBuf=NULL;
  chans=0;
}
