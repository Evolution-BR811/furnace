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

#ifndef _PWM32X_H
#define _PWM32X_H

#include "../dispatch.h"

// the Sega 32X's PWM: two pulse width DACs (left and right) that output one
// value every (cycle-1) clocks of the 32X's SH2 clock, from 0 to the cycle
// register (the middle is silence). the 32X has no sound channels: its SH2
// mixes them in software and writes the sum to the PWM. the channels here are
// that mixer, done like the drivers did it: samples stepped at the PWM's rate
// without interpolation, a volume and a left/right level each, summed and
// clipped to the PWM's range
class DivPlatformPWM32X: public DivDispatch {
  struct Channel: public SharedChannel {
    bool audDir;
    int audPos;
    unsigned int audSub;
    int sample;
    int panL, panR;
    bool setPos;
    int envVol;
    Channel(bool linear=true):
      SharedChannel(255,linear),
      audDir(false),
      audPos(0),
      audSub(0),
      sample(-1),
      panL(255),
      panR(255),
      setPos(false),
      envVol(64) {}
  };
  Channel* chan;
  bool* isMuted;
  int chans;
  DivDispatchOscBuffer* oscBuf;
  DivPitchTableManager samplePitchTable;
  // the cycle register (12 bits) and what the outputs last held
  int cycle;
  int lastL, lastR;

  friend void putDispatchChip(void*,int);
  friend void putDispatchChan(void*,int,int);

  public:
    void acquire(short** buf, size_t len);
    int dispatch(DivCommand c);
    SharedChannel* getChanState(int chan);
    DivDispatchOscBuffer* getOscBuffer(int chan);
    void reset();
    void forceIns();
    void tick(bool sysTick=true);
    void muteChannel(int ch, bool mute);
    int getOutputCount();
    bool hasSoftPan(int ch);
    DivMacroInt* getChanMacroInt(int ch);
    unsigned short getPan(int chan);
    DivSamplePos getSamplePos(int ch);
    void setFlags(const DivConfig& flags);
    void notifyInsChange(int ins);
    void notifyInsDeletion(void* ins);
    void notifyPitchTable(int sample=-1);
    unsigned int getMaxFreq(int ch);
    int init(DivEngine* parent, int channels, int sugRate, const DivConfig& flags);
    void quit();
    DivPlatformPWM32X():
      chan(NULL), isMuted(NULL), chans(0), oscBuf(NULL), cycle(1047), lastL(0), lastR(0) {}
};

#endif
