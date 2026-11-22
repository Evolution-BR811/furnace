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

// the Sega 32X's PWM, as the hardware does it (after ares and PicoDrive):
// - two pulse width outputs, left and right, that take a value every cycle-1
//   clocks of the SH2's clock (the cycle register, 12 bits)
// - the value is the pulse's width in those clocks: the level is
//   width/(cycle-1), clipped at a full pulse (100%)
// - the control register's LMD/RMD: each output plays its own channel, the
//   other one, or nothing
// - the output goes through the board's DC removal (a one-pole high-pass,
//   0.999 per output, as in ares), so only changes are heard
// the 32X has no sound channels: its SH2 mixes them in software and writes
// the sum to the PWM. the channels here are that mixer, done like the
// drivers did it: samples stepped at the PWM's rate without interpolation, a
// volume and a left/right level each, summed into whole pulse widths
// (the PWM's FIFO and timer interrupt are how the SH2 keeps it fed: a mixer
// that writes every output never lets it run dry, so they aren't heard)
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
  // the cycle register (12 bits), the output modes (LMD/RMD: 0 off, 1 own
  // channel, 2 the other channel, 3 not allowed: off) and what the outputs
  // last held
  int cycle;
  int lmode, rmode;
  int lastL, lastR;
  // the DC removal's state per output (last input, output)
  double dcIn[2], dcOut[2];

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
      chan(NULL), isMuted(NULL), chans(0), oscBuf(NULL), cycle(1047), lmode(1), rmode(1), lastL(0), lastR(0), dcIn{0,0}, dcOut{0,0} {}
};

#endif
