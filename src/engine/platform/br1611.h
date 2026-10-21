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

#ifndef _BR1611_H
#define _BR1611_H

#include "../dispatch.h"
#include "sound/br1611.h"

// BR1611: chip de som do console ficticio Ompreza 2 (1989), por Evolution BR811.
// 7 canais de sintese WMS (4 operadores somados, cada um com sua propria onda).
// Os canais 6 e 7 podem tocar PCM (16, 8 ou 4 bits, ou CP) a partir da ROM do cartucho.
class DivPlatformBR1611: public DivDispatch {
  struct Channel: public SharedChannel {
    int pan;          // posicao estereo: 0 esq, 1 meio-esq, 2 centro, 3 meio-dir, 4 dir
    int fms;          // profundidade do vibrato (0-7)
    int waves[4];     // onda (wavetable da musica) de cada operador
    bool pcm;         // canais 6 e 7: tocando sample
    int sample;       // canais 6 e 7: numero do sample
    DivInstrumentFM state;
    Channel(bool linear=true):
      SharedChannel(127,linear),
      pan(2),
      fms(0),
      pcm(false),
      sample(-1) {
      for (int i=0; i<4; i++) waves[i]=-1;
    }
  };
  Channel chan[7];
  DivDispatchOscBuffer* oscBuf[7];
  bool isMuted[7];
  DivPitchTable pitchTable;
  DivPitchTableManager samplePitchTable;

  unsigned char regPool[512];
  BR1611 chip;
  int lfo;            // registrador $00 (liga + velocidade)

  // ROM do cartucho para os samples (4 MB, sem mapper)
  unsigned char* sampleMem;
  size_t sampleMemLen;
  unsigned int* sampleOff;
  unsigned int* sampleEnd;
  unsigned int* sampleLoopOff;
  unsigned char* sampleFmt;
  bool* sampleLoaded;
  DivMemoryComposition memCompo;

  void writeReg(int page, int addr, unsigned char val);
  void uploadWave(int ch, int op, int waveIndex);
  void commitOps(int ch);
  void commitChannel(int ch);
  void commitFreq(int ch);
  void setupPcm(int ch);
  void keyOn(int ch);
  void keyOff(int ch);

  friend void putDispatchChip(void*,int);
  friend void putDispatchChan(void*,int,int);
  public:
    void acquire(short** buf, size_t len);
    int dispatch(DivCommand c);
    SharedChannel* getChanState(int chan);
    DivMacroInt* getChanMacroInt(int ch);
    DivDispatchOscBuffer* getOscBuffer(int chan);
    unsigned char* getRegisterPool();
    int getRegisterPoolSize();
    void reset();
    void forceIns();
    void tick(bool sysTick=true);
    void muteChannel(int ch, bool mute);
    bool keyOffAffectsArp(int ch);
    void setFlags(const DivConfig& flags);
    void notifyInsChange(int ins);
    void notifyWaveChange(int wave);
    void notifyInsDeletion(void* ins);
    void notifyPitchTable(int sample=-1);
    unsigned int getMaxFreq(int ch);
    int getOutputCount();
    void poke(unsigned int addr, unsigned short val);
    void poke(std::vector<DivRegWrite>& wlist);
    const char** getRegisterSheet();
    const void* getSampleMem(int index);
    size_t getSampleMemCapacity(int index);
    size_t getSampleMemUsage(int index);
    bool isSampleLoaded(int index, int sample);
    const DivMemoryComposition* getMemCompo(int index);
    void renderSamples(int chipID);
    int init(DivEngine* parent, int channels, int sugRate, const DivConfig& flags);
    void quit();
    DivPlatformBR1611();
    ~DivPlatformBR1611();
};

#endif
