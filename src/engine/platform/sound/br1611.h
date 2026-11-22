// =============================================================================
// BR1611 - chip de som do console fictício Ompreza 2 (1989)
// Síntese WMS (Wave Modulation Synthesis) + PCM nos canais 6 e 7
// Autor: Evolution BR811
// =============================================================================
//
// Convenções principais:
//  - Clock do chip: clock do RS16 (10,272 MHz) / 321 = 32.000 Hz de saída
//  - Frequência: f = fnum * 2^bloco * 32000 / 2^21   (fnum 11 bits, bloco 0-7)
//      ex.: Lá 0 (27,5 Hz) = fnum 1802 no bloco 0; Lá 4 (440 Hz) = fnum 1802 no bloco 4
//  - Ondas: 28 slots de 256 bytes, slot = canal * 4 + operador
//      cada byte é 0-255 com centro em 128 (igual ao wavetable do Furnace)
//  - 4 operadores com os 8 algoritmos e o feedback do OPN (YM2612): cada
//    operador modula a fase do seguinte, mas toca a sua propria onda
//  - LFO unico: vibrato (FMS) e tremolo (AMS + bit AM de cada operador)
//  - PCM: C-4 toca o sample a 32.000 Hz; endereços de 24 bits em little-endian
//  - Envelope do PCM: registradores do operador 1 (op 0) do canal
//  - CP: blocos de 9 bytes (cabeçalho + 16 amostras de 4 bits), mesmo layout
//    de bloco do BRR, para aproveitar o codificador BRR do Furnace
//
// Páginas de registradores: veja a especificação técnica do Ompreza 2.
// =============================================================================
#pragma once
#include <cstdint>
#include <cstddef>

class BR1611 {
public:
  static constexpr int CHANNELS = 7;
  static constexpr int OPS = 4;
  static constexpr int WAVE_SLOTS = 28;
  static constexpr int WAVE_LEN = 256;
  static constexpr int SAMPLE_RATE = 32000;

  BR1611();

  // volta o chip ao estado inicial
  void reset();

  // escreve num registrador (page 0 = global/canais/PCM, page 1 = operadores)
  void write(int page, uint8_t addr, uint8_t val);
  uint8_t read(int page, uint8_t addr) const;

  // memória de onde o PCM lê os samples (ROM do cartucho)
  void setSampleMemory(const uint8_t* mem, size_t len);

  // gera "count" amostras estéreo de 16 bits a 32.000 Hz
  void render(int16_t* outL, int16_t* outR, size_t count);

  // última saída de cada canal (útil para o osciloscópio do Furnace)
  int32_t chanOut(int ch) const { return chOut[ch]; }

private:
  enum EnvState { ENV_OFF, ENV_ATTACK, ENV_DECAY1, ENV_DECAY2, ENV_RELEASE };

  struct Op {
    uint32_t phase;   // fase de 32 bits; os 10 bits de cima = 1.024 pontos
    int32_t att;      // atenuação em 16.16 (0 = volume máximo, 1023 = silêncio)
    EnvState state;
    bool keyed;
    bool ssgInv;      // SSG-EG: saída invertida
    bool ssgHeld;     // SSG-EG: envelope travado (hold)
  };

  struct Pcm {
    bool active;
    uint64_t pos;        // posição no sample em 16.16
    uint32_t decodedIdx; // CP: próxima amostra a decodificar
    int32_t p1, p2;      // CP: histórico do filtro de previsão
    int32_t cur;         // CP: última amostra decodificada
  };

  uint8_t regs[2][256];
  uint8_t wave[WAVE_SLOTS][WAVE_LEN];
  uint8_t waveSlot, waveIdx;
  Op op[CHANNELS][OPS];
  Pcm pcm[CHANNELS];
  double lfoPhase;
  const uint8_t* mem;
  size_t memLen;
  int32_t chOut[CHANNELS];
  uint16_t gainTab[1024];

  bool isPcm(int ch) const;
  void keyReg(int ch, int mask);
  void keyOnOp(int ch, int o);
  void keyOffOp(int ch, int o);
  void envUpdate(Op& p, int ar, int d1, int d2, int rr, int sl, int rs, int kc, int ssg);
  int32_t opStep(int ch, int o, int fn, int blk, double vib, int kc, int32_t mod, int amAtt);
  int32_t fbHist[CHANNELS][2]; // feedback: as duas ultimas saidas do operador 1
  int32_t pcmStep(int ch, int fn, int blk, double vib, int kc);
  int32_t pcmFetch(int ch, int fmt, uint32_t start, uint32_t idx);
  uint8_t memAt(uint32_t addr) const;
  uint32_t rd24(int addr) const;
};
