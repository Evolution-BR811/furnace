// =============================================================================
// BR1611 - chip de som do console fictício Ompreza 2 (1989)
// Autor: Evolution BR811
// =============================================================================
#include "br1611.h"
#include <cmath>
#include <cstring>

namespace {
  const double PI = 3.14159265358979323846;
  // velocidades do LFO (Hz), mesmas do YM2612
  const double LFO_HZ[8] = {3.98, 5.56, 6.02, 6.37, 6.88, 9.63, 48.1, 72.2};
  // profundidade do vibrato (FMS) em cents
  const double FMS_CENTS[8] = {0, 3.4, 6.7, 10, 14, 20, 40, 80};
  // estéreo: 2 = 100%, 1 = 50%, 0 = desligado
  // posições: 0 esquerda, 1 meio-esquerda, 2 centro, 3 meio-direita, 4 direita
  const int PAN_L[5] = {2, 2, 2, 1, 0};
  const int PAN_R[5] = {0, 1, 2, 2, 2};
  // valor de (fnum << bloco) que corresponde a C-4 (toca o sample a 32.000 Hz)
  const double PCM_REF = 17146.0;
  const int32_t MAX_ATT = 1023 << 16;

  // passo do envelope por amostra (em 16.16) para uma taxa efetiva 0-63
  uint32_t envStep(int r) {
    if (r <= 0) return 0;
    return (uint32_t)(4 + (r & 3)) << (r >> 2);
  }

  int32_t clamp16(int32_t v) {
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return v;
  }
}

BR1611::BR1611() : mem(nullptr), memLen(0) {
  // tabela de ganho: 1024 passos de 0,09375 dB (cerca de 96 dB no total)
  for (int i = 0; i < 1024; i++) {
    gainTab[i] = (uint16_t)std::lround(std::pow(10.0, -i * 0.09375 / 20.0) * 4096.0);
  }
  reset();
}

void BR1611::reset() {
  std::memset(regs, 0, sizeof(regs));
  std::memset(wave, 128, sizeof(wave)); // ondas começam em silêncio (centro)
  waveSlot = 0;
  waveIdx = 0;
  lfoPhase = 0.0;
  for (int c = 0; c < CHANNELS; c++) {
    for (int o = 0; o < OPS; o++) {
      op[c][o] = {0, MAX_ATT, ENV_OFF, false, false, false};
    }
    pcm[c] = {false, 0, 0, 0, 0, 0};
    chOut[c] = 0;
    regs[0][0x20 + c] = 2; // estéreo começa no centro
  }
}

void BR1611::setSampleMemory(const uint8_t* m, size_t len) {
  mem = m;
  memLen = len;
}

uint8_t BR1611::read(int page, uint8_t addr) const {
  return regs[page & 1][addr];
}

void BR1611::write(int page, uint8_t addr, uint8_t val) {
  page &= 1;
  regs[page][addr] = val;
  if (page != 0) return;
  switch (addr) {
    case 0x01: { // key on/off: bits 7-4 operadores, bits 2-0 canal
      int ch = val & 7;
      if (ch < CHANNELS) keyReg(ch, val >> 4);
      break;
    }
    case 0x04: // slot da memória de onda (canal * 4 + operador)
      waveSlot = (val & 31) < WAVE_SLOTS ? (val & 31) : WAVE_SLOTS - 1;
      break;
    case 0x05: // índice do ponto
      waveIdx = val;
      break;
    case 0x06: // dado da onda, com auto-incremento
      wave[waveSlot][waveIdx++] = val;
      break;
  }
}

bool BR1611::isPcm(int ch) const {
  if (ch == 5) return regs[0][0x02] & 1;
  if (ch == 6) return regs[0][0x02] & 2;
  return false;
}

void BR1611::keyReg(int ch, int mask) {
  for (int o = 0; o < OPS; o++) {
    bool on = mask & (1 << o);
    if (on && !op[ch][o].keyed) {
      keyOnOp(ch, o);
      // no modo PCM, a borda de subida do operador 1 dispara o sample
      if (o == 0 && isPcm(ch)) {
        pcm[ch] = {true, 0, 0, 0, 0, 0};
      }
    } else if (!on && op[ch][o].keyed) {
      keyOffOp(ch, o);
    }
  }
}

void BR1611::keyOnOp(int ch, int o) {
  Op& p = op[ch][o];
  uint8_t ssg = regs[1][0xC0 + o * 8 + ch] & 15;
  p.phase = 0;
  p.keyed = true;
  p.state = ENV_ATTACK;
  p.ssgInv = (ssg & 8) && (ssg & 4);
  p.ssgHeld = false;
}

void BR1611::keyOffOp(int ch, int o) {
  Op& p = op[ch][o];
  // se o SSG-EG estava invertido, converte para a atenuação real antes do release
  if (p.ssgInv) {
    int a = p.att >> 16;
    a = (a < 0x200) ? 0x200 - a : 0;
    p.att = a << 16;
    p.ssgInv = false;
  }
  p.keyed = false;
  if (p.state != ENV_OFF) p.state = ENV_RELEASE;
}

void BR1611::envUpdate(Op& p, int ar, int d1, int d2, int rr, int sl, int rs, int kc, int ssg) {
  int ks = kc >> (3 - rs); // rate scaling: notas agudas aceleram o envelope
  auto eff = [ks](int r) {
    if (r == 0) return 0;
    int e = r * 2 + ks;
    return e > 63 ? 63 : e;
  };
  bool ssgOn = (ssg & 8) && p.keyed;

  switch (p.state) {
    case ENV_OFF:
      break;
    case ENV_ATTACK: {
      int r = eff(ar);
      if (r >= 62) { p.att = 0; p.state = ENV_DECAY1; break; }
      uint32_t st = envStep(r);
      if (!st) break;
      // ataque exponencial: rápido no começo, suave perto do volume máximo
      int32_t dec = (int32_t)((((int64_t)(p.att >> 16) + 1) * st) >> 7);
      if (dec < 1) dec = 1;
      p.att -= dec;
      if (p.att <= 0) { p.att = 0; p.state = ENV_DECAY1; }
      break;
    }
    case ENV_DECAY1: {
      if (ssgOn && p.ssgHeld) break;
      p.att += envStep(eff(d1));
      int32_t slv = (((sl == 15) ? 31 : sl) * 32) << 16;
      if (p.att >= slv) { p.att = slv; p.state = ENV_DECAY2; }
      break;
    }
    case ENV_DECAY2:
      if (ssgOn && p.ssgHeld) break;
      p.att += envStep(eff(d2));
      if (p.att > MAX_ATT) p.att = MAX_ATT;
      break;
    case ENV_RELEASE:
      p.att += envStep(eff(rr * 2 + 1));
      if (p.att >= MAX_ATT) { p.att = MAX_ATT; p.state = ENV_OFF; }
      break;
  }

  // SSG-EG simplificado: ao chegar em 0x200, repete, alterna ou trava
  if (ssgOn && !p.ssgHeld && (p.state == ENV_DECAY1 || p.state == ENV_DECAY2) &&
      (p.att >> 16) >= 0x200) {
    bool alt = ssg & 2, hold = ssg & 1;
    if (hold) {
      p.att = 0x200 << 16;
      if (alt) p.ssgInv = !p.ssgInv;
      p.ssgHeld = true;
    } else {
      if (alt) p.ssgInv = !p.ssgInv;
      else p.phase = 0;
      p.state = ENV_ATTACK;
    }
  }
}

int32_t BR1611::opStep(int ch, int o, int fn, int blk, double vib, int kc) {
  int i = o * 8 + ch;
  uint8_t r0 = regs[1][0x00 + i];
  int tl = regs[1][0x20 + i] & 0x7F;
  uint8_t r2 = regs[1][0x40 + i];
  int d1 = regs[1][0x60 + i] & 31;
  int d2 = regs[1][0x80 + i] & 31;
  uint8_t r5 = regs[1][0xA0 + i];
  int ssg = regs[1][0xC0 + i] & 15;

  int mul = r0 & 15, dt = (r0 >> 4) & 7;
  int rs = r2 >> 6, ar = r2 & 31;
  int sl = r5 >> 4, rr = r5 & 15;

  Op& p = op[ch][o];
  envUpdate(p, ar, d1, d2, rr, sl, rs, kc, ssg);
  if (p.state == ENV_OFF) return 0;

  // --- fase ---
  int64_t base = (int64_t)fn << blk;
  int64_t det = (int64_t)(dt & 3) << (blk + 1);
  base += (dt & 4) ? -det : det;
  if (base < 0) base = 0;
  double inc = (double)base * 2048.0 * (mul ? (double)mul : 0.5) * vib;
  p.phase += (uint32_t)(uint64_t)std::llround(inc);

  // --- onda: 256 pontos interpolados para 1.024 ---
  uint32_t pos = p.phase >> 22;         // 0-1023
  int idx = pos >> 2, frac = pos & 3;
  const uint8_t* w = wave[ch * 4 + o];
  int w0 = (int)w[idx] - 128;
  int w1 = (int)w[(idx + 1) & 255] - 128;
  int v = w0 * 4 + (w1 - w0) * frac;    // cerca de -512 a 508

  // --- atenuação: envelope + TL ---
  int a = p.att >> 16;
  if (p.ssgInv) a = (a < 0x200) ? 0x200 - a : 0;
  a += tl * 8;
  if (a > 1023) a = 1023;

  return (v * (int32_t)gainTab[a]) >> 8; // saída de 14 bits (cerca de ±8192)
}

uint8_t BR1611::memAt(uint32_t addr) const {
  return (mem && addr < memLen) ? mem[addr] : 0;
}

uint32_t BR1611::rd24(int addr) const {
  return (uint32_t)regs[0][addr] | ((uint32_t)regs[0][addr + 1] << 8) |
         ((uint32_t)regs[0][addr + 2] << 16);
}

// converte um tamanho em bytes para número de amostras, conforme o formato
static uint32_t idxCount(int fmt, uint32_t bytes) {
  switch (fmt) {
    case 0: return bytes / 2;          // PCM 16 bits
    case 1: return bytes;              // PCM 8 bits
    case 2: return bytes * 2;          // PCM 4 bits
    default: return (bytes / 9) * 16;  // CP: blocos de 9 bytes = 16 amostras
  }
}

int32_t BR1611::pcmFetch(int ch, int fmt, uint32_t start, uint32_t idx) {
  switch (fmt) {
    case 0: { // PCM 16 bits com sinal, little-endian
      uint32_t a = start + idx * 2;
      return (int16_t)(memAt(a) | (memAt(a + 1) << 8));
    }
    case 1: // PCM 8 bits com sinal
      return (int32_t)(int8_t)memAt(start + idx) << 8;
    case 2: { // PCM 4 bits com sinal, nibble alto primeiro
      uint8_t b = memAt(start + idx / 2);
      int n = (idx & 1) ? (b & 15) : (b >> 4);
      if (n >= 8) n -= 16;
      return n << 12;
    }
    default: { // CP: ADPCM próprio, estilo BRR
      Pcm& p = pcm[ch];
      // voltou para trás (loop): recomeça no início do bloco, mantendo o histórico
      if (idx + 1 < p.decodedIdx) p.decodedIdx = idx & ~15u;
      while (p.decodedIdx <= idx) {
        uint32_t k = p.decodedIdx;
        uint32_t ba = start + (k >> 4) * 9;
        uint8_t hdr = memAt(ba);
        int shift = hdr >> 4;
        int filter = (hdr >> 2) & 3;
        uint8_t b = memAt(ba + 1 + ((k & 15) >> 1));
        int n = (k & 1) ? (b & 15) : (b >> 4);
        if (n >= 8) n -= 16;
        // formato de bloco compatível com o BRR: (nibble << shift) >> 1
        int32_t s = (shift <= 12) ? ((n << shift) >> 1) : (n < 0 ? -2048 : 0);
        int32_t p1 = p.p1, p2 = p.p2;
        switch (filter) {
          case 1: s += p1 - (p1 >> 4); break;                                  // 15/16
          case 2: s += 2 * p1 - ((3 * p1) >> 5) - p2 + (p2 >> 4); break;       // 61/32, -15/16
          case 3: s += 2 * p1 - ((13 * p1) >> 6) - p2 + ((3 * p2) >> 4); break; // 115/64, -13/16
        }
        s = clamp16(s);
        s = (int16_t)(s << 1) >> 1; // 15 bits, como no BRR
        p.p2 = p.p1;
        p.p1 = s;
        p.cur = s * 2;               // saída em 16 bits
        p.decodedIdx++;
      }
      return p.cur;
    }
  }
}

int32_t BR1611::pcmStep(int ch, int fn, int blk, double vib, int kc) {
  // envelope do PCM = registradores do operador 1 (op 0) do canal
  Op& e = op[ch][0];
  int i = ch;
  int tl = regs[1][0x20 + i] & 0x7F;
  uint8_t r2 = regs[1][0x40 + i];
  int d1 = regs[1][0x60 + i] & 31;
  int d2 = regs[1][0x80 + i] & 31;
  uint8_t r5 = regs[1][0xA0 + i];
  envUpdate(e, r2 & 31, d1, d2, r5 & 15, r5 >> 4, r2 >> 6, kc, 0);

  Pcm& p = pcm[ch];
  if (!p.active) return 0;
  if (e.state == ENV_OFF) { p.active = false; return 0; }

  int rb = (ch == 5) ? 0x30 : 0x40;
  uint32_t start = rd24(rb), loop = rd24(rb + 3), end = rd24(rb + 6);
  uint8_t ctrl = regs[0][rb + 9];
  int fmt = ctrl & 3;
  bool lp = ctrl & 4;
  uint32_t len = idxCount(fmt, end > start ? end - start : 0);
  uint32_t lpi = idxCount(fmt, loop > start ? loop - start : 0);

  uint32_t idx = (uint32_t)(p.pos >> 16);
  if (idx >= len) {
    if (lp && lpi < len) {
      idx = lpi + (idx - len) % (len - lpi);
      p.pos = ((uint64_t)idx << 16) | (p.pos & 0xFFFF);
    } else {
      p.active = false;
      return 0;
    }
  }

  int32_t s = pcmFetch(ch, fmt, start, idx);

  // avança: em C-4 anda 1 amostra por saída (32.000 Hz)
  double stp = (double)((int64_t)fn << blk) * 65536.0 / PCM_REF * vib;
  p.pos += (uint64_t)std::llround(stp);

  int a = (e.att >> 16) + tl * 8;
  if (a > 1023) a = 1023;
  return (s * (int32_t)gainTab[a]) >> 12;
}

void BR1611::render(int16_t* outL, int16_t* outR, size_t count) {
  for (size_t n = 0; n < count; n++) {
    // LFO único do chip (só vibrato)
    bool lfoOn = regs[0][0x00] & 8;
    double lfo = 0.0;
    if (lfoOn) {
      lfoPhase += LFO_HZ[regs[0][0x00] & 7] / SAMPLE_RATE;
      if (lfoPhase >= 1.0) lfoPhase -= 1.0;
      lfo = std::sin(2.0 * PI * lfoPhase);
    }

    int32_t L = 0, R = 0;
    for (int ch = 0; ch < CHANNELS; ch++) {
      int fn = regs[0][0x08 + ch] | ((regs[0][0x10 + ch] & 7) << 8);
      int blk = (regs[0][0x10 + ch] >> 3) & 7;
      int fms = (regs[0][0x20 + ch] >> 4) & 7;
      int pan = regs[0][0x20 + ch] & 7;
      if (pan > 4) pan = 2;
      int vol = (int8_t)regs[0][0x18 + ch];
      double vib = (lfoOn && fms) ? std::pow(2.0, FMS_CENTS[fms] * lfo / 1200.0) : 1.0;
      int kc = (blk << 2) | ((fn >> 9) & 3); // código de tecla (rate scaling)

      int32_t sum = 0;
      if (isPcm(ch)) {
        sum = pcmStep(ch, fn, blk, vib, kc);
      } else {
        for (int o = 0; o < OPS; o++) sum += opStep(ch, o, fn, blk, vib, kc);
        // equilibra a WMS com o PCM: 2 operadores no máximo = escala cheia
        sum = clamp16(sum * 2);
      }

      int32_t out = (sum * vol) >> 9;
      chOut[ch] = out;
      L += (out * PAN_L[pan]) >> 1;
      R += (out * PAN_R[pan]) >> 1;
    }
    outL[n] = (int16_t)clamp16(L);
    outR[n] = (int16_t)clamp16(R);
  }
}
