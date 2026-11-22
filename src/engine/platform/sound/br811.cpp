#include "br811.h"

#include <algorithm>
#include <cmath>

namespace br811 {

// Formas de onda finais (secao 5 da especificacao).
// A Onda A foi desenhada de 0 a 7 e sai deslocada em 1 bit (0, 12, 14).
static const uint8_t WAVE_A[WAVE_A_STEPS] = {0, 0, 12, 14, 14, 14, 12, 0};
// BR811 corrigido: onda quadrada verdadeira (metade alta, metade baixa), com o
// mesmo pico da Onda A original para manter o equilibrio com a Onda B.
static const uint8_t WAVE_A_SQUARE[WAVE_A_STEPS] = {0, 0, 14, 14, 14, 14, 0, 0};
static const uint8_t WAVE_B[WAVE_B_STEPS] = {0, 0, 15, 15, 15, 15, 15, 15,
                                             15, 0, 0, 0, 15, 15, 0, 0};

// Duracao de um passo do PCM por unidade de (periodo + 1), em unidades de tempo.
// A taxa do PCM e 8.000 Hz * (PCM_REF_PERIOD + 1) / (periodo + 1), entao em C-4
// (periodo 430) o sample toca exatamente a 8.000 Hz.
static constexpr int64_t PCM_UNITS_PER_PERIOD =
    static_cast<int64_t>(UNITS_PER_CLOCK * CLOCK_HZ / (8000.0 * (PCM_REF_PERIOD + 1)));

// Nivel maximo de um canal (onda 15 * volume 15), usado para normalizar a mistura.
static constexpr float CHANNEL_MAX = 15.0f * VOLUME_MAX;

// A saida de cada canal e centrada em zero (onda de -7,5 a +7,5) e a mistura
// recebe este ganho (+6 dB), acompanhando o aumento de volume do BR1611 para
// manter o equilibrio entre os dois chips no Ompreza 2. Vale para o BR811 e o
// BR411. Quando todos os canais estao no maximo juntos (ou o PCM passa de 15
// no PCM Volume Plus), um limitador suave segura a saida em vez de cortar seco.
static constexpr float WAVE_CENTER = 7.5f;
static constexpr double OUTPUT_GAIN = 2.0;

static double softLimit(double x) {
    const double T = 0.75;
    const double a = std::fabs(x);
    if (a <= T) return x;
    const double y = T + (1.0 - T) * std::tanh((a - T) / (1.0 - T));
    return x < 0 ? -y : y;
}

// ---------------------------------------------------------------------------
// Utilitarios de afinacao
// ---------------------------------------------------------------------------
int periodForFrequency(double hz) {
    const double p = CLOCK_HZ / (DIVISOR * hz) - 1.0;
    const int v = static_cast<int>(std::lround(p));
    return v < 0 ? 0 : (v > MAX_PERIOD ? MAX_PERIOD : v);
}

int periodForMidi(int midiNote) {
    return periodForFrequency(440.0 * std::pow(2.0, (midiNote - 69) / 12.0));
}

// ---------------------------------------------------------------------------
// Chip
// ---------------------------------------------------------------------------
Chip::Chip(double sampleRate) : sampleRate_(sampleRate) {
    unitsPerSample_ = (clockHz_ / sampleRate_) * UNITS_PER_CLOCK;
    reset();
}

void Chip::setSampleRate(double sampleRate) {
    sampleRate_ = sampleRate;
    unitsPerSample_ = (clockHz_ / sampleRate_) * UNITS_PER_CLOCK;
}

void Chip::setVariant(Variant v) {
    variant_ = v;
    if (v == VARIANT_BR411) {
        clockHz_ = BR411_CLOCK_HZ;
        divisor_ = BR411_DIVISOR;
        numCh_   = BR411_NUM_CHANNELS;
        pcmVolumePlus_ = false;  // o BR411 nao tem PCM
        squareWaveA_ = false;    // nem a versao corrigida da Onda A
    } else {
        clockHz_ = CLOCK_HZ;
        divisor_ = DIVISOR;
        numCh_   = NUM_CHANNELS;
    }
    unitsPerSample_ = (clockHz_ / sampleRate_) * UNITS_PER_CLOCK;
    reset();
}

// Divisores do ruido, na ordem do enum NoiseRate (0..5).
// BR811: potencias de 2 do clock de 6,42 MHz.
// BR411: escolhidos para dar as mesmas taxas em Hz com o clock de 5 MHz
// (12.500, 6.250, 3.125, -, 25.000 e 50.000 Hz).
static const int64_t NOISE_DIV_BR811[6] = {512, 1024, 2048, 0, 256, 128};
static const int64_t NOISE_DIV_BR411[6] = {400,  800, 1600, 0, 200, 100};

void Chip::reset() {
    for (int i = 0; i < NUM_CHANNELS; i++) {
        ch_[i] = Channel();
        ch_[i].timer = stepPeriod(ch_[i], i);
        noiseShifts_[i] = 0;
    }
    pcm_ = Pcm();
    unitsAccum_ = 0.0;
    totalUnits_ = 0;
    stolenCycles_ = 0;
}

// Quanto tempo dura um passo no modo atual do canal.
int64_t Chip::stepPeriod(const Channel& c, int index) const {
    const int64_t p1 = static_cast<int64_t>(c.period) + 1;
    switch (c.mode) {
        case MODE_WAVE_A:
            // BR811: 7,125 clocks * (P+1); BR411: 8 clocks * (P+1)
            return divisor_ * p1 * UNITS_PER_CLOCK / WAVE_A_STEPS;
        case MODE_WAVE_B:
            return divisor_ * p1 * UNITS_PER_CLOCK / WAVE_B_STEPS;  // 3,5625 clocks * (P+1)
        case MODE_NOISE: {
            if (c.noiseRate == NOISE_LINKED) return divisor_ * p1 * UNITS_PER_CLOCK;  // vinculado
            // codigos reservados (6 e 7) agem como o codigo 0
            const int rate = (c.noiseRate <= NOISE_DIV128) ? static_cast<int>(c.noiseRate) : static_cast<int>(NOISE_DIV512);
            const int64_t* table = (variant_ == VARIANT_BR411) ? NOISE_DIV_BR411 : NOISE_DIV_BR811;
            return table[rate] * UNITS_PER_CLOCK;
        }
        case MODE_PCM:
            if (index == PCM_CHANNEL) return PCM_UNITS_PER_PERIOD * p1;
            break;
    }
    return UNITS_PER_CLOCK * 4096;  // modo reservado: silencio
}

// Nivel de saida do canal agora, ja multiplicado pelo volume.
float Chip::level(const Channel& c, int index) const {
    float v = 0.0f;
    switch (c.mode) {
        case MODE_WAVE_A: v = (squareWaveA_ ? WAVE_A_SQUARE[c.step] : WAVE_A[c.step]) - WAVE_CENTER; break;
        case MODE_WAVE_B: v = WAVE_B[c.step] - WAVE_CENTER; break;
        case MODE_NOISE:  v = (c.lfsr & 1) ? WAVE_CENTER : -WAVE_CENTER; break;
        case MODE_PCM:
            if (index == PCM_CHANNEL) {
                // PCM parado fica no ponto medio (silencio), sem estalo no fim do sample
                v = pcm_.playing ? (pcm_.sample - 63.5f) * (15.0f / 127.0f) : 0.0f;
            }
            break;
    }
    return v * c.volume;
}

// Um passo do canal: avanca a onda, o LFSR ou o PCM.
void Chip::advance(Channel& c, int index) {
    switch (c.mode) {
        case MODE_WAVE_A: c.step = (c.step + 1) % WAVE_A_STEPS; break;
        case MODE_WAVE_B: c.step = (c.step + 1) % WAVE_B_STEPS; break;
        case MODE_NOISE: {
            // LFSR de 16 bits, realimentacao por XOR dos bits 0 e 3 (premissa
            // da especificacao: versao Sega do SN76489).
            const uint16_t feedback = (c.lfsr ^ (c.lfsr >> 3)) & 1;
            c.lfsr = static_cast<uint16_t>((c.lfsr >> 1) | (feedback << 15));
            if (c.lfsr == 0) c.lfsr = 0x8000;  // nunca trava em zero
            noiseShifts_[index]++;
            break;
        }
        case MODE_PCM:
            if (index == PCM_CHANNEL) pcmNext();
            break;
    }
}

// Integra o nivel do canal ao longo de dt unidades de tempo, avancando os
// passos que caem nesse intervalo. A integral da uma media por amostra
// (filtro de caixa), o que reduz o aliasing sem precisar de um buffer a mais.
double Chip::runChannel(Channel& c, int index, int64_t dt) {
    double acc = 0.0;
    while (dt > 0) {
        const int64_t slice = std::min(dt, c.timer);
        acc += static_cast<double>(level(c, index)) * static_cast<double>(slice);
        c.timer -= slice;
        dt -= slice;
        if (c.timer == 0) {
            advance(c, index);
            c.timer = stepPeriod(c, index);
        }
    }
    return acc;
}

void Chip::write(int channel, int reg, uint8_t value) {
    if (channel < 0 || channel >= numCh_) return;
    Channel& c = ch_[channel];
    switch (reg) {
        case 0: {
            uint8_t newMode = (value >> 6) & 3;
            // BR411: nao tem o circuito da Onda B nem o PCM, e o bit 0 do modo
            // nem chega ao chip: modo 1 (Onda B) toca a Onda A e modo 3 (PCM) toca ruido
            if (variant_ == VARIANT_BR411) newMode &= 2;
            // volume de 4 bits (0-15); o PCM do canal 5 usa 5 bits (0-30) quando
            // o chip de cartucho PCM Volume Plus esta presente
            c.volume = (pcmVolumePlus_ && newMode == MODE_PCM && channel == PCM_CHANNEL)
                           ? static_cast<uint8_t>(std::min<int>(value & 0x1F, PCM_VOLUME_MAX))
                           : static_cast<uint8_t>(value & 0x0F);
            if (newMode != c.mode) {
                c.mode = newMode;
                c.step = 0;
                c.timer = stepPeriod(c, channel);
            }
            break;
        }
        case 1: c.period = static_cast<uint16_t>((c.period & 0xF00) | value); break;
        case 2:
            c.period = static_cast<uint16_t>((c.period & 0x0FF) | ((value & 0x0F) << 8));
            c.noiseRate = (value >> 4) & 7;  // taxa de ruido: 3 bits (bits 6-4) (codigos 6 e 7 reservados)
            break;
    }
}

void Chip::writePcm(int reg, uint8_t value) {
    if (variant_ != VARIANT_BR811) return;  // o BR411 nao tem PCM
    switch (reg) {
        case PCM_ADDR_LO: pcm_.addr = static_cast<uint16_t>((pcm_.addr & 0xFF00) | value); break;
        case PCM_ADDR_HI: pcm_.addr = static_cast<uint16_t>((pcm_.addr & 0x00FF) | (value << 8)); break;
        case PCM_LENGTH:  pcm_.length = value; break;
        case PCM_CTRL: {
            pcm_.loop = (value & PCM_CTRL_LOOP) != 0;
            const bool play = (value & PCM_CTRL_PLAY) != 0;
            if (play && !pcm_.playing) {
                pcm_.playing = true;   // comeca do inicio do sample
                pcm_.offset = 0;
                pcmFetch();
                ch_[PCM_CHANNEL].timer = stepPeriod(ch_[PCM_CHANNEL], PCM_CHANNEL);
            } else if (!play) {
                pcm_.playing = false;
            }
            break;
        }
    }
}

// Le o byte atual do sample na ROM. Cada leitura rouba ciclos da CPU.
void Chip::pcmFetch() {
    const uint16_t address = static_cast<uint16_t>(pcm_.addr + pcm_.offset);
    const uint8_t byte = rom_ ? rom_(address) : 0;
    pcm_.sample = byte & 0x7F;  // PCM linear de 7 bits
    stolenCycles_ += PCM_CYCLES_PER_BYTE;
}

void Chip::pcmNext() {
    if (!pcm_.playing) return;
    const uint32_t totalBytes = (static_cast<uint32_t>(pcm_.length) + 1) * PCM_BLOCK_BYTES;
    if (++pcm_.offset >= totalBytes) {
        if (pcm_.loop) {
            pcm_.offset = 0;   // loop global: repete o sample inteiro
        } else {
            pcm_.playing = false;
            return;
        }
    }
    pcmFetch();
}

void Chip::render(float* out, int numSamples, float* const* perChannel) {
    for (int i = 0; i < numSamples; i++) {
        unitsAccum_ += unitsPerSample_;
        const int64_t dt = static_cast<int64_t>(unitsAccum_);
        unitsAccum_ -= static_cast<double>(dt);

        double mix = 0.0;
        for (int c = 0; c < numCh_; c++) {
            const double v = runChannel(ch_[c], c, dt) / static_cast<double>(dt);
            mix += v;
            // o PCM com volume acima de 15 passa de 1,0; limita para o osciloscopio
            if (perChannel) perChannel[c][i] = static_cast<float>(std::max(std::min(v / (CHANNEL_MAX * 0.5), 1.99), -1.99));
        }
        totalUnits_ += dt;
        // mesma escala no BR811 e no BR411, com limitador suave
        out[i] = static_cast<float>(softLimit(mix * OUTPUT_GAIN / (NUM_CHANNELS * CHANNEL_MAX)));
    }
}

}  // namespace br811
