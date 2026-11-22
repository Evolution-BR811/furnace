// Editores de instrumento do BR811 (console ficticio Ompreza) e do BR411
// (portatil ficticio Ompreza P). Por Evolution BR811.

#include "insEditCommon.h"

// "BR811 corrigido" ligado em algum BR811 da musica: a Onda A e quadrada
static bool br811SquareWaveA=false;

static String br811WaveformName(int index, float value, void* data) {
  switch ((int)value) {
    case 0: return String(br811SquareWaveA?_("Wave A (square)"):_("Wave A"));
    case 1: return String(_("Wave B"));
    case 2: return String(_("Noise /512"));
    case 3: return String(_("Noise /1024"));
    case 4: return String(_("Noise /2048"));
    case 5: return String(_("Noise (linked to pitch)"));
    case 6: return String(_("Noise /256 (open hat)"));
    case 7: return String(_("Noise /128 (closed hat)"));
  }
  return String("");
}

// BR411: sem a Onda B, as fontes vao de 0 a 6 (mesmas taxas de ruido do BR811)
static String br411WaveformName(int index, float value, void* data) {
  switch ((int)value) {
    case 0: return String(_("Wave A"));
    case 1: return String(_("Noise 12.5 kHz"));
    case 2: return String(_("Noise 6.25 kHz"));
    case 3: return String(_("Noise 3.125 kHz"));
    case 4: return String(_("Noise (linked to pitch)"));
    case 5: return String(_("Noise 25 kHz (open hat)"));
    case 6: return String(_("Noise 50 kHz (closed hat)"));
  }
  return String("");
}

void FurnaceGUI::insEditBR811(DivInstrument* ins) {
  std::vector<FurnaceGUIMacroDesc> macroList;

  // aba Sample: PCM nativo do canal 5
  insTabSample(ins);

  // volume 0-15 (como NES e Master System). So um instrumento de sample (PCM)
  // numa musica com o chip de cartucho PCM Volume Plus vai de 0 a 30.
  bool pcmVolumePlus=false;
  bool hasBR411=false;
  br811SquareWaveA=false;
  for (int i=0; i<e->song.systemLen; i++) {
    if (e->song.system[i]==DIV_SYSTEM_BR811) {
      if (e->song.systemFlags[i].getBool("pcmVolumePlus",false)) pcmVolumePlus=true;
      if (e->song.systemFlags[i].getBool("squareWaveA",false)) br811SquareWaveA=true;
    }
    if (e->song.system[i]==DIV_SYSTEM_BR411) hasBR411=true;
  }
  int volMax=(ins->amiga.useSample && pcmVolumePlus)?30:15;
  // o editor guarda o zoom vertical de cada macro; quando o maximo muda
  // (15 <-> 30), o zoom antigo esconderia a parte de cima. Reajusta.
  if (ins->temp.vZoom[DIV_MACRO_VOL]!=volMax) {
    ins->temp.vZoom[DIV_MACRO_VOL]=-1;
  }

  if (ImGui::BeginTabItem(_("Macros"))) {
    // musica com BR411: oferece converter este instrumento para o tipo BR411
    if (hasBR411) {
      if (ImGui::Button(_("Convert to BR411 instrument"))) {
        // fontes do BR811 -> BR411: Onda B vira Onda A, ruidos descem 1
        for (int k=0; k<ins->std.dutyMacro.len; k++) {
          int v=ins->std.dutyMacro.val[k];
          ins->std.dutyMacro.val[k]=(v<=1)?0:MIN(v-1,6);
        }
        ins->type=DIV_INS_BR411;
        ins->amiga.useSample=false;
        memset(ins->temp.vZoom,-1,sizeof(ins->temp.vZoom));
        MARK_MODIFIED;
      }
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(_("changes this instrument to the BR411 type and renumbers its Waveform macro (no Wave B on BR411). 20xx effects in the song are not changed."));
      }
    }
    macroList.push_back(FurnaceGUIMacroDesc(_("Volume"),&ins->std.volMacro,0,volMax,160,uiColors[GUI_COLOR_MACRO_VOLUME]));
    macroList.push_back(FurnaceGUIMacroDesc(_("Arpeggio"),&ins->std.arpMacro,-120,120,160,uiColors[GUI_COLOR_MACRO_PITCH],true,NULL,macroHoverNote,false,NULL,true,ins->std.arpMacro.val));
    macroList.push_back(FurnaceGUIMacroDesc(_("Waveform"),&ins->std.dutyMacro,0,7,160,uiColors[GUI_COLOR_MACRO_WAVE],false,NULL,br811WaveformName));
    macroList.push_back(FurnaceGUIMacroDesc(_("Pitch"),&ins->std.pitchMacro,-2048,2047,160,uiColors[GUI_COLOR_MACRO_PITCH],true,macroRelativeMode));

    drawMacros(macroList,macroEditStateMacros,ins);
    ImGui::EndTabItem();
  }
}

// BR411: so macros (sem aba Sample, o chip nao tem PCM). Volume 0-15.
void FurnaceGUI::insEditBR411(DivInstrument* ins) {
  std::vector<FurnaceGUIMacroDesc> macroList;

  if (ImGui::BeginTabItem(_("Macros"))) {
    macroList.push_back(FurnaceGUIMacroDesc(_("Volume"),&ins->std.volMacro,0,15,160,uiColors[GUI_COLOR_MACRO_VOLUME]));
    macroList.push_back(FurnaceGUIMacroDesc(_("Arpeggio"),&ins->std.arpMacro,-120,120,160,uiColors[GUI_COLOR_MACRO_PITCH],true,NULL,macroHoverNote,false,NULL,true,ins->std.arpMacro.val));
    macroList.push_back(FurnaceGUIMacroDesc(_("Waveform"),&ins->std.dutyMacro,0,6,160,uiColors[GUI_COLOR_MACRO_WAVE],false,NULL,br411WaveformName));
    macroList.push_back(FurnaceGUIMacroDesc(_("Pitch"),&ins->std.pitchMacro,-2048,2047,160,uiColors[GUI_COLOR_MACRO_PITCH],true,macroRelativeMode));

    drawMacros(macroList,macroEditStateMacros,ins);
    ImGui::EndTabItem();
  }
}
