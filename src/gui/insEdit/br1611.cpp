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

// Editor de instrumento do BR1611 (chip de som do console ficticio Ompreza 2).
// Sintese WMS: 4 operadores somados, cada um com sua onda e seu envelope.
// Por Evolution BR811.

#include "insEditCommon.h"
#include "../intConst.h"

static const char* br1611PanNames[5]={
  _N("left"),
  _N("half left"),
  _N("center"),
  _N("half right"),
  _N("right")
};

static String br1611PanName(int index, float value, void* data) {
  int v=(int)value;
  if (v<0 || v>4) return String("");
  return String(_(br1611PanNames[v]));
}

static const char* br1611SSGNames[8]={
  "\\\\\\\\", "\\___", "\\/\\/", "\\---", "////", "/---", "/\\/\\", "/___"
};

void FurnaceGUI::insEditBR1611(DivInstrument* ins) {
  std::vector<FurnaceGUIMacroDesc> macroList;
  char label[64];

  if (ImGui::BeginTabItem(_("WMS"))) {
    ImGui::TextWrapped(_("WMS (Wave Modulation Synthesis): the 4 operators are summed. each one plays its own wavetable with its own envelope."));

    P(CWSliderScalar(_("Vibrato depth (FMS)"),ImGuiDataType_U8,&ins->fm.fms,&_ZERO,&_SEVEN)); rightClickable
    ImGui::TextDisabled(_("the LFO is shared by the whole chip: use effect 10xx to turn it on and pick its speed."));

    float sliderWidth=20.0f*dpiScale;
    float sliderHeight=110.0f*dpiScale;
    ImVec2 sliderSize=ImVec2(sliderWidth,sliderHeight);

    for (int j=0; j<4; j++) {
      DivInstrumentFM::Operator& op=ins->fm.op[j];
      ImGui::PushID(j);
      ImGui::Separator();

      snprintf(label,63,_("Operator %d"),j+1);
      bool enabled=op.enable;
      if (ImGui::Checkbox(label,&enabled)) {
        op.enable=enabled;
        PARAMETER;
      }

      // onda do operador: uma wavetable da musica
      ImGui::SameLine();
      int wave=ins->br1611.wave[j];
      ImGui::SetNextItemWidth(100.0f*dpiScale);
      if (ImGui::InputInt(_("Wave##BRWave"),&wave,1,4)) {
        if (wave<0) wave=0;
        if (wave>255) wave=255;
        ins->br1611.wave[j]=wave;
        PARAMETER;
      }
      ImGui::SameLine();
      if (ImGui::Button(_("Edit##BRWaveEdit"))) {
        if (e->song.waveLen<1) {
          showError(_("this song has no wavetables yet. add one in the wavetable list first."));
        } else {
          curWave=MIN(ins->br1611.wave[j],e->song.waveLen-1);
          waveEditOpen=true;
          nextWindow=GUI_WINDOW_WAVE_EDIT;
        }
      }

      // pre-visualizacao da onda
      if (e->song.waveLen>0 && ins->br1611.wave[j]<e->song.waveLen) {
        DivWavetable* w=e->getWave(ins->br1611.wave[j]);
        if (w!=NULL && w->len>0) {
          float data[256];
          int len=MIN(w->len,256);
          for (int k=0; k<len; k++) {
            data[k]=(w->max>0)?((float)w->data[k]/(float)w->max):0.0f;
          }
          ImGui::PlotLines("##BRWavePreview",data,len,0,NULL,0.0f,1.0f,ImVec2(ImGui::GetContentRegionAvail().x,40.0f*dpiScale));
        }
      } else {
        if (e->song.waveLen<1) {
          ImGui::TextDisabled(_("no wavetables in this song: this operator will play a sine wave."));
        } else {
          ImGui::TextDisabled(_("this wavetable does not exist: wavetable 0 will be used."));
        }
      }

      // envelope (estilo OPN): AR, D1R, SL, D2R, RR e TL
      if (ImGui::BeginTable("BREnv",7,ImGuiTableFlags_NoHostExtendX)) {
        for (int k=0; k<6; k++) {
          ImGui::TableSetupColumn(fmt::sprintf("c%d",k).c_str(),ImGuiTableColumnFlags_WidthFixed,sliderWidth);
        }
        ImGui::TableSetupColumn("env",ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableNextRow();
        const char* names[6]={"AR","D1R","SL","D2R","RR","TL"};
        for (int k=0; k<6; k++) {
          ImGui::TableNextColumn();
          CENTER_TEXT(names[k]);
          ImGui::TextUnformatted(names[k]);
        }
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(_("Envelope"));

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        P(CWVSliderScalar("##AR",sliderSize,ImGuiDataType_U8,&op.ar,&_THIRTY_ONE,&_ZERO)); rightClickable
        ImGui::TableNextColumn();
        P(CWVSliderScalar("##D1R",sliderSize,ImGuiDataType_U8,&op.dr,&_THIRTY_ONE,&_ZERO)); rightClickable
        ImGui::TableNextColumn();
        P(CWVSliderScalar("##SL",sliderSize,ImGuiDataType_U8,&op.sl,&_FIFTEEN,&_ZERO)); rightClickable
        ImGui::TableNextColumn();
        P(CWVSliderScalar("##D2R",sliderSize,ImGuiDataType_U8,&op.d2r,&_THIRTY_ONE,&_ZERO)); rightClickable
        ImGui::TableNextColumn();
        P(CWVSliderScalar("##RR",sliderSize,ImGuiDataType_U8,&op.rr,&_FIFTEEN,&_ZERO)); rightClickable
        ImGui::TableNextColumn();
        P(CWVSliderScalar("##TL",sliderSize,ImGuiDataType_U8,&op.tl,&_ONE_HUNDRED_TWENTY_SEVEN,&_ZERO)); rightClickable
        ImGui::TableNextColumn();
        drawFMEnv(op.tl&127,op.ar&31,op.dr&31,op.d2r&31,op.rr&15,op.sl&15,0,op.ssgEnv&8,7,127,31,15,ImVec2(ImGui::GetContentRegionAvail().x,sliderHeight),DIV_INS_FM);
        ImGui::EndTable();
      }

      // afinacao e outros parametros
      if (ImGui::BeginTable("BRParams",2,ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        P(CWSliderScalar(_("MULT"),ImGuiDataType_U8,&op.mult,&_ZERO,&_FIFTEEN)); rightClickable
        ImGui::TableNextColumn();
        P(CWSliderScalar(_("DT"),ImGuiDataType_U8,&op.dt,&_ZERO,&_SEVEN)); rightClickable

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        P(CWSliderScalar(_("RS"),ImGuiDataType_U8,&op.rs,&_ZERO,&_THREE)); rightClickable
        ImGui::TableNextColumn();
        bool ssgOn=op.ssgEnv&8;
        if (ImGui::Checkbox(_("SSG-EG"),&ssgOn)) {
          op.ssgEnv=(op.ssgEnv&7)|(ssgOn?8:0);
          PARAMETER;
        }
        if (ssgOn) {
          ImGui::SameLine();
          int ssgType=op.ssgEnv&7;
          ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
          if (ImGui::SliderInt("##SSGType",&ssgType,0,7,br1611SSGNames[ssgType&7])) {
            op.ssgEnv=8|(ssgType&7);
            PARAMETER;
          }
        }
        ImGui::EndTable();
      }
      ImGui::PopID();
    }
    ImGui::EndTabItem();
  }

  // aba Sample: PCM dos canais 6 e 7 (com envelope proprio do PCM)
  insTabSample(ins);

  for (int i=0; i<4; i++) {
    snprintf(label,63,_("OP%d Macros"),i+1);
    if (ImGui::BeginTabItem(label)) {
      ImGui::PushID(i);
      macroList.push_back(FurnaceGUIMacroDesc(_("Wave"),&ins->std.opMacros[i].wsMacro,0,255,160,uiColors[GUI_COLOR_MACRO_WAVE]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_TL),&ins->std.opMacros[i].tlMacro,0,127,128,uiColors[GUI_COLOR_MACRO_VOLUME]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_AR),&ins->std.opMacros[i].arMacro,0,31,64,uiColors[GUI_COLOR_MACRO_ENVELOPE]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_DR),&ins->std.opMacros[i].drMacro,0,31,64,uiColors[GUI_COLOR_MACRO_ENVELOPE]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_SL),&ins->std.opMacros[i].slMacro,0,15,64,uiColors[GUI_COLOR_MACRO_ENVELOPE]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_D2R),&ins->std.opMacros[i].d2rMacro,0,31,64,uiColors[GUI_COLOR_MACRO_ENVELOPE]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_RR),&ins->std.opMacros[i].rrMacro,0,15,64,uiColors[GUI_COLOR_MACRO_ENVELOPE]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_RS),&ins->std.opMacros[i].rsMacro,0,3,32,uiColors[GUI_COLOR_MACRO_OTHER]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_MULT),&ins->std.opMacros[i].multMacro,0,15,64,uiColors[GUI_COLOR_MACRO_OTHER]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_DT),&ins->std.opMacros[i].dtMacro,0,7,64,uiColors[GUI_COLOR_MACRO_PITCH]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_SSG),&ins->std.opMacros[i].ssgMacro,0,4,64,uiColors[GUI_COLOR_MACRO_ENVELOPE],false,NULL,NULL,true,ssgEnvBits));

      drawMacros(macroList,macroEditStateOP[i],ins);
      ImGui::PopID();
      ImGui::EndTabItem();
    }
  }

  if (ImGui::BeginTabItem(_("Macros"))) {
    macroList.push_back(FurnaceGUIMacroDesc(_("Volume"),&ins->std.volMacro,0,127,160,uiColors[GUI_COLOR_MACRO_VOLUME]));
    macroList.push_back(FurnaceGUIMacroDesc(_("Arpeggio"),&ins->std.arpMacro,-120,120,160,uiColors[GUI_COLOR_MACRO_PITCH],true,NULL,macroHoverNote,false,NULL,true,ins->std.arpMacro.val));
    macroList.push_back(FurnaceGUIMacroDesc(_("Panning"),&ins->std.panLMacro,0,4,96,uiColors[GUI_COLOR_MACRO_OTHER],false,NULL,br1611PanName));
    macroList.push_back(FurnaceGUIMacroDesc(_("Pitch"),&ins->std.pitchMacro,-2048,2047,160,uiColors[GUI_COLOR_MACRO_PITCH],true,macroRelativeMode));
    macroList.push_back(FurnaceGUIMacroDesc(_("Vibrato depth"),&ins->std.fmsMacro,0,7,96,uiColors[GUI_COLOR_MACRO_OTHER]));
    macroList.push_back(FurnaceGUIMacroDesc(_("Phase Reset"),&ins->std.phaseResetMacro,0,1,32,uiColors[GUI_COLOR_MACRO_OTHER],false,NULL,NULL,true));

    drawMacros(macroList,macroEditStateMacros,ins);
    ImGui::EndTabItem();
  }
}

// envelope proprio do PCM (canais 6 e 7), desenhado dentro da aba Sample.
// fica separado do envelope dos operadores da WMS.
void FurnaceGUI::insEditBR1611PCMEnv(DivInstrument* ins) {
  ImGui::Separator();
  ImGui::Text(_("PCM envelope (channels 6 and 7)"));
  ImGui::TextDisabled(_("controls how the sample note fades. separate from the WMS operator envelopes."));

  float sliderWidth=20.0f*dpiScale;
  float sliderHeight=110.0f*dpiScale;
  ImVec2 sliderSize=ImVec2(sliderWidth,sliderHeight);

  if (ImGui::BeginTable("BRPCMEnv",7,ImGuiTableFlags_NoHostExtendX)) {
    for (int k=0; k<6; k++) {
      ImGui::TableSetupColumn(fmt::sprintf("c%d",k).c_str(),ImGuiTableColumnFlags_WidthFixed,sliderWidth);
    }
    ImGui::TableSetupColumn("env",ImGuiTableColumnFlags_WidthStretch);

    ImGui::TableNextRow();
    const char* names[6]={"AR","D1R","SL","D2R","RR","TL"};
    for (int k=0; k<6; k++) {
      ImGui::TableNextColumn();
      CENTER_TEXT(names[k]);
      ImGui::TextUnformatted(names[k]);
    }
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(_("Envelope"));

    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    P(CWVSliderScalar("##PCMAR",sliderSize,ImGuiDataType_U8,&ins->br1611.pcmAR,&_THIRTY_ONE,&_ZERO)); rightClickable
    ImGui::TableNextColumn();
    P(CWVSliderScalar("##PCMD1R",sliderSize,ImGuiDataType_U8,&ins->br1611.pcmD1R,&_THIRTY_ONE,&_ZERO)); rightClickable
    ImGui::TableNextColumn();
    P(CWVSliderScalar("##PCMSL",sliderSize,ImGuiDataType_U8,&ins->br1611.pcmSL,&_FIFTEEN,&_ZERO)); rightClickable
    ImGui::TableNextColumn();
    P(CWVSliderScalar("##PCMD2R",sliderSize,ImGuiDataType_U8,&ins->br1611.pcmD2R,&_THIRTY_ONE,&_ZERO)); rightClickable
    ImGui::TableNextColumn();
    P(CWVSliderScalar("##PCMRR",sliderSize,ImGuiDataType_U8,&ins->br1611.pcmRR,&_FIFTEEN,&_ZERO)); rightClickable
    ImGui::TableNextColumn();
    P(CWVSliderScalar("##PCMTL",sliderSize,ImGuiDataType_U8,&ins->br1611.pcmTL,&_ONE_HUNDRED_TWENTY_SEVEN,&_ZERO)); rightClickable
    ImGui::TableNextColumn();
    drawFMEnv(ins->br1611.pcmTL&127,ins->br1611.pcmAR&31,ins->br1611.pcmD1R&31,ins->br1611.pcmD2R&31,ins->br1611.pcmRR&15,ins->br1611.pcmSL&15,0,0,7,127,31,15,ImVec2(ImGui::GetContentRegionAvail().x,sliderHeight),DIV_INS_FM);
    ImGui::EndTable();
  }
  if (ImGui::Button(_("Hold while the note is on##BRPCMHold"))) {
    ins->br1611.pcmAR=31;
    ins->br1611.pcmD1R=0;
    ins->br1611.pcmSL=0;
    ins->br1611.pcmD2R=0;
    ins->br1611.pcmRR=15;
    ins->br1611.pcmTL=0;
    PARAMETER;
  }
  ImGui::Separator();
}
