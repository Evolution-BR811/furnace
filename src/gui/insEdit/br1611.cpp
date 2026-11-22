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
// Sintese WMS: os 4 operadores do OPN (algoritmos, feedback, envelopes, LFO),
// cada um tocando a sua propria onda, guardada no instrumento.
// Por Evolution BR811.

#include "insEditCommon.h"
#include "../intConst.h"
#include "../guiConst.h"
#include "misc/cpp/imgui_stdlib.h"

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

// ondas prontas para os operadores (inclui as 8 ondas do OPL3)
static const char* br1611WavePresets[]={
  _N("Sine"), _N("Half sine"), _N("Absolute sine"), _N("Quarter sine"),
  _N("Alternating sine"), _N("Camel sine"), _N("Square"), _N("Derived square"),
  _N("Triangle"), _N("Saw"), _N("Pulse 25%"), _N("Pulse 12.5%"), _N("Noise"), _N("Silence")
};

static void br1611MakePreset(DivInstrumentBR1611& b, int op, int preset) {
  int len=CLAMP(b.waveLen[op],1,256);
  if (preset==12 || preset<=8) {
    // (as ondas suaves ficam melhores com todos os pontos)
    if (len<64) len=256;
  }
  b.waveLen[op]=len;
  unsigned int seed=0x1611;
  for (int i=0; i<len; i++) {
    double x=(double)i/(double)len;
    double s=sin(2.0*M_PI*x);
    double v=0.0;
    switch (preset) {
      case 0: v=s; break;
      case 1: v=(x<0.5)?s:0.0; break;
      case 2: v=fabs(s); break;
      case 3: v=(fmod(x,0.5)<0.25)?fabs(s):0.0; break;
      case 4: v=(x<0.5)?sin(4.0*M_PI*x):0.0; break;
      case 5: v=(x<0.5)?fabs(sin(4.0*M_PI*x)):0.0; break;
      case 6: v=(x<0.5)?1.0:-1.0; break;
      case 7: {
        // a onda 7 do OPL3: quadrada com as bordas em exponencial
        double t=(x<0.5)?x:(x-0.5);
        double e=pow(2.0,-t*16.0);
        v=(x<0.5)?e:-pow(2.0,-(0.5-t)*16.0);
        break;
      }
      case 8: v=(x<0.25)?(x*4.0):((x<0.75)?(2.0-x*4.0):(x*4.0-4.0)); break;
      case 9: v=(x<0.5)?(x*2.0):(x*2.0-2.0); break;
      case 10: v=(x<0.25)?1.0:-1.0; break;
      case 11: v=(x<0.125)?1.0:-1.0; break;
      case 12:
        seed=seed*1103515245+12345;
        v=((double)((seed>>16)&0x7fff)/16383.5)-1.0;
        break;
      default: v=0.0; break;
    }
    b.waveData[op][i]=(unsigned char)CLAMP((int)round(128.0+127.0*v),0,255);
  }
}

// muda o tamanho da onda, reamostrando os pontos
static void br1611Resize(DivInstrumentBR1611& b, int op, int newLen) {
  int len=CLAMP(b.waveLen[op],1,256);
  newLen=CLAMP(newLen,1,256);
  if (newLen==len) return;
  unsigned char old[256];
  memcpy(old,b.waveData[op],256);
  for (int i=0; i<newLen; i++) {
    double pos=(double)i*(double)len/(double)newLen;
    int a=(int)pos;
    double frac=pos-(double)a;
    int c=(a+1)%len;
    a%=len;
    b.waveData[op][i]=(unsigned char)CLAMP((int)round((double)old[a]*(1.0-frac)+(double)old[c]*frac),0,255);
  }
  b.waveLen[op]=newLen;
}

static String br1611WaveToText(const DivInstrumentBR1611& b, int op, bool hex) {
  String t;
  int len=CLAMP(b.waveLen[op],1,256);
  for (int i=0; i<len; i++) {
    if (i) t+=" ";
    t+=hex?fmt::sprintf("$%.2X",b.waveData[op][i]):fmt::sprintf("%d",b.waveData[op][i]);
  }
  return t;
}

// le os valores digitados: decimal, ou hex com $ ou 0x
static bool br1611TextToWave(DivInstrumentBR1611& b, int op, const String& t) {
  std::vector<int> vals;
  size_t i=0;
  while (i<t.size()) {
    while (i<t.size() && !isalnum((unsigned char)t[i]) && t[i]!='$' && t[i]!='-') i++;
    if (i>=t.size()) break;
    bool hex=false;
    if (t[i]=='$') {
      hex=true;
      i++;
    } else if (t[i]=='0' && i+1<t.size() && (t[i+1]=='x' || t[i+1]=='X')) {
      hex=true;
      i+=2;
    }
    size_t j=i;
    while (j<t.size() && (isalnum((unsigned char)t[j]) || (j==i && t[j]=='-'))) j++;
    String num=t.substr(i,j-i);
    i=j;
    if (num.empty()) continue;
    char* endp=NULL;
    long v=strtol(num.c_str(),&endp,hex?16:10);
    if (endp==num.c_str() || *endp!=0) return false;
    vals.push_back(CLAMP((int)v,0,255));
    if (vals.size()>=256) break;
  }
  if (vals.empty()) return false;
  b.waveLen[op]=vals.size();
  for (size_t k=0; k<vals.size(); k++) b.waveData[op][k]=vals[k];
  return true;
}

// botao da onda de um operador (no cabecalho do operador, na aba FM) e o
// editor que ele abre: desenhar, ondas prontas, digitar, importar e exportar
void FurnaceGUI::br1611OpWave(DivInstrument* ins, int op, int disp) {
  DivInstrumentBR1611& b=ins->br1611;
  int len=CLAMP(b.waveLen[op],1,256);

  // pre-visualizacao pequena, que tambem abre o editor
  ImVec2 pvSize=ImVec2(48.0f*dpiScale,ImGui::GetFrameHeight());
  ImVec2 pvPos=ImGui::GetCursorScreenPos();
  bool open=ImGui::InvisibleButton("##BRWavePv",pvSize);
  ImDrawList* dl=ImGui::GetWindowDrawList();
  dl->AddRectFilled(pvPos,ImVec2(pvPos.x+pvSize.x,pvPos.y+pvSize.y),ImGui::GetColorU32(ImGui::IsItemHovered()?ImGuiCol_FrameBgHovered:ImGuiCol_FrameBg),ImGui::GetStyle().FrameRounding);
  {
    ImU32 col=ImGui::GetColorU32(uiColors[GUI_COLOR_MACRO_WAVE]);
    ImVec2 prev;
    for (int k=0; k<=24; k++) {
      int idx=(k*len/24)%len;
      ImVec2 pt=ImVec2(pvPos.x+2.0f+(pvSize.x-4.0f)*(float)k/24.0f,pvPos.y+2.0f+(pvSize.y-4.0f)*(1.0f-(float)b.waveData[op][idx]/255.0f));
      if (k>0) dl->AddLine(prev,pt,col,dpiScale);
      prev=pt;
    }
  }
  if (ImGui::IsItemHovered()) ImGui::SetTooltip(_("waveform of operator %d (click to edit)"),disp+1);
  if (open) {
    br1611WaveIns=curIns;
    br1611WaveOp=op;
    br1611WaveText=br1611WaveToText(b,op,br1611WaveHex);
    ImGui::OpenPopup("BRWaveEdit");
  }

  ImGui::SetNextWindowSizeConstraints(ImVec2(420.0f*dpiScale,0.0f),ImVec2(FLT_MAX,FLT_MAX));
  if (ImGui::BeginPopup("BRWaveEdit")) {
    ImGui::Text(_("Operator %d waveform"),disp+1);
    ImGui::SameLine();
    ImGui::TextDisabled("(%d)",len);

    // --- desenhar ---
    ImVec2 cvSize=ImVec2(MAX(400.0f*dpiScale,ImGui::GetContentRegionAvail().x),160.0f*dpiScale);
    ImVec2 cvPos=ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##BRWaveCanvas",cvSize);
    bool hov=ImGui::IsItemHovered();
    ImDrawList* dl2=ImGui::GetWindowDrawList();
    dl2->AddRectFilled(cvPos,ImVec2(cvPos.x+cvSize.x,cvPos.y+cvSize.y),ImGui::GetColorU32(ImGuiCol_FrameBg));
    dl2->AddLine(ImVec2(cvPos.x,cvPos.y+cvSize.y*0.5f),ImVec2(cvPos.x+cvSize.x,cvPos.y+cvSize.y*0.5f),ImGui::GetColorU32(ImGuiCol_Separator));
    {
      ImU32 col=ImGui::GetColorU32(uiColors[GUI_COLOR_MACRO_WAVE]);
      float cw=cvSize.x/(float)len;
      for (int k=0; k<len; k++) {
        float y=cvPos.y+cvSize.y*(1.0f-(float)b.waveData[op][k]/255.0f);
        float y2=cvPos.y+cvSize.y*(1.0f-(float)b.waveData[op][(k+1)%len]/255.0f);
        dl2->AddLine(ImVec2(cvPos.x+cw*k,y),ImVec2(cvPos.x+cw*(k+1),y),col,2.0f*dpiScale);
        if (k+1<len) dl2->AddLine(ImVec2(cvPos.x+cw*(k+1),y),ImVec2(cvPos.x+cw*(k+1),y2),col,dpiScale);
      }
    }
    if (hov && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      br1611WaveDrawing=true;
      br1611WaveLastX=-1;
    }
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) br1611WaveDrawing=false;
    if (br1611WaveDrawing) {
      ImVec2 m=ImGui::GetMousePos();
      int x=CLAMP((int)((m.x-cvPos.x)/cvSize.x*(float)len),0,len-1);
      int y=CLAMP((int)round(255.0f*(1.0f-(m.y-cvPos.y)/cvSize.y)),0,255);
      // (liga ao ponto anterior, para nao deixar buracos ao arrastar rapido)
      int x0=(br1611WaveLastX<0)?x:br1611WaveLastX;
      int y0=(br1611WaveLastX<0)?y:br1611WaveLastY;
      int steps=abs(x-x0);
      e->lockEngine([&]() {
        for (int k=0; k<=steps; k++) {
          int xx=(steps==0)?x:(x0+(x-x0)*k/steps);
          int yy=(steps==0)?y:(y0+(y-y0)*k/steps);
          b.waveData[op][xx]=yy;
        }
      });
      if (x!=br1611WaveLastX || y!=br1611WaveLastY) {
        e->notifyInsChange(curIns);
        MARK_MODIFIED;
      }
      br1611WaveLastX=x;
      br1611WaveLastY=y;
      br1611WaveText=br1611WaveToText(b,op,br1611WaveHex);
    }
    if (hov) {
      ImVec2 m=ImGui::GetMousePos();
      int x=CLAMP((int)((m.x-cvPos.x)/cvSize.x*(float)len),0,len-1);
      int y=CLAMP((int)round(255.0f*(1.0f-(m.y-cvPos.y)/cvSize.y)),0,255);
      ImGui::SetTooltip("%d: %d ($%.2X)",x,y,y);
    }

    bool changed=false;
    // --- tamanho ---
    int newLen=len;
    ImGui::SetNextItemWidth(120.0f*dpiScale);
    if (ImGui::InputInt(_("Length"),&newLen,1,16)) {
      newLen=CLAMP(newLen,1,256);
      e->lockEngine([&]() {
        br1611Resize(b,op,newLen);
      });
      changed=true;
    }
    ImGui::SameLine();
    if (ImGui::Button(_("Invert"))) {
      e->lockEngine([&]() {
        for (int k=0; k<len; k++) b.waveData[op][k]=255-b.waveData[op][k];
      });
      changed=true;
    }
    ImGui::SameLine();
    if (ImGui::Button(_("Reverse"))) {
      e->lockEngine([&]() {
        for (int k=0; k<len/2; k++) {
          unsigned char t=b.waveData[op][k];
          b.waveData[op][k]=b.waveData[op][len-1-k];
          b.waveData[op][len-1-k]=t;
        }
      });
      changed=true;
    }
    ImGui::SameLine();
    if (ImGui::Button(_("Smooth"))) {
      e->lockEngine([&]() {
        unsigned char old[256];
        memcpy(old,b.waveData[op],256);
        for (int k=0; k<len; k++) {
          b.waveData[op][k]=(old[(k+len-1)%len]+2*old[k]+old[(k+1)%len]+2)/4;
        }
      });
      changed=true;
    }

    // --- ondas prontas ---
    ImGui::Text(_("Presets:"));
    for (int pr=0; pr<14; pr++) {
      if (pr%5!=0) ImGui::SameLine();
      if (ImGui::Button(_(br1611WavePresets[pr]))) {
        e->lockEngine([&]() {
          br1611MakePreset(b,op,pr);
        });
        changed=true;
      }
    }

    // --- copiar de outro operador ---
    ImGui::Text(_("Copy from:"));
    for (int k=0; k<4; k++) {
      int src=opOrder[k];
      if (src==op) continue;
      ImGui::SameLine();
      if (ImGui::Button(fmt::sprintf(_("OP%d##BRCopyOp"),k+1).c_str())) {
        e->lockEngine([&]() {
          b.waveLen[op]=b.waveLen[src];
          memcpy(b.waveData[op],b.waveData[src],256);
        });
        changed=true;
      }
    }

    // --- importar / exportar ---
    ImGui::Separator();
    if (ImGui::Button(_("Load wavetable..."))) {
      br1611WaveIns=curIns;
      br1611WaveOp=op;
      openFileDialog(GUI_FILE_BR1611_WAVE_OPEN);
    }
    ImGui::SameLine();
    if (ImGui::Button(_("Save wavetable..."))) {
      br1611WaveIns=curIns;
      br1611WaveOp=op;
      openFileDialog(GUI_FILE_BR1611_WAVE_SAVE);
    }
    if (e->song.waveLen>0) {
      ImGui::SameLine();
      ImGui::SetNextItemWidth(90.0f*dpiScale);
      if (ImGui::InputInt("##BRFromWave",&br1611WaveFrom,1,4)) {
        br1611WaveFrom=CLAMP(br1611WaveFrom,0,e->song.waveLen-1);
      }
      ImGui::SameLine();
      if (ImGui::Button(_("Copy from wavetable"))) {
        DivWavetable* w=e->getWave(CLAMP(br1611WaveFrom,0,e->song.waveLen-1));
        if (w!=NULL && w->len>0) {
          int wl=CLAMP(w->len,1,256);
          int wmin=w->min, wmax=w->max;
          if (wmax<=wmin) wmax=wmin+1;
          e->lockEngine([&]() {
            b.waveLen[op]=wl;
            for (int k=0; k<wl; k++) {
              b.waveData[op][k]=(unsigned char)CLAMP((int)round((double)(w->data[k]-wmin)*255.0/(double)(wmax-wmin)),0,255);
            }
          });
          changed=true;
        }
      }
    }

    // --- digitar ---
    ImGui::Separator();
    if (ImGui::Checkbox(_("Hexadecimal"),&br1611WaveHex)) {
      br1611WaveText=br1611WaveToText(b,op,br1611WaveHex);
    }
    ImGui::SameLine();
    ImGui::TextDisabled(_("(values 0 to 255, center 128; hex with $ or 0x)"));
    ImGui::InputTextMultiline("##BRWaveText",&br1611WaveText,ImVec2(cvSize.x,70.0f*dpiScale));
    if (ImGui::Button(_("Apply"))) {
      DivInstrumentBR1611 tmp=b;
      if (br1611TextToWave(tmp,op,br1611WaveText)) {
        e->lockEngine([&]() {
          b.waveLen[op]=tmp.waveLen[op];
          memcpy(b.waveData[op],tmp.waveData[op],256);
        });
        changed=true;
      } else {
        showError(_("could not read the values: use numbers from 0 to 255 separated by spaces or commas."));
      }
    }
    ImGui::SameLine();
    if (ImGui::Button(_("Copy"))) {
      ImGui::SetClipboardText(br1611WaveToText(b,op,br1611WaveHex).c_str());
    }
    ImGui::SameLine();
    if (ImGui::Button(_("Paste"))) {
      const char* clip=ImGui::GetClipboardText();
      if (clip!=NULL) br1611WaveText=clip;
    }

    if (changed) {
      e->notifyInsChange(curIns);
      MARK_MODIFIED;
      br1611WaveText=br1611WaveToText(b,op,br1611WaveHex);
    }
    ImGui::EndPopup();
  }
}

void FurnaceGUI::insEditBR1611(DivInstrument* ins) {
  std::vector<FurnaceGUIMacroDesc> macroList;
  char label[64];

  // aba FM igual a do OPN; cada operador tem o botao da sua onda
  insTabFM(ins);

  // aba Sample: PCM dos canais 6 e 7 (com envelope proprio do PCM)
  insTabSample(ins);

  if (ImGui::BeginTabItem(_("FM Macros"))) {
    macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_FB),&ins->std.fbMacro,0,7,96,uiColors[GUI_COLOR_MACRO_OTHER]));
    macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_ALG),&ins->std.algMacro,0,7,96,uiColors[GUI_COLOR_MACRO_OTHER]));
    macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_FMS),&ins->std.fmsMacro,0,7,96,uiColors[GUI_COLOR_MACRO_OTHER]));
    macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_AMS),&ins->std.amsMacro,0,3,48,uiColors[GUI_COLOR_MACRO_OTHER]));
    macroList.push_back(FurnaceGUIMacroDesc(_("LFO Speed"),&ins->std.ex3Macro,0,8,96,uiColors[GUI_COLOR_MACRO_OTHER]));
    macroList.push_back(FurnaceGUIMacroDesc(_("OpMask"),&ins->std.ex4Macro,0,4,128,uiColors[GUI_COLOR_MACRO_OTHER],false,NULL,NULL,true,fmOperatorBits));

    drawMacros(macroList,macroEditStateFM,ins);
    ImGui::EndTabItem();
  }

  for (int i=0; i<4; i++) {
    snprintf(label,63,_("OP%d Macros"),i+1);
    if (ImGui::BeginTabItem(label)) {
      ImGui::PushID(i);
      int ordi=orderedOps[i];
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_TL),&ins->std.opMacros[ordi].tlMacro,0,127,128,uiColors[GUI_COLOR_MACRO_VOLUME]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_AR),&ins->std.opMacros[ordi].arMacro,0,31,64,uiColors[GUI_COLOR_MACRO_ENVELOPE]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_DR),&ins->std.opMacros[ordi].drMacro,0,31,64,uiColors[GUI_COLOR_MACRO_ENVELOPE]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_D2R),&ins->std.opMacros[ordi].d2rMacro,0,31,64,uiColors[GUI_COLOR_MACRO_ENVELOPE]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_RR),&ins->std.opMacros[ordi].rrMacro,0,15,64,uiColors[GUI_COLOR_MACRO_ENVELOPE]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_SL),&ins->std.opMacros[ordi].slMacro,0,15,64,uiColors[GUI_COLOR_MACRO_ENVELOPE]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_RS),&ins->std.opMacros[ordi].rsMacro,0,3,32,uiColors[GUI_COLOR_MACRO_OTHER]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_MULT),&ins->std.opMacros[ordi].multMacro,0,15,64,uiColors[GUI_COLOR_MACRO_OTHER]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_DT),&ins->std.opMacros[ordi].dtMacro,0,7,64,uiColors[GUI_COLOR_MACRO_PITCH]));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_AM),&ins->std.opMacros[ordi].amMacro,0,1,32,uiColors[GUI_COLOR_MACRO_OTHER],false,NULL,NULL,true));
      macroList.push_back(FurnaceGUIMacroDesc(FM_NAME(FM_SSG),&ins->std.opMacros[ordi].ssgMacro,0,4,64,uiColors[GUI_COLOR_MACRO_ENVELOPE],false,NULL,NULL,true,ssgEnvBits));

      drawMacros(macroList,macroEditStateOP[ordi],ins);
      ImGui::PopID();
      ImGui::EndTabItem();
    }
  }

  if (ImGui::BeginTabItem(_("Macros"))) {
    macroList.push_back(FurnaceGUIMacroDesc(_("Volume"),&ins->std.volMacro,0,127,160,uiColors[GUI_COLOR_MACRO_VOLUME]));
    macroList.push_back(FurnaceGUIMacroDesc(_("Arpeggio"),&ins->std.arpMacro,-120,120,160,uiColors[GUI_COLOR_MACRO_PITCH],true,NULL,macroHoverNote,false,NULL,true,ins->std.arpMacro.val));
    macroList.push_back(FurnaceGUIMacroDesc(_("Panning"),&ins->std.panLMacro,0,4,96,uiColors[GUI_COLOR_MACRO_OTHER],false,NULL,br1611PanName));
    macroList.push_back(FurnaceGUIMacroDesc(_("Pitch"),&ins->std.pitchMacro,-2048,2047,160,uiColors[GUI_COLOR_MACRO_PITCH],true,macroRelativeMode));
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
