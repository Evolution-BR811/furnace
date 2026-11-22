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

#include "gui.h"
#include "misc/cpp/imgui_stdlib.h"
#include "IconsFontAwesome4.h"
#include <fmt/printf.h>
#include <imgui.h>

void FurnaceGUI::drawSysManager() {
  if (nextWindow==GUI_WINDOW_SYS_MANAGER) {
    sysManagerOpen=true;
    ImGui::SetNextWindowFocus();
    nextWindow=GUI_WINDOW_NOTHING;
  }
  if (!sysManagerOpen) return;
  if (mobileUI) {
    patWindowPos=(portrait?ImVec2(0.0f,(mobileMenuPos*-0.65*canvasH)):ImVec2((0.16*canvasH)+0.5*canvasW*mobileMenuPos,0.0f));
    patWindowSize=(portrait?ImVec2(canvasW,canvasH-(0.16*canvasW)):ImVec2(canvasW-(0.16*canvasH),canvasH));
    ImGui::SetNextWindowPos(patWindowPos);
    ImGui::SetNextWindowSize(patWindowSize);
  } else {
    //ImGui::SetNextWindowSizeConstraints(ImVec2(440.0f*dpiScale,400.0f*dpiScale),ImVec2(canvasW,canvasH));
  }
  if (!sysAutoVolumeLoaded) {
    sysAutoVolume=e->getConfBool("sysAutoVolume",true);
    sysAutoVolumeLoaded=true;
  }
  if (ImGui::Begin("Chip Manager",&sysManagerOpen,globalWinFlags,_("Chip Manager"))) {
    ImGuiStorage* openedConfig=ImGui::GetStateStorage();
    ImGui::Checkbox(_("Preserve channel order"),&preserveChanPos);
    ImGui::SameLine();
    ImGui::Checkbox(_("Clone channel data"),&sysDupCloneChannels);
    ImGui::SameLine();
    ImGui::Checkbox(_("Clone at end"),&sysDupEnd);

    // mixagem dos chips: volume de cada chip da musica.
    // Usa o mesmo valor da janela Mixer (song.systemVol),
    // entao fica salvo no .fur e os dois lugares mostram sempre o mesmo.
    if (ImGui::CollapsingHeader(_("Chip mixing"),ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::BeginTable("ChipMixTable",3,ImGuiTableFlags_SizingStretchProp|ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn(_("Chip"),ImGuiTableColumnFlags_WidthStretch,0.4f);
        ImGui::TableSetupColumn(_("Volume"),ImGuiTableColumnFlags_WidthStretch,0.6f);
        ImGui::TableSetupColumn("##Reset",ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();
        for (int i=0; i<e->song.systemLen; i++) {
          ImGui::PushID(i);
          ImGui::TableNextRow();

          ImGui::TableNextColumn();
          ImGui::AlignTextToFramePadding();
          ImGui::TextUnformatted(e->getSystemName(e->song.system[i]));

          // volume em porcentagem (100% = padrao), sem perder a inversao de fase
          ImGui::TableNextColumn();
          float vol=fabs(e->song.systemVol[i]);
          bool inverted=e->song.systemVol[i]<0;
          float pct=vol*100.0f;
          ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
          if (ImGui::SliderFloat("##MixVol",&pct,0.0f,200.0f,"%.0f%%")) {
            if (pct<0.0f) pct=0.0f;
            if (pct>1000.0f) pct=1000.0f;
            vol=pct/100.0f;
            if (inverted && vol<0.0001f) vol=0.0001f;
            e->song.systemVol[i]=inverted?-vol:vol;
            MARK_MODIFIED;
          } rightClickable
          if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
            if (vol>0.0001f) {
              ImGui::SetTooltip(_("Chip volume in the song mix: %+.1f dB (100%% = default).\nCtrl+click or right-click to type a value."),20.0f*log10f(vol));
            } else {
              ImGui::SetTooltip(_("Chip volume in the song mix: muted (100%% = default).\nCtrl+click or right-click to type a value."));
            }
          }

          ImGui::TableNextColumn();
          if (ImGui::Button(ICON_FA_UNDO "##MixReset")) {
            e->song.systemVol[i]=inverted?-1.0f:1.0f;
            MARK_MODIFIED;
          }
          if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(_("Reset to 100%%"));
          }
          ImGui::PopID();
        }
        ImGui::EndTable();
      }
    }

    // this is a "rack" style chip list
    int dispatchOff=0;
    for (int i=0; i<e->song.systemLen; i++) {
      String rackID=fmt::sprintf("SysEntry%d",i);
      String rackNameID=fmt::sprintf("SysName%d",i);
      const DivSysDef* sysDef=e->getSystemDef(e->song.system[i]);

      ImGui::PushID(i);
      if (ImGui::BeginChild(rackID.c_str(),ImVec2(0,0),ImGuiChildFlags_Borders|ImGuiChildFlags_AutoResizeY)) {
        // swap handle and name
        if (ImGui::Button(ICON_FA_ARROWS)) {
        }
        if (ImGui::BeginDragDropSource()) {
          sysToMove=i;
          ImGui::SetDragDropPayload("FUR_SYS",NULL,0,ImGuiCond_Once);
          ImGui::Button(ICON_FA_ARROWS "##SysDrag");
          ImGui::EndDragDropSource();
        } else if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip(_("(drag to swap chips)"));
        }
        if (ImGui::BeginDragDropTarget()) {
          const ImGuiPayload* dragItem=ImGui::AcceptDragDropPayload("FUR_SYS");
          if (dragItem!=NULL) {
            if (dragItem->IsDataType("FUR_SYS")) {
              if (sysToMove!=i && sysToMove>=0) {
                e->swapSystem(sysToMove,i,preserveChanPos);
                MARK_MODIFIED;
              }
              sysToMove=-1;
            }
          }
          ImGui::EndDragDropTarget();
        }
        ImGui::SameLine();
        float buttonInnerSize=ImGui::CalcTextSize(ICON_FA_CLONE).x;
        float buttonCount=settings.rackShowLEDs?4.0f:5.0f;
        float sideButtonSize=ImGui::GetStyle().ItemSpacing.x*buttonCount+buttonInnerSize*buttonCount+ImGui::GetStyle().FramePadding.x*2*buttonCount;
        ImGui::AlignTextToFramePadding();
        ImGui::ScrollText(ImGui::GetID(rackNameID.c_str()),sysDef->name,ImVec2(0.0f,0.0f),ImVec2(ImGui::GetContentRegionAvail().x-sideButtonSize,0),false);
        ImGui::Dummy(ImVec2(ImGui::GetContentRegionAvail().x-sideButtonSize,1.0f));
        // action buttons
        ImGui::SameLine();
        if (ImGui::Button(ICON_FA_CLONE "##SysDup")) {
          if (!e->duplicateSystem(i,sysDupCloneChannels,sysDupEnd)) {
            showError(fmt::sprintf(_("cannot clone chip! (%s)"),e->getLastError()));
          } else {
            if (e->song.autoSystem) {
              autoDetectSystem();
              updateWindowTitle();
            }
            updateROMExportAvail();
            MARK_MODIFIED;
          }
        }
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip(_("Clone"));
        }
        ImGui::SameLine();
        // ajuste automatico de volume na troca de chip (liga/desliga, vale para todos os chips)
        pushToggleColors(sysAutoVolume);
        if (ImGui::Button(ICON_FA_MAGIC "##SysAutoVol")) {
          sysAutoVolume=!sysAutoVolume;
          e->setConf("sysAutoVolume",sysAutoVolume);
        }
        popToggleColors();
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip(sysAutoVolume?
            _("Automatic volume adjust: ON\nwhen changing this chip, the volume column, the volume macros\nand the chip mix are converted to the new chip's volume curve."):
            _("Automatic volume adjust: OFF\nwhen changing this chip, volumes are kept as they are."));
        }
        ImGui::SameLine();
        ImGui::Button(ICON_FA_EJECT "##SysChange");
        if (ImGui::BeginPopupContextItem("SysPickerC",ImGuiPopupFlags_MouseButtonLeft)) {
          DivSystem picked=systemPicker(false);
          if (picked!=DIV_SYSTEM_NULL) {
            bool changed=false;
            String volReport;
            if (sysAutoVolume) {
              changed=e->changeSystemAdaptVolume(i,picked,preserveChanPos,volReport);
            } else {
              changed=e->changeSystem(i,picked,preserveChanPos);
            }
            if (changed) {
              if (!volReport.empty()) showWarning(volReport,GUI_WARN_GENERIC);
              MARK_MODIFIED;
              recalcTimestamps=true;
              if (e->song.autoSystem) {
                autoDetectSystem();
              }
              updateWindowTitle();
              updateROMExportAvail();
            } else {
              showError(fmt::sprintf(_("cannot change chip! (%s)"),e->getLastError()));
            }
            ImGui::CloseCurrentPopup();
          }
          if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            ImGui::CloseCurrentPopup();
          }
          ImGui::EndPopup();
        }
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip(_("Change"));
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(e->song.systemLen<=1);
        pushDestColor();
        if (ImGui::Button(ICON_FA_TIMES "##SysRemove")) {
          sysToDelete=i;
          showWarning(_("Are you sure you want to remove this chip?"),GUI_WARN_SYSTEM_DEL);
        }
        popDestColor();
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip(_("Remove"));
        }
        ImGui::EndDisabled();

        // channel LEDs and chip config button
        float height=0;
        if (settings.rackShowLEDs) {
          height=drawSystemChannelInfo(sysDef,dispatchOff,ImGui::GetContentRegionAvail().x-(ImGui::CalcTextSize(ICON_FA_CHEVRON_DOWN).x+ImGui::GetStyle().ItemSpacing.x),e->song.systemChans[i]);
        }

        ImGuiID openedID=ImGui::GetID("OpenSysConfig");
        bool opened=openedConfig->GetBool(openedID,false);
        ImGui::SameLine();
        ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign,ImVec2(0.5f,0.5f));
        if (ImGui::Selectable(opened?(ICON_FA_CHEVRON_UP "###OpenThing"):(ICON_FA_CHEVRON_DOWN "###OpenThing"),false,0,ImVec2(0,height))) {
          opened=!opened;
          openedConfig->SetBool(openedID,opened);
        }
        ImGui::PopStyleVar();

        if (opened) {
          ImGui::Separator();
          ImGui::Indent();
          if (drawSysConf(i,i,e->song.system[i],e->song.systemFlags[i],e->song.systemChans[i],true)) MARK_MODIFIED
          ImGui::Unindent();
        }
      }
      ImGui::EndChild();
      ImGui::PopID();

      dispatchOff+=e->song.systemChans[i];
    }

    if (e->song.systemLen<DIV_MAX_CHIPS) {
      ImGui::Button(ICON_FA_PLUS "##SysAdd");
      if (ImGui::BeginPopupContextItem("SysPickerA",ImGuiPopupFlags_MouseButtonLeft)) {
        DivSystem picked=systemPicker(false);
        if (picked!=DIV_SYSTEM_NULL) {
          if (!e->addSystem(picked)) {
            showError(fmt::sprintf(_("cannot add chip! (%s)"),e->getLastError()));
          } else {
            MARK_MODIFIED;
            recalcTimestamps=true;
          }
          if (e->song.autoSystem) {
            autoDetectSystem();
          }
          updateWindowTitle();
          updateROMExportAvail();
          ImGui::CloseCurrentPopup();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
      }
    }
  }
  if (ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)) curWindow=GUI_WINDOW_SYS_MANAGER;
  ImGui::End();
}
