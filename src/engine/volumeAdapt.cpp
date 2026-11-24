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

// Adaptacao automatica de volume na troca de chip - por Evolution BR811
//
// Quando um chip e trocado por outro, a coluna de volume e as macros de volume
// continuam com os numeros do chip antigo, que tem outra faixa (ex.: 0-127 no
// YM2612, 0-15 no SN76489) e outra curva (passos de 0,75 dB, de 2 dB, linear...).
//
// Em vez de uma tabela escrita a mao para cada chip, o volume e calibrado medindo:
// um chip escondido (fora da musica) toca A-4 em cada volume e o nivel e medido
// em dB. Com a curva do chip antigo e a do novo, cada valor de volume vira o valor
// do chip novo que soa com o mesmo nivel relativo ao volume maximo. Por fim o nivel
// do chip na mixagem (Chip mixing) e ajustado para compensar a diferenca entre os
// volumes maximos dos dois chips.

#include "engine.h"
#include "../ta-log.h"
#include <math.h>
#include <fmt/printf.h>

#define VOLADAPT_RATE 44100.0
#define VOLADAPT_TICK 735
#define VOLADAPT_NOTE 117   // A-4
#define VOLADAPT_MAXPOINTS 48

bool DivEngine::calibrateVolumeCurve(DivSystem sys, const DivConfig& flags, int chans, std::vector<DivVolumeCurve>& curves) {
  curves.clear();
  DivDispatchContainer dc;
  dc.init(sys,this,chans,VOLADAPT_RATE,flags,true);
  if (dc.dispatch==NULL) return false;
  dc.grow(8192);
  dc.setRates(VOLADAPT_RATE);
  dc.setQuality(false,false);
  dc.clear();

  int nch=getChannelCount(sys);
  if (chans>0 && chans<nch) nch=chans;
  int outs=dc.dispatch->getOutputCount();
  if (outs<1) outs=1;
  if (outs>DIV_MAX_OUTPUTS) outs=DIV_MAX_OUTPUTS;

  // roda um tick do chip e devolve a energia media (sem nivel DC) da saida
  auto runTick=[&]() -> double {
    dc.dispatch->tick();
    if (dc.bb[0]==NULL) return 0.0;
    int total=blip_clocks_needed(dc.bb[0],VOLADAPT_TICK);
    if (total>(int)dc.bbInLen) {
      dc.grow(total+256);
      dc.setRates(VOLADAPT_RATE);
    }
    dc.acquire(total);
    dc.fillBuf(total,0,VOLADAPT_TICK);
    double e=0.0;
    for (int o=0; o<outs; o++) {
      if (dc.bbOut[o]==NULL) continue;
      double m=0.0;
      for (int j=0; j<VOLADAPT_TICK; j++) m+=dc.bbOut[o][j];
      m/=VOLADAPT_TICK;
      for (int j=0; j<VOLADAPT_TICK; j++) {
        double d=dc.bbOut[o][j]-m;
        e+=d*d;
      }
    }
    // media das saidas: um chip estereo nao conta como "mais alto" que um mono
    return e/((double)VOLADAPT_TICK*outs);
  };
  // ganho que o mixer do Furnace aplica a este chip
  double postAmp=dc.dispatch->getPostAmp();
  if (postAmp<=0.0) postAmp=1.0;

  for (int ch=0; ch<nch; ch++) {
    DivVolumeCurve c;
    c.maxVol=dc.dispatch->dispatch(DivCommand(DIV_CMD_GET_VOLMAX,ch));
    c.ok=false;
    c.maxLevel=0.0;
    if (c.maxVol<1 || c.maxVol>65535) {
      curves.push_back(c);
      continue;
    }

    // pontos medidos (todos, ou ate VOLADAPT_MAXPOINTS espalhados)
    std::vector<int> pts;
    if (c.maxVol<=VOLADAPT_MAXPOINTS) {
      for (int v=0; v<=c.maxVol; v++) pts.push_back(v);
    } else {
      for (int k=0; k<=VOLADAPT_MAXPOINTS; k++) {
        int v=(int)round((double)k*c.maxVol/VOLADAPT_MAXPOINTS);
        if (pts.empty() || pts.back()!=v) pts.push_back(v);
      }
    }

    std::vector<double> lev(pts.size(),0.0);
    for (size_t k=0; k<pts.size(); k++) {
      dc.dispatch->dispatch(DivCommand(DIV_CMD_VOLUME,ch,pts[k]));
      dc.dispatch->dispatch(DivCommand(DIV_CMD_NOTE_ON,ch,VOLADAPT_NOTE));
      double acc=0.0;
      for (int t=0; t<6; t++) {
        double e=runTick();
        if (t>=2) acc+=e;
      }
      lev[k]=acc/4.0;
      dc.dispatch->dispatch(DivCommand(DIV_CMD_NOTE_OFF,ch));
      for (int t=0; t<6; t++) runTick();
    }

    // interpola para todos os volumes
    double top=lev.back();
    for (double l: lev) if (l>top) top=l;
    c.maxLevel=sqrt(top)*postAmp;
    if (top<1.0) {
      // chip mudo nesta medicao (ex.: canal que so toca sample)
      curves.push_back(c);
      continue;
    }
    std::vector<double> L(c.maxVol+1,0.0);
    for (int v=0; v<=c.maxVol; v++) {
      size_t k=0;
      while (k+1<pts.size() && pts[k+1]<v) k++;
      if (k+1>=pts.size() || pts[k]==v) {
        L[v]=lev[k];
      } else {
        double f=(double)(v-pts[k])/(double)(pts[k+1]-pts[k]);
        L[v]=lev[k]+(lev[k+1]-lev[k])*f;
      }
    }
    // volume nunca soa mais alto com um valor menor (canais de ruido medem com leve oscilacao)
    for (int v=c.maxVol-1; v>=0; v--) if (L[v]>L[v+1]) L[v]=L[v+1];

    // dB relativos ao volume maximo
    c.db.assign(c.maxVol+1,-200.0);
    for (int v=1; v<=c.maxVol; v++) if (L[v]>0.0) c.db[v]=10.0*log10(L[v]/top);

    // patamar: alguns chips (FM) param de baixar a partir de um certo volume (ruido do
    // proprio chip) e depois ficam mudos. abaixo do ponto onde a curva "achata", ela e
    // prolongada com a inclinacao medida na parte confiavel de cima.
    if (c.maxVol>=8) {
      int pstart=-1;
      for (int v=c.maxVol-1; v>=4; v--) {
        if (c.db[v]>-12.0) continue;
        double upper=(0.0-c.db[v])/(double)(c.maxVol-v);
        double local=(c.db[v]-c.db[v-3])/3.0;
        if (local<0.25*upper) { pstart=v; break; }
      }
      if (pstart>0) {
        int vr=c.maxVol;
        for (int v=c.maxVol; v>=1; v--) {
          if (c.db[v]>c.db[pstart]+6.0) vr=v; else break;
        }
        if (vr<c.maxVol) {
          double slope=(0.0-c.db[vr])/(double)(c.maxVol-vr);
          if (slope<0.05) slope=0.05;
          for (int v=1; v<vr; v++) c.db[v]=c.db[vr]-slope*(vr-v);
        }
      }
    }
    for (int v=c.maxVol-1; v>=1; v--) if (c.db[v]>c.db[v+1]) c.db[v]=c.db[v+1];
    c.db[0]=-200.0;
    // a curva precisa subir com o volume; se nao subir, a medicao nao serve
    if (c.db[c.maxVol]>-1.0 && c.db[0]<c.db[c.maxVol]-6.0) c.ok=true;
    curves.push_back(c);
  }

  dc.quit();
  return true;
}

// converte um valor de volume pela curva medida (ou proporcionalmente, se nao houver curva)
static int adaptVolumeValue(int v, const DivVolumeCurve& from, const DivVolumeCurve& to) {
  if (v<=0) return 0;
  if (from.maxVol<1 || to.maxVol<1) return v;
  if (v>=from.maxVol) return to.maxVol;
  if (!from.ok || !to.ok) {
    int r=(int)round((double)v*to.maxVol/from.maxVol);
    if (r<1) r=1;
    return r;
  }
  double target=from.db[v];
  int best=1;
  double bestErr=1e9;
  for (int w=1; w<=to.maxVol; w++) {
    double err=fabs(to.db[w]-target);
    if (err<bestErr) { bestErr=err; best=w; }
  }
  return best;
}

bool DivEngine::changeSystemAdaptVolume(int index, DivSystem which, bool preserveOrder, String& report) {
  report="";
  if (index<0 || index>=song.systemLen) {
    lastError=_("invalid index");
    return false;
  }
  DivSystem oldSys=song.system[index];
  DivConfig oldFlags=song.systemFlags[index];
  int oldChans=song.systemChans[index];

  // 1. curva do chip antigo (com as configuracoes dele)
  std::vector<DivVolumeCurve> oldCurves;
  bool oldOk=calibrateVolumeCurve(oldSys,oldFlags,oldChans,oldCurves);

  // 2. troca o chip normalmente
  if (!changeSystem(index,which,preserveOrder)) return false;

  // 3. curva do chip novo
  std::vector<DivVolumeCurve> newCurves;
  bool newOk=calibrateVolumeCurve(which,song.systemFlags[index],song.systemChans[index],newCurves);
  if (!oldOk || !newOk || oldCurves.empty() || newCurves.empty()) {
    report=_("the chip was changed, but its volume could not be measured. volumes were left as they were.");
    return true;
  }

  BUSY_BEGIN;
  saveLock.lock();

  int firstChan=-1;
  for (int i=0; i<song.chans; i++) {
    if (song.dispatchOfChan[i]==index) { firstChan=i; break; }
  }
  int newChans=song.systemChans[index];
  int convChans=MIN(MIN(oldChans,newChans),MIN((int)oldCurves.size(),(int)newCurves.size()));

  int volChanged=0, macrosChanged=0, macrosShared=0;
  if (firstChan>=0) {
    // instrumentos usados dentro e fora do chip convertido
    std::vector<int> insChan(song.ins.size(),-1);
    std::vector<bool> insOutside(song.ins.size(),false);
    for (DivSubSong* sub: song.subsong) {
      for (int ch=0; ch<song.chans; ch++) {
        bool inside=(ch>=firstChan && ch<firstChan+convChans);
        for (int p=0; p<DIV_MAX_PATTERNS; p++) {
          DivPattern* pat=sub->pat[ch].data[p];
          if (pat==NULL) continue;
          for (int r=0; r<DIV_MAX_ROWS; r++) {
            int ins=pat->newData[r][DIV_PAT_INS];
            if (ins<0 || ins>=(int)song.ins.size()) continue;
            if (inside) {
              if (insChan[ins]<0) insChan[ins]=ch-firstChan;
            } else {
              insOutside[ins]=true;
            }
          }
        }
      }
    }

    // coluna de volume
    for (DivSubSong* sub: song.subsong) {
      for (int k=0; k<convChans; k++) {
        int ch=firstChan+k;
        for (int p=0; p<DIV_MAX_PATTERNS; p++) {
          DivPattern* pat=sub->pat[ch].data[p];
          if (pat==NULL) continue;
          for (int r=0; r<DIV_MAX_ROWS; r++) {
            short& v=pat->newData[r][DIV_PAT_VOL];
            if (v<0) continue;
            int nv=adaptVolumeValue(v,oldCurves[k],newCurves[k]);
            if (nv!=v) { v=nv; volChanged++; }
          }
        }
      }
    }

    // macros de volume dos instrumentos usados so neste chip
    for (size_t i=0; i<song.ins.size(); i++) {
      if (insChan[i]<0) continue;
      if (insOutside[i]) { macrosShared++; continue; }
      DivInstrumentMacro& m=song.ins[i]->std.volMacro;
      if (m.len<1) continue;
      const DivVolumeCurve& from=oldCurves[insChan[i]];
      const DivVolumeCurve& to=newCurves[insChan[i]];
      bool fits=true;
      for (int j=0; j<m.len; j++) if (m.val[j]<0 || m.val[j]>from.maxVol) fits=false;
      if (!fits) continue;
      for (int j=0; j<m.len; j++) m.val[j]=adaptVolumeValue(m.val[j],from,to);
      macrosChanged++;
    }
  }

  // nivel do chip na mixagem: compensa a diferenca entre os volumes maximos
  double ratioSum=0.0;
  int ratioN=0;
  for (int k=0; k<convChans; k++) {
    if (!oldCurves[k].ok || !newCurves[k].ok) continue;
    if (newCurves[k].maxLevel<=0.0) continue;
    ratioSum+=log(oldCurves[k].maxLevel/newCurves[k].maxLevel);
    ratioN++;
  }
  float mixBefore=fabs(song.systemVol[index]);
  float mixAfter=mixBefore;
  if (ratioN>0) {
    double ratio=exp(ratioSum/ratioN);
    if (ratio<0.25) ratio=0.25;
    if (ratio>4.0) ratio=4.0;
    mixAfter=mixBefore*ratio;
    if (mixAfter>10.0f) mixAfter=10.0f;
    song.systemVol[index]=(song.systemVol[index]<0)?-mixAfter:mixAfter;
  }

  saveLock.unlock();
  BUSY_END;

  report=fmt::sprintf(_("volumes adapted to the new chip:\n- %d volume column values\n- %d instrument volume macros\n- chip mix: %.0f%% -> %.0f%%"),volChanged,macrosChanged,mixBefore*100.0f,mixAfter*100.0f);
  if (macrosShared>0) {
    report+=fmt::sprintf(_("\n%d instruments are also used by other chips, so their macros were left as they were."),macrosShared);
  }
  logI("volume adapt: %d values, %d macros, mix %.2f -> %.2f",volChanged,macrosChanged,mixBefore,mixAfter);
  return true;
}
