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

// GSF/MiniGSF importer (Game Boy Advance), by Evolution BR811.
//
// a GSF is a PSF (version 0x22) holding a piece of a Game Boy Advance ROM; a
// minigsf only holds a patch (the song number) and names the .gsflib, the
// game's ROM, next to it. most GBA games play their music with Nintendo's
// MP2K driver (also called "Sappy"/m4a): this importer reads the song from
// the ROM the way the driver plays it (its sequence, its voices, its tempo)
// and writes it as a Furnace song:
// - the driver's sample voices (DirectSound) go to the GBA MinMod chip (a
//   software mixer, like the driver's), one channel per voice the song
//   plays at once on each track, with the driver's envelope as the volume
//   macro (it runs once per frame, as Furnace's macros do here)
// - its Game Boy voices (pulse 1, pulse 2, wave, noise) go to the Game Boy
// - its tempo is Furnace's virtual tempo: the driver adds the tempo to a
//   counter every frame and plays a tick every 150, which is what Furnace's
//   virtual tempo does (59.7275 Hz, numerator/denominator), so a tick of the
//   song is a tick of the driver

#include "fileOpsCommon.h"
#include "../../fileutils.h"
#include <algorithm>
#include <math.h>
#include <map>
#include <set>
#include <vector>

namespace {

struct GSFError {
  String msg;
  GSFError(const String& m): msg(m) {}
};

// a PSF: its program section (offset in the GBA's memory, data) and its tags
struct GSFPart {
  unsigned int offset;
  std::vector<unsigned char> data;
  std::map<String,String> tags;
};

bool gsfInflate(const unsigned char* in, size_t len, std::vector<unsigned char>& out) {
  z_stream zs;
  memset(&zs,0,sizeof(zs));
  if (inflateInit(&zs)!=Z_OK) return false;
  zs.next_in=(Bytef*)in;
  zs.avail_in=len;
  unsigned char buf[65536];
  int ret=Z_OK;
  do {
    zs.next_out=buf;
    zs.avail_out=sizeof(buf);
    ret=inflate(&zs,Z_NO_FLUSH);
    if (ret!=Z_OK && ret!=Z_STREAM_END) {
      inflateEnd(&zs);
      return false;
    }
    out.insert(out.end(),buf,buf+(sizeof(buf)-zs.avail_out));
    if (out.size()>64*1024*1024) {
      inflateEnd(&zs);
      return false;
    }
  } while (ret!=Z_STREAM_END && zs.avail_in>0);
  inflateEnd(&zs);
  return true;
}

void gsfParse(const unsigned char* f, size_t len, GSFPart& p) {
  if (len<16 || memcmp(f,"PSF",3)!=0) throw GSFError("not a PSF file");
  if (f[3]!=0x22) throw GSFError("this PSF is not for the Game Boy Advance");
  unsigned int rl=f[4]|(f[5]<<8)|(f[6]<<16)|((unsigned int)f[7]<<24);
  unsigned int pl=f[8]|(f[9]<<8)|(f[10]<<16)|((unsigned int)f[11]<<24);
  if ((size_t)16+rl+pl>len) throw GSFError("the file is cut short");
  p.offset=0;
  p.data.clear();
  if (pl>0) {
    std::vector<unsigned char> prog;
    if (!gsfInflate(f+16+rl,pl,prog)) throw GSFError("its program can't be decompressed");
    if (prog.size()>=12) {
      unsigned int off=prog[4]|(prog[5]<<8)|(prog[6]<<16)|((unsigned int)prog[7]<<24);
      unsigned int sz=prog[8]|(prog[9]<<8)|(prog[10]<<16)|((unsigned int)prog[11]<<24);
      if ((size_t)sz>prog.size()-12) sz=prog.size()-12;
      p.offset=off;
      p.data.assign(prog.begin()+12,prog.begin()+12+sz);
    }
  }
  // the tags: "[TAG]" and then name=value lines
  size_t t=16+rl+pl;
  if (t+5<=len && memcmp(f+t,"[TAG]",5)==0) {
    String s((const char*)f+t+5,len-t-5);
    size_t pos=0;
    while (pos<s.size()) {
      size_t e=s.find('\n',pos);
      if (e==String::npos) e=s.size();
      String line=s.substr(pos,e-pos);
      pos=e+1;
      size_t eq=line.find('=');
      if (eq==String::npos) continue;
      String k=line.substr(0,eq), v=line.substr(eq+1);
      while (!k.empty() && (k.back()==' ' || k.back()=='\r')) k.pop_back();
      while (!k.empty() && k[0]==' ') k.erase(0,1);
      while (!v.empty() && (v.back()==' ' || v.back()=='\r')) v.pop_back();
      while (!v.empty() && v[0]==' ') v.erase(0,1);
      for (char& c: k) if (c>='A' && c<='Z') c+='a'-'A';
      if (p.tags.count(k) && k!="_lib") {
        p.tags[k]+="\n"+v;
      } else {
        p.tags[k]=v;
      }
    }
  }
}

bool gsfReadFile(const String& path, std::vector<unsigned char>& out) {
  FILE* fl=ps_fopen(path.c_str(),"rb");
  if (fl==NULL) return false;
  fseek(fl,0,SEEK_END);
  long sz=ftell(fl);
  fseek(fl,0,SEEK_SET);
  if (sz<=0 || sz>64*1024*1024) {
    fclose(fl);
    return false;
  }
  out.resize(sz);
  bool ok=(fread(out.data(),1,sz,fl)==(size_t)sz);
  fclose(fl);
  return ok;
}

// the GBA's memory the song lives in (ROM at 0x08000000, or a multiboot
// game's work RAM at 0x02000000), put together from the library and patches
struct GSFMem {
  unsigned int region=0x08;
  std::vector<unsigned char> m;
  unsigned int mask() const {
    return (region==0x02)?0x3ffff:0x1ffffff;
  }
  void put(unsigned int off, const std::vector<unsigned char>& d) {
    if (d.empty()) return;
    size_t p=off&mask();
    if (m.size()<p+d.size()) m.resize(p+d.size(),0);
    memcpy(m.data()+p,d.data(),d.size());
  }
  unsigned char u8(size_t a) const {
    return (a<m.size())?m[a]:0;
  }
  unsigned int u16(size_t a) const {
    return u8(a)|(u8(a+1)<<8);
  }
  unsigned int u32(size_t a) const {
    return u8(a)|(u8(a+1)<<8)|(u8(a+2)<<16)|((unsigned int)u8(a+3)<<24);
  }
  // a pointer (a GBA address) to a place in it, -1: not in it
  long long ptr(size_t a) const {
    unsigned int v=u32(a);
    if ((v>>24)!=region) return -1;
    size_t p=v&mask();
    if (p>=m.size()) return -1;
    return (long long)p;
  }
};

// the MP2K driver's note lengths (and waits): N01-N96, W00-W96
const int mpLen[49]={
  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,
  28,30,32,36,40,42,44,48,52,54,56,60,64,66,68,72,76,78,80,84,88,90,92,96
};

// the start of the driver's m4aSongNumStart (the song table is in its
// literal pool, 40 bytes on)
const unsigned char mpSelectSong[30]={
  0x00,0xB5,0x00,0x04,0x07,0x4A,0x08,0x49,0x40,0x0B,0x40,0x18,0x83,0x88,0x59,0x00,
  0xC9,0x18,0x89,0x00,0x89,0x18,0x0A,0x68,0x01,0x68,0x10,0x1C,0x00,0xF0
};

// what a track does, tick by tick
struct MPNote {
  int track, start, len;
  int key, vel;
  int prog;
  // the track's state when it starts
  int vol, pan, keysh, tune, bend, bendr, mod, lfos, lfodl, modt;
};
struct MPCtrl {
  int track, tick;
  // 0 vol, 1 pan, 2 bend, 3 bendr, 4 tune, 5 keysh, 6 mod, 7 lfos, 8 lfodl, 9 modt, 10 voice
  int kind, val;
};
struct MPTrackRun {
  int endTick=0;
  int loopTick=-1;
  bool loops=false;
};

class MPSong {
  const GSFMem& M;
  public:
  std::vector<MPNote> notes;
  std::vector<MPCtrl> ctrls;
  std::vector<std::pair<int,int>> tempos; // (tick, tempo argument: BPM/2)
  int numTracks=0;
  long long voiceGroup=-1;
  int reverb=0;
  MPSong(const GSFMem& mem): M(mem) {}

  // play one track up to the tick "until" (-1: until it ends or loops for
  // the first time); what it does goes to the lists if "record" is on
  MPTrackRun run(int tr, long long start, int until, bool record) {
    MPTrackRun res;
    long long a=start;
    int t=0;
    int last=-1;
    int key=60, vel=127, gate=0;
    std::vector<long long> stack;
    std::map<long long,int> reptCount;
    // the tick each place of the track was first played at (for its loop)
    std::map<long long,int> firstAt;
    int vol=100, pan=0, keysh=0, tune=0, bend=0, bendr=2, mod=0, lfos=22, lfodl=0, modt=0, prog=0;
    // ties: key -> the note that is held
    std::map<int,size_t> ties;
    int guard=0;
    while (true) {
      if (++guard>4000000) break;
      if (until>=0 && t>=until) break;
      if (a<0 || a>=(long long)M.m.size()) break;
      if (stack.empty() && !firstAt.count(a)) firstAt[a]=t;
      int c=M.u8(a);
      int cmd;
      if (c<0x80) {
        cmd=last;
        if (cmd<0) break;
      } else {
        cmd=c;
        a++;
      }
      if (cmd>=0x80 && cmd<=0xb0) {
        t+=mpLen[cmd-0x80];
        continue;
      }
      if (cmd==0xb1) {
        break;
      }
      if (cmd==0xb2) {
        long long dst=M.ptr(a);
        if (dst<0) break;
        if (!res.loops) {
          res.loops=true;
          res.endTick=t;
          auto it=firstAt.find(dst);
          res.loopTick=(it!=firstAt.end())?it->second:0;
          if (until<0) return res;
        }
        a=dst;
        continue;
      }
      if (cmd==0xb3) {
        long long dst=M.ptr(a);
        if (dst<0 || stack.size()>=3) {
          a+=4;
          continue;
        }
        stack.push_back(a+4);
        a=dst;
        continue;
      }
      if (cmd==0xb4) {
        if (!stack.empty()) {
          a=stack.back();
          stack.pop_back();
        }
        continue;
      }
      if (cmd==0xb5) {
        int n=M.u8(a);
        long long dst=M.ptr(a+1);
        int& rc=reptCount[a];
        rc++;
        if (dst>=0 && (n==0 || rc<n)) {
          if (n==0 && until<0) {
            // (repeating forever is a loop)
            res.loops=true;
            res.endTick=t;
            auto it=firstAt.find(dst);
            res.loopTick=(it!=firstAt.end())?it->second:0;
            return res;
          }
          a=dst;
        } else {
          a+=5;
          rc=0;
        }
        continue;
      }
      if (cmd==0xb9) {
        a+=3;
        continue;
      }
      if (cmd==0xcd) {
        a+=2;
        continue;
      }
      if (cmd>=0xba && cmd<=0xcc) {
        int v=M.u8(a);
        a++;
        if (cmd>=0xbd) last=cmd;
        int kind=-1, val=v;
        switch (cmd) {
          case 0xbb:
            if (record) tempos.push_back(std::make_pair(t,v));
            break;
          case 0xbc: keysh=(signed char)v; kind=5; val=keysh; break;
          case 0xbd: prog=v; kind=10; break;
          case 0xbe: vol=v&127; kind=0; val=vol; break;
          case 0xbf: pan=(v&127)-64; kind=1; val=pan; break;
          case 0xc0: bend=(v&127)-64; kind=2; val=bend; break;
          case 0xc1: bendr=v; kind=3; break;
          case 0xc2: lfos=v; kind=7; break;
          case 0xc3: lfodl=v; kind=8; break;
          case 0xc4: mod=v; kind=6; break;
          case 0xc5: modt=v; kind=9; break;
          case 0xc8: tune=(v&127)-64; kind=4; val=tune; break;
          default: break;
        }
        if (record && kind>=0) {
          MPCtrl cc;
          cc.track=tr;
          cc.tick=t;
          cc.kind=kind;
          cc.val=val;
          ctrls.push_back(cc);
        }
        continue;
      }
      if (cmd==0xce || cmd==0xcf || cmd>=0xd0) {
        last=cmd;
        if (M.u8(a)<0x80) {
          key=M.u8(a);
          a++;
          if (M.u8(a)<0x80) {
            vel=M.u8(a);
            a++;
            if (cmd>=0xd0 && M.u8(a)<0x80) {
              gate=M.u8(a);
              a++;
            }
          }
        }
        if (cmd==0xce) {
          // end of a tie: that key (or every held one)
          if (record) {
            auto it=ties.find(key);
            if (it!=ties.end()) {
              notes[it->second].len=t-notes[it->second].start;
              ties.erase(it);
            } else if (c>=0x80) {
              for (auto& x: ties) notes[x.second].len=t-notes[x.second].start;
              ties.clear();
            }
          }
          continue;
        }
        if (record) {
          MPNote n;
          n.track=tr;
          n.start=t;
          n.len=(cmd==0xcf)?-1:(mpLen[cmd-0xcf]+gate);
          n.key=key;
          n.vel=vel;
          n.prog=prog;
          n.vol=vol;
          n.pan=pan;
          n.keysh=keysh;
          n.tune=tune;
          n.bend=bend;
          n.bendr=bendr;
          n.mod=mod;
          n.lfos=lfos;
          n.lfodl=lfodl;
          n.modt=modt;
          if (cmd==0xcf) {
            auto it=ties.find(key);
            if (it!=ties.end()) notes[it->second].len=t-notes[it->second].start;
            ties[key]=notes.size();
          }
          notes.push_back(n);
        }
        if (cmd>=0xd0) gate=0;
        continue;
      }
      // (0xb6-0xb8 and 0xc6/0xc7/0xc9-0xcc are not used: stop)
      break;
    }
    // (ties still held at the end: until it)
    if (record) for (auto& x: ties) if (notes[x.second].len<0) notes[x.second].len=std::max(1,t-notes[x.second].start);
    if (!res.loops) res.endTick=t;
    return res;
  }
};

// the driver's release, from a level, appended frame by frame: a sample
// voice's falls by re/256 each frame, a Game Boy voice's a step every re
// frames (0: at once)
void mpRelease(bool gb, int level, int re, std::vector<int>& seq) {
  if (gb) {
    re&=7;
    int env=level;
    if (re==0 || env==0) {
      seq.push_back(0);
      return;
    }
    while (env>0 && (int)seq.size()<254) {
      for (int k=0; k<re-1 && (int)seq.size()<254; k++) seq.push_back(env);
      env--;
      seq.push_back(env);
    }
    if (seq.back()!=0) seq.push_back(0);
    return;
  }
  int r=level;
  while ((int)seq.size()<254 && r>0) {
    r=(r*re)>>8;
    seq.push_back(r);
  }
  if (seq.empty() || seq.back()!=0) seq.push_back(0);
}

// the driver's envelope for a sample voice, frame by frame, as a volume
// macro (0-255): attack, decay to the sustain (where it loops), release
void mpDSEnvelope(int at, int de, int su, int re, std::vector<int>& seq, int& loop, int& rel) {
  seq.clear();
  int env=0;
  // attack
  for (int f=0; f<255; f++) {
    env+=at;
    if (at==0) env=255;
    if (env>=255) {
      env=255;
      seq.push_back(env);
      break;
    }
    seq.push_back(env);
  }
  // decay
  while ((int)seq.size()<200 && env>su) {
    env=(env*de)>>8;
    if (env<=su) env=su;
    seq.push_back(env);
  }
  // (Furnace loops from loop to rel, both included, and only before the
  // release if loop<rel: the sustain twice, so the loop holds it and the
  // release ends once)
  seq.push_back(seq.back());
  loop=(int)seq.size()-2;
  rel=(int)seq.size()-1;
  // release, from the sustain level
  mpRelease(false,env,re,seq);
}

// a Game Boy voice's envelope (0-15), frame by frame: the driver moves it a
// step every so many frames (0: at once)
void mpCGBEnvelope(int at, int de, int su, int re, std::vector<int>& seq, int& loop, int& rel) {
  seq.clear();
  at&=7;
  de&=7;
  su&=15;
  re&=7;
  int env=0;
  if (at==0) {
    env=15;
    seq.push_back(env);
  } else {
    while (env<15 && (int)seq.size()<150) {
      for (int k=0; k<at-1 && (int)seq.size()<150; k++) seq.push_back(env);
      env++;
      seq.push_back(env);
    }
  }
  if (de==0) {
    env=su;
    seq.push_back(env);
  } else {
    while (env>su && (int)seq.size()<220) {
      for (int k=0; k<de-1 && (int)seq.size()<220; k++) seq.push_back(env);
      env--;
      seq.push_back(env);
    }
  }
  // (Furnace loops from loop to rel, both included, and only before the
  // release if loop<rel: the sustain twice, so the loop holds it and the
  // release ends once)
  seq.push_back(seq.back());
  loop=(int)seq.size()-2;
  rel=(int)seq.size()-1;
  mpRelease(true,env,re,seq);
}

}

bool DivEngine::loadGSF(unsigned char* file, size_t len, const char* path) {
  bool success=false;
  warnings="";
  try {
    GSFPart mini;
    gsfParse(file,len,mini);

    // the library (the game's ROM), next to the file, and then this file's
    // own data on top of it
    GSFMem mem;
    String dir;
    if (path!=NULL) {
      String p=path;
      size_t sl=p.find_last_of("/\\");
      if (sl!=String::npos) dir=p.substr(0,sl+1);
    }
    std::vector<GSFPart> libs;
    for (int li=1; li<10; li++) {
      String key=(li==1)?String("_lib"):fmt::sprintf("_lib%d",li);
      auto it=mini.tags.find(key);
      if (it==mini.tags.end()) {
        if (li==1) continue;
        break;
      }
      std::vector<unsigned char> lf;
      if (!gsfReadFile(dir+it->second,lf)) throw GSFError(fmt::sprintf("its library %s was not found next to it",it->second));
      GSFPart lp;
      gsfParse(lf.data(),lf.size(),lp);
      libs.push_back(lp);
    }
    unsigned int base=libs.empty()?mini.offset:libs[0].offset;
    mem.region=(base>>24)&0xff;
    if (mem.region!=0x08 && mem.region!=0x02) mem.region=0x08;
    for (GSFPart& lp: libs) mem.put(lp.offset,lp.data);
    mem.put(mini.offset,mini.data);
    if (mem.m.empty()) throw GSFError("there is no game in it");

    // the driver: where its song table is
    long long selectSong=-1;
    for (size_t i=0; i+64<mem.m.size(); i++) {
      if (mem.m[i]==0x00 && memcmp(mem.m.data()+i,mpSelectSong,30)==0) {
        selectSong=(long long)i;
        break;
      }
    }
    if (selectSong<0) throw GSFError("this game doesn't use the MP2K (Sappy) sound driver, the only one that can be imported for now");
    long long songTable=mem.ptr(selectSong+40);
    if (songTable<0) throw GSFError("the MP2K song table could not be found");

    // the song: a minigsf's patch is the song's number. ripping drivers load
    // it into r0 before calling the driver (MOVS r0,#n: the patch is n), or
    // keep it as data (the patch is the number itself)
    int songNum=-1;
    {
      size_t pp=mini.offset&mem.mask();
      if (!mini.data.empty() && mini.data.size()<=4) {
        if (mem.u8(pp+1)==0x20 && mini.data.size()==1) {
          songNum=mini.data[0];
        } else {
          songNum=mini.data[0]|((mini.data.size()>1)?(mini.data[1]<<8):0);
        }
      }
      if (mini.tags.count("_song")) songNum=atoi(mini.tags["_song"].c_str());
    }
    if (songNum<0) throw GSFError("this GSF doesn't say which song it plays (a full GSF without a song number can't be imported yet)");
    long long header=mem.ptr(songTable+(long long)songNum*8);
    if (header<0) throw GSFError(fmt::sprintf("song %d is not in the game's song table",songNum));
    MPSong mp(mem);
    mp.numTracks=mem.u8(header);
    mp.reverb=mem.u8(header+3);
    mp.voiceGroup=mem.ptr(header+4);
    if (mp.numTracks<1 || mp.numTracks>16 || mp.voiceGroup<0) throw GSFError(fmt::sprintf("song %d is not a valid MP2K song",songNum));
    logI("GSF import: MP2K song %d, %d tracks, voices at %x, reverb %x",songNum,mp.numTracks,(int)mp.voiceGroup,mp.reverb);

    // the tracks: first how long each one is and where it loops, then all of
    // them up to the end of the longest
    std::vector<long long> trackStart(mp.numTracks);
    std::vector<MPTrackRun> firstRun(mp.numTracks);
    int songEnd=0, loopTick=-1, loopLen=-1;
    bool loopMismatch=false;
    for (int tr=0; tr<mp.numTracks; tr++) {
      trackStart[tr]=mem.ptr(header+8+4*tr);
      if (trackStart[tr]<0) throw GSFError(fmt::sprintf("track %d of song %d is not in the game",tr+1,songNum));
      firstRun[tr]=mp.run(tr,trackStart[tr],-1,false);
      songEnd=std::max(songEnd,firstRun[tr].endTick);
    }
    for (int tr=0; tr<mp.numTracks; tr++) {
      if (!firstRun[tr].loops || firstRun[tr].endTick!=songEnd) continue;
      int ll=firstRun[tr].endTick-firstRun[tr].loopTick;
      if (loopTick<0) {
        loopTick=firstRun[tr].loopTick;
        loopLen=ll;
      } else if (firstRun[tr].loopTick!=loopTick) {
        loopMismatch=true;
      }
    }
    for (int tr=0; tr<mp.numTracks; tr++) {
      if (firstRun[tr].loops && firstRun[tr].endTick!=songEnd) {
        int ll=firstRun[tr].endTick-firstRun[tr].loopTick;
        if (loopLen>0 && ll>0 && (loopLen%ll)!=0) loopMismatch=true;
      }
      // (a track that loops sooner goes around until the song's end)
      mp.run(tr,trackStart[tr],firstRun[tr].loops?songEnd:-1,true);
    }
    if (loopMismatch) {
      addWarning("the tracks of this song loop at different points; the loop may not line up");
    }
    if (songEnd<=0) throw GSFError("this song is empty");
    if (mp.tempos.empty() || mp.tempos[0].first>0) mp.tempos.insert(mp.tempos.begin(),std::make_pair(0,75));
    std::stable_sort(mp.tempos.begin(),mp.tempos.end(),[](const std::pair<int,int>& x, const std::pair<int,int>& y) {
      return x.first<y.first;
    });
    std::stable_sort(mp.ctrls.begin(),mp.ctrls.end(),[](const MPCtrl& x, const MPCtrl& y) {
      return x.tick<y.tick;
    });
    // (notes that go past the end of the song stop there)
    for (MPNote& n: mp.notes) {
      if (n.len<0) n.len=songEnd-n.start;
      if (n.start+n.len>songEnd) n.len=songEnd-n.start;
      if (n.len<1) n.len=1;
    }

    // each note's voice: a keysplit or a drum kit picks one by the key (a
    // drum plays at its own key)
    struct VoiceRef {
      long long addr;
      int type; // the voice's type byte
      int playKey;
      int rhythmPan; // (a sample voice's own pan, -64..63, or -1000: none)
    };
    auto resolve=[&](int prog, int key) -> VoiceRef {
      VoiceRef r;
      r.addr=-1;
      r.type=-1;
      r.playKey=key;
      r.rhythmPan=-1000;
      long long v=mp.voiceGroup+(long long)prog*12;
      int t=mem.u8(v);
      if (t==0x40) {
        long long sub=mem.ptr(v+4), table=mem.ptr(v+8);
        if (sub<0 || table<0) return r;
        v=sub+(long long)mem.u8(table+(key&127))*12;
        t=mem.u8(v);
      } else if (t==0x80) {
        long long sub=mem.ptr(v+4);
        if (sub<0) return r;
        v=sub+(long long)(key&127)*12;
        t=mem.u8(v);
        r.playKey=mem.u8(v+1);
      }
      if (t==0x40 || t==0x80) return r;
      r.addr=v;
      r.type=t;
      if ((t&7)==0 && (mem.u8(v+3)&0x80)) r.rhythmPan=(int)mem.u8(v+3)-0xc0;
      return r;
    };

    // the tempo at a tick (a frame is 75/tempo ticks)
    auto tempoAt=[&](int t) -> int {
      int v=mp.tempos[0].second;
      for (auto& tp: mp.tempos) {
        if (tp.first>t) break;
        v=tp.second;
      }
      return std::max(1,v);
    };
    // each voice's envelope (frame by frame) and, for a sample voice, its
    // sample: how long a note keeps sounding after it is let go
    struct VoiceInfo {
      std::vector<int> seq;
      int loop=0, rel=0, re=0;
      bool gb=false, loops=true;
      double secs=0; // (the sample's length at key 60, if it doesn't loop)
    };
    std::map<long long,VoiceInfo> voiceInfo;
    auto infoOf=[&](const VoiceRef& v) -> VoiceInfo& {
      auto it=voiceInfo.find(v.addr);
      if (it!=voiceInfo.end()) return it->second;
      VoiceInfo& d=voiceInfo[v.addr];
      const long long a=v.addr;
      const int ct=v.type&7;
      d.gb=(ct>=1 && ct<=4);
      d.re=mem.u8(a+11);
      if (d.gb) {
        mpCGBEnvelope(mem.u8(a+8),mem.u8(a+9),mem.u8(a+10),mem.u8(a+11),d.seq,d.loop,d.rel);
      } else {
        mpDSEnvelope(mem.u8(a+8),mem.u8(a+9),mem.u8(a+10),mem.u8(a+11),d.seq,d.loop,d.rel);
        long long sp=mem.ptr(a+4);
        if (sp>=0 && mem.u8(sp)==0) {
          d.loops=(mem.u8(sp+3)&0x40)!=0;
          double rate=(double)mem.u32(sp+4)/1024.0;
          if (!d.loops && rate>0) d.secs=(double)mem.u32(sp+12)/rate;
        }
      }
      return d;
    };
    // the envelope's level when a note is let go after f frames
    auto levelAt=[](const VoiceInfo& d, int f) -> int {
      int idx=std::max(0,std::min(f-1,d.loop));
      return d.seq[idx];
    };
    // how many ticks a sample note keeps its channel: until its release has
    // faded (no longer than cap ticks past the note), or its sample ends
    auto busyTicks=[&](const VoiceRef& v, const MPNote& n, int cap) -> int {
      VoiceInfo& d=infoOf(v);
      int tempo=tempoAt(n.start);
      int f=(int)round((double)n.len*75.0/(double)tempo);
      std::vector<int> tail;
      mpRelease(false,levelAt(d,f),d.re,tail);
      int tailTicks=(int)ceil((double)tail.size()*(double)tempo/75.0);
      int b=n.len+std::min(tailTicks,cap);
      if (!d.loops && d.secs>0) {
        double mult=1.0;
        if (!(v.type&8)) mult=pow(2.0,((double)(v.playKey+n.keysh-60)+(double)n.tune/256.0)/12.0);
        int st=(int)ceil(d.secs/mult*59.7275*(double)tempo/75.0);
        b=std::min(b,std::max(1,st));
      }
      return std::max(1,b);
    };

    // the channels: sample notes on MinMod (each track gets as many
    // channels as notes it sounds at once, releases included), Game Boy
    // notes on their channel
    struct NoteOut {
      size_t n;
      VoiceRef v;
      bool gb;
      int col;
    };
    std::vector<NoteOut> outs;
    {
      std::vector<size_t> order(mp.notes.size());
      for (size_t i=0; i<order.size(); i++) order[i]=i;
      std::stable_sort(order.begin(),order.end(),[&](size_t x, size_t y) {
        return mp.notes[x].start<mp.notes[y].start;
      });
      for (size_t i: order) {
        MPNote& n=mp.notes[i];
        NoteOut o;
        o.n=i;
        o.v=resolve(n.prog,n.key);
        if (o.v.addr<0) continue;
        int ct=o.v.type&7;
        o.gb=(ct>=1 && ct<=4);
        o.col=o.gb?(ct-1):0;
        outs.push_back(o);
      }
    }
    // (per track: the whole release if that fits in MinMod's 16 channels,
    // or less of it)
    auto allocTracks=[&](int cap, bool apply) -> int {
      std::vector<std::vector<int>> slotEnd(mp.numTracks);
      std::vector<int> poly(mp.numTracks,0);
      std::vector<int> slotOf(outs.size(),0);
      for (size_t i=0; i<outs.size(); i++) {
        NoteOut& o=outs[i];
        if (o.gb) continue;
        MPNote& n=mp.notes[o.n];
        std::vector<int>& se=slotEnd[n.track];
        int slot=-1;
        for (size_t k=0; k<se.size(); k++) {
          if (se[k]<=n.start) {
            slot=(int)k;
            break;
          }
        }
        if (slot<0) {
          slot=(int)se.size();
          se.push_back(0);
        }
        se[slot]=n.start+busyTicks(o.v,n,cap);
        slotOf[i]=slot;
        poly[n.track]=std::max(poly[n.track],slot+1);
      }
      std::vector<int> base(mp.numTracks,0);
      int total=0;
      for (int tr=0; tr<mp.numTracks; tr++) {
        base[tr]=total;
        total+=poly[tr];
      }
      if (apply) {
        for (size_t i=0; i<outs.size(); i++) {
          if (!outs[i].gb) outs[i].col=base[mp.notes[outs[i].n].track]+slotOf[i];
        }
      }
      return total;
    };
    int dsChans=0;
    {
      // (a release goes on until the channel's next note: reserving
      // channels for releases would add many more of them)
      static const int caps[1]={0};
      int chosen=-1;
      for (int ci=0; ci<1; ci++) {
        if (allocTracks(caps[ci],false)<=16) {
          chosen=caps[ci];
          break;
        }
      }
      if (chosen>=0) {
        dsChans=allocTracks(chosen,true);
      } else {
        // (more than MinMod has: they share, the newest note taking the
        // channel that is free soonest)
        addWarning(fmt::sprintf("this song plays up to %d sample notes at once; MinMod has 16 channels, so some notes cut others",allocTracks(0,false)));
        std::vector<int> colEnd(16,0);
        for (NoteOut& o: outs) {
          if (o.gb) continue;
          MPNote& n=mp.notes[o.n];
          int best=0;
          for (int k=1; k<16; k++) if (colEnd[k]<colEnd[best]) best=k;
          o.col=best;
          colEnd[best]=n.start+busyTicks(o.v,n,0);
        }
        dsChans=16;
      }
    }
    bool anyGB=false;
    for (NoteOut& o: outs) if (o.gb) anyGB=true;
    if (dsChans==0 && !anyGB) throw GSFError("this song has no notes");

    // the rows: 6 ticks (a 16th note) each, or fewer if a channel has two
    // notes within one
    int speed=6;
    {
      static const int cands[4]={6,3,2,1};
      for (int ci=0; ci<4; ci++) {
        speed=cands[ci];
        std::set<std::pair<int,int>> used;
        bool ok=true;
        for (NoteOut& o: outs) {
          int col=o.gb?(100+o.col):o.col;
          if (!used.insert(std::make_pair(col,mp.notes[o.n].start/speed)).second) {
            ok=false;
            break;
          }
        }
        if (ok) break;
      }
    }
    const int numRows=(songEnd+speed-1)/speed+1;
    int patLen=64;
    while ((numRows+patLen-1)/patLen>200 && patLen<256) patLen*=2;
    if (patLen>256) patLen=256;
    const int numOrders=(numRows+patLen-1)/patLen;
    if (numOrders>256) throw GSFError("this song is too long to import");

    DivSong ds;
    ds.version=DIV_VERSION_NSF;
    ds.systemLen=0;
    int gbSys=-1, mmSys=-1;
    if (anyGB) {
      gbSys=ds.systemLen;
      ds.system[ds.systemLen++]=DIV_SYSTEM_GB;
    }
    if (dsChans>0) {
      mmSys=ds.systemLen;
      ds.system[ds.systemLen]=DIV_SYSTEM_GBA_MINMOD;
      // (the driver's usual mixing rate)
      ds.systemFlags[ds.systemLen].set("sampRate",13379);
      // (the driver adds each voice in 8 bits: at its own levels a note
      // has a few bits left and sounds grainy; this mixes them cleanly)
      ds.systemFlags[ds.systemLen].set("hqMix",true);
      ds.systemFlags[ds.systemLen].set("channels",dsChans);
      // (the driver's own levels: a note at full volume, velocity and
      // envelope, hard to one side, is the whole range there; MinMod
      // leaves room for 16 of them by default, which is far quieter)
      ds.systemFlags[ds.systemLen].set("volScale",32768);
      ds.systemLen++;
    }
    for (int i=0; i<ds.systemLen; i++) {
      ds.systemVol[i]=1.0f;
      ds.systemPan[i]=0.0f;
    }
    ds.initDefaultSystemChans();
    if (mmSys>=0) ds.systemChans[mmSys]=dsChans;
    const int gbBase=0;
    const int mmBase=(gbSys>=0)?4:0;
    auto tagOr=[&](const char* k, const String& def) -> String {
      auto it=mini.tags.find(k);
      return (it!=mini.tags.end())?it->second:def;
    };
    ds.name=tagOr("title",fmt::sprintf("Song %d",songNum));
    ds.author=tagOr("artist","");
    ds.category=tagOr("game","");
    ds.systemName="Game Boy Advance";
    ds.notes=fmt::sprintf("Imported from GSF (MP2K song %d).\nGame: %s\nArtist: %s\nYear: %s\nCopyright: %s\nRipped by: %s",songNum,tagOr("game",""),tagOr("artist",""),tagOr("year",""),tagOr("copyright",""),tagOr("gsfby",""));

    DivSubSong* sub=ds.subsong[0];
    sub->name=ds.name;
    // a tick of the song is a tick of the driver: 59.7275 Hz frames, and
    // the tempo argument (BPM/2) over 75 (the driver's 150 over 2)
    sub->hz=16777216.0f/280896.0f;
    sub->virtualTempoN=std::max(1,mp.tempos[0].second);
    sub->virtualTempoD=75;
    sub->speeds.len=1;
    sub->speeds.val[0]=speed;
    sub->patLen=patLen;
    sub->ordersLen=numOrders;
    const int totalChans=mmBase+dsChans;
    for (int c=0; c<totalChans; c++) {
      for (int o=0; o<numOrders; o++) sub->orders.ord[c][o]=o;
    }
    int maxFx[DIV_MAX_CHANS];
    for (int c=0; c<DIV_MAX_CHANS; c++) maxFx[c]=1;
    auto cellAt=[&](int c, int row) -> short* {
      return sub->pat[c].getPattern(row/patLen,true)->newData[row%patLen];
    };
    auto putFx=[&](int c, short* cell, int fx, int val) {
      for (int i=0; i<DIV_MAX_EFFECTS; i++) {
        if (cell[DIV_PAT_FX(i)]==-1) {
          cell[DIV_PAT_FX(i)]=fx;
          cell[DIV_PAT_FXVAL(i)]=val;
          if (i+1>maxFx[c]) maxFx[c]=i+1;
          return;
        }
      }
    };

    // (a row's delay: one per cell, the last one set wins)
    auto setDelay=[&](int c, short* cell, int d) {
      for (int i=0; i<DIV_MAX_EFFECTS; i++) {
        if (cell[DIV_PAT_FX(i)]==0xed) {
          cell[DIV_PAT_FXVAL(i)]=d;
          return;
        }
      }
      if (d>0) putFx(c,cell,0xed,d);
    };

    // the samples and instruments (one per voice)
    std::map<long long,int> sampleOf;
    std::map<long long,int> insOf;
    std::map<String,int> waveOf;
    int compressed=0;
    auto sampleFor=[&](long long sp) -> int {
      auto it=sampleOf.find(sp);
      if (it!=sampleOf.end()) return it->second;
      // the driver's wave data: type, flags (0x40: loops), frequency
      // (times 1024, at key 60), loop start, length, then 8-bit values
      int type=mem.u8(sp);
      if (type!=0) {
        compressed++;
        sampleOf[sp]=-1;
        return -1;
      }
      bool loops=(mem.u8(sp+3)&0x40)!=0;
      unsigned int freq=mem.u32(sp+4);
      unsigned int lp=mem.u32(sp+8);
      unsigned int sz=mem.u32(sp+12);
      if (sz==0 || sz>16*1024*1024 || sp+16+(long long)sz>(long long)mem.m.size()) {
        sampleOf[sp]=-1;
        return -1;
      }
      DivSample* s=new DivSample;
      s->name=fmt::sprintf("Sample %X",(unsigned int)(sp|((unsigned int)mem.region<<24)));
      s->depth=DIV_SAMPLE_DEPTH_8BIT;
      s->centerRate=std::max(1,(int)round((double)freq/1024.0));
      s->init(sz);
      memcpy(s->data8,mem.m.data()+sp+16,sz);
      if (loops && lp<sz) {
        s->loop=true;
        s->loopStart=lp;
        s->loopEnd=sz;
        s->loopMode=DIV_SAMPLE_LOOP_FORWARD;
      }
      int idx=(int)ds.sample.size();
      ds.sample.push_back(s);
      sampleOf[sp]=idx;
      return idx;
    };
    // (on a release the macro goes on from where it is, or, with active,
    // jumps there)
    auto setMacro=[](DivInstrumentMacro& mac, const std::vector<int>& seq, int loop, int rel, bool active) {
      mac.len=std::min((int)seq.size(),255);
      for (int k=0; k<mac.len; k++) mac.val[k]=seq[k];
      mac.loop=255;
      mac.rel=255;
      mac.open&=~8;
      if (loop>=0 && loop<mac.len) mac.loop=loop;
      if (rel>=0 && rel<mac.len) {
        mac.rel=rel;
        if (active) mac.open|=8;
      }
    };
    static const char* dutyName[4]={"12.5%","25%","50%","75%"};
    auto insBase=[&](const VoiceRef& v) -> int {
      auto it=insOf.find(v.addr);
      if (it!=insOf.end()) return it->second;
      if (ds.ins.size()>=256) return -1;
      const long long a=v.addr;
      const int t=v.type, ct=t&7;
      DivInstrument* ins=new DivInstrument;
      std::vector<int> seq;
      int loop=-1, rel=-1;
      if (ct>=1 && ct<=4) {
        ins->type=DIV_INS_GB;
        ins->gb.softEnv=true;
        ins->gb.envVol=15;
        ins->gb.envDir=0;
        ins->gb.envLen=0;
        ins->gb.soundLen=64;
        mpCGBEnvelope(mem.u8(a+8),mem.u8(a+9),mem.u8(a+10),mem.u8(a+11),seq,loop,rel);
        setMacro(ins->std.volMacro,seq,loop,rel,false);
        if (ct==1 || ct==2) {
          int duty=mem.u32(a+4)&3;
          ins->std.dutyMacro.len=1;
          ins->std.dutyMacro.val[0]=duty;
          ins->name=fmt::sprintf("Pulse %s %d",dutyName[duty],(int)ds.ins.size());
        } else if (ct==3) {
          long long wp=mem.ptr(a+4);
          int widx=0;
          if (wp>=0) {
            String key((const char*)mem.m.data()+wp,16);
            auto wi=waveOf.find(key);
            if (wi!=waveOf.end()) {
              widx=wi->second;
            } else {
              DivWavetable* wt=new DivWavetable;
              wt->len=32;
              wt->max=15;
              for (int i=0; i<16; i++) {
                wt->data[i*2]=mem.u8(wp+i)>>4;
                wt->data[i*2+1]=mem.u8(wp+i)&15;
              }
              widx=(int)ds.wave.size();
              ds.wave.push_back(wt);
              waveOf[key]=widx;
            }
          }
          ins->std.waveMacro.len=1;
          ins->std.waveMacro.val[0]=widx;
          ins->name=fmt::sprintf("Wave %d",(int)ds.ins.size());
        } else {
          int shortNoise=mem.u32(a+4)&1;
          ins->std.dutyMacro.len=1;
          ins->std.dutyMacro.val[0]=shortNoise;
          ins->name=fmt::sprintf("%s %d",shortNoise?"Noise short":"Noise",(int)ds.ins.size());
        }
      } else {
        long long sp=mem.ptr(a+4);
        int si=(sp>=0)?sampleFor(sp):-1;
        if (si<0) {
          delete ins;
          insOf[v.addr]=-1;
          return -1;
        }
        ins->type=DIV_INS_GBA_MINMOD;
        ins->amiga.initSample=si;
        mpDSEnvelope(mem.u8(a+8),mem.u8(a+9),mem.u8(a+10),mem.u8(a+11),seq,loop,rel);
        setMacro(ins->std.volMacro,seq,loop,rel,false);
        ins->name=fmt::sprintf("%s %d",(t&8)?"Drum":"Sample",(int)ds.ins.size());
      }
      int idx=(int)ds.ins.size();
      ds.ins.push_back(ins);
      insOf[v.addr]=idx;
      return idx;
    };

    // a note's envelope as an instrument of its own when it differs from
    // its voice's:
    // - a note let go before its envelope reaches the sustain fades from
    //   the level it had then (not from the sustain)
    // - a note that comes while the one before it on its channel still
    //   fades doesn't start from silence: in the driver both sound (each on
    //   its own channel), so it starts from the level of that fade
    std::map<std::pair<std::pair<long long,int>,std::vector<int>>,int> insVarOf;
    auto insFor=[&](const VoiceRef& v, int relF, const std::vector<int>& tail) -> int {
      int base=insBase(v);
      if (base<0) return base;
      VoiceInfo& d=infoOf(v);
      if (relF>d.loop || relF<1) relF=-1;
      std::vector<int> seq;
      int loop=-1, rel=-1, limit;
      if (relF>0) {
        seq.assign(d.seq.begin(),d.seq.begin()+relF);
        mpRelease(d.gb,seq.back(),d.re,seq);
        limit=relF;
      } else {
        seq=d.seq;
        loop=d.loop;
        rel=d.rel;
        limit=d.loop;
      }
      std::vector<int> t;
      for (int f=0; f<(int)tail.size() && f<limit; f++) {
        if (tail[f]>seq[f]) t.resize(f+1,0), t[f]=tail[f];
      }
      for (int f=0; f<(int)t.size(); f++) {
        if (t[f]==0) t[f]=-1;
      }
      if (relF<0 && t.empty()) return base;
      auto key=std::make_pair(std::make_pair(v.addr,relF),t);
      auto it=insVarOf.find(key);
      if (it!=insVarOf.end()) return it->second;
      if (ds.ins.size()>=256) return base;
      for (int f=0; f<(int)t.size(); f++) seq[f]=std::max(seq[f],t[f]);
      DivInstrument* ins=new DivInstrument;
      *(DivInstrumentPOD*)ins=*(DivInstrumentPOD*)ds.ins[base];
      if (relF>0) {
        ins->name=fmt::sprintf("%s (%d%s)",ds.ins[base]->name,relF,t.empty()?"":"+");
      } else {
        ins->name=fmt::sprintf("%s (+)",ds.ins[base]->name);
      }
      // (a release variant falls on its own at that frame, even inside a
      // row)
      setMacro(ins->std.volMacro,seq,loop,rel,false);
      int idx=(int)ds.ins.size();
      ds.ins.push_back(ins);
      insVarOf[key]=idx;
      return idx;
    };
    // the level of an instrument's envelope f frames into its note (held)
    auto macroAt=[&](int ins, int f) -> int {
      if (ins<0 || ins>=(int)ds.ins.size()) return 0;
      DivInstrumentMacro& mac=ds.ins[ins]->std.volMacro;
      if (mac.len<1) return 255;
      int idx=std::max(0,f-1);
      if (mac.loop<mac.len) {
        idx=std::min(idx,(int)mac.loop);
      } else if (idx>=mac.len) {
        return 0;
      }
      return mac.val[idx];
    };

    // what each channel's column holds now (to write only changes)
    struct ColState {
      int fine=0, pan=-1000, vib=0, vibFine=15, shape=-1;
      bool legato=false;
    };
    std::vector<ColState> cs(totalChans);
    // the note before on each channel (for the fade a new note starts on)
    struct PrevNote {
      int ins=-1, start=0, end=0, col=0, re=0;
    };
    std::vector<PrevNote> prevOf(totalChans);
    // each channel's notes in time, to know where the next one starts
    std::vector<std::vector<size_t>> colNotes(totalChans);
    for (size_t i=0; i<outs.size(); i++) {
      int c=outs[i].gb?(gbBase+outs[i].col):(mmBase+outs[i].col);
      colNotes[c].push_back(i);
    }
    // the volume of a note: the track's volume and the note's velocity
    auto noteVol=[&](bool gb, int vol, int vel) -> int {
      if (gb) return std::max(0,std::min(15,(int)round(15.0*(double)vol*(double)vel/(127.0*127.0))));
      return std::max(0,std::min(255,(int)round(2.0*(double)vol*(double)vel/127.0)));
    };
    // the pan: on the Game Boy left, right or both (08xy); on a sample
    // channel each side's level as the driver sets it (81xx/82xx): the
    // track's pan p gives (127-2p)/256 and (128+2p)/256 of the volume
    // (half on each side in the middle), times the voice's own pan if it
    // has one
    auto panFx=[&](const NoteOut& o, int pan) -> int {
      if (o.gb) {
        if (pan<-21) return 0xf0;
        if (pan>21) return 0x0f;
        return 0xff;
      }
      int y=std::max(-128,std::min(127,pan*2));
      double l=(double)(127-y)/256.0, r=(double)(128+y)/256.0;
      if (o.v.rhythmPan!=-1000) {
        int rp=std::max(-128,std::min(127,o.v.rhythmPan*2));
        l*=(double)(127-rp)/128.0;
        r*=(double)(128+rp)/128.0;
      }
      int pl=std::max(0,std::min(255,(int)round(l*255.0)));
      int pr=std::max(0,std::min(255,(int)round(r*255.0)));
      return (pl<<8)|pr;
    };
    auto putPan=[&](const NoteOut& o, int c, short* cell, int pv) {
      if (o.gb) {
        putFx(c,cell,0x08,pv);
      } else {
        putFx(c,cell,0x81,pv>>8);
        putFx(c,cell,0x82,pv&255);
      }
    };
    // the pitch past the note (1/128 semitone): tune and bend
    auto pitchOff=[&](int tune, int bend, int bendr) -> int {
      return (tune+bend*bendr)*2;
    };
    // (whole semitones of it, rounding half a semitone toward the note)
    auto semisOf=[](int off) -> int {
      return (off>=0)?((off+63)/128):-((-off+63)/128);
    };
    int skipped=0, legatoNotes=0;
    for (int c=0; c<totalChans; c++) {
      std::vector<size_t>& cn=colNotes[c];
      for (size_t k=0; k<cn.size(); k++) {
        NoteOut& o=outs[cn[k]];
        MPNote& n=mp.notes[o.n];
        int nextStart=(k+1<cn.size())?mp.notes[outs[cn[k+1]].n].start:songEnd;
        int end=std::min(n.start+n.len,nextStart);
        const bool releases=(end<nextStart || k+1>=cn.size());
        const int tempo=tempoAt(n.start);
        int relF=-1;
        if (releases) relF=std::max(1,(int)round((double)(end-n.start)*75.0/(double)tempo));
        // (the fade of the note before it on this channel, frame by frame
        // from this note's start, at this note's volume)
        std::vector<int> tail;
        PrevNote& pv=prevOf[c];
        int myCol=noteVol(o.gb,n.vol,n.vel);
        if (!o.gb && pv.ins>=0 && pv.col>0 && myCol>0) {
          int pf=std::max(1,(int)round((double)(pv.end-pv.start)*75.0/(double)tempo));
          int lv=macroAt(pv.ins,pf);
          int e=std::max(0,(int)round((double)(n.start-pv.end)*75.0/(double)tempo));
          if (lv>0 && e<64) {
            std::vector<int> fall;
            mpRelease(false,lv,pv.re,fall);
            for (int f=e; f<(int)fall.size() && f-e<64; f++) {
              int x=(int)round((double)fall[f]*(double)pv.col/(double)myCol);
              tail.push_back(std::min(255,x));
            }
            while (!tail.empty() && tail.back()==0) tail.pop_back();
          }
        }
        int ins=insFor(o.v,relF,tail);
        if (ins<0) {
          skipped++;
          continue;
        }
        if (!o.gb) {
          pv.ins=ins;
          pv.start=n.start;
          pv.end=end;
          pv.col=myCol;
          pv.re=infoOf(o.v).re;
        }
        const bool fixedFreq=!o.gb && (o.v.type&8);
        int row=n.start/speed;
        int delay=n.start%speed;
        short* cell=cellAt(c,row);
        if (cell[DIV_PAT_NOTE]!=-1 && cell[DIV_PAT_NOTE]!=DIV_NOTE_REL) {
          // (two notes in one row: the earlier one stays)
          skipped++;
          continue;
        }
        // the note, and its fine pitch
        int off=fixedFreq?0:pitchOff(n.tune,n.bend,n.bendr);
        int semis=semisOf(off);
        int fine=off-semis*128;
        int note=fixedFreq?108:(o.v.playKey+(o.gb?0:0)+n.keysh+semis+48);
        if (o.gb && (o.v.type&7)==4) note=o.v.playKey+n.keysh+48;
        note=std::max(0,std::min(179,note));
        cell[DIV_PAT_NOTE]=note;
        cell[DIV_PAT_INS]=ins;
        cell[DIV_PAT_VOL]=noteVol(o.gb,n.vol,n.vel);
        setDelay(c,cell,delay);
        ColState& st=cs[c];
        if (st.legato) {
          putFx(c,cell,0xea,0x00);
          st.legato=false;
        }
        if (fine!=st.fine) {
          putFx(c,cell,0xe5,0x80+fine);
          st.fine=fine;
        }
        int pf=panFx(o,n.pan);
        if (pf!=st.pan) {
          putPan(o,c,cell,pf);
          st.pan=pf;
        }
        // the driver's LFO on the pitch: a triangle vibrato, mod/16 of a
        // semitone deep, a cycle every 256/speed ticks (Furnace: 64/rate)
        int vib=0, vibFine=15;
        if (!fixedFreq && n.mod>0 && n.lfos>0 && n.modt==0 && (o.v.type&7)!=4) {
          int rate=std::max(1,std::min(15,(int)round((double)n.lfos/4.0)));
          int depth=n.mod, vf=15;
          if (depth>15) {
            vf=std::min(255,depth);
            depth=15;
          }
          vib=(rate<<4)|depth;
          vibFine=vf;
        }
        // (the shape and range, where the vibrato is turned on)
        auto vibOn=[&](short* vc) {
          if (st.shape!=3) {
            putFx(c,vc,0xe3,3);
            st.shape=3;
          }
          if (vibFine!=st.vibFine) {
            putFx(c,vc,0xe4,vibFine);
            st.vibFine=vibFine;
          }
          putFx(c,vc,0x04,vib);
          st.vib=vib;
        };
        if (vib!=0 && n.lfodl>0) {
          // the driver starts the LFO over on every note, after its delay:
          // off at the note, on lfodl ticks later (if the note lasts)
          if (st.vib!=0) {
            putFx(c,cell,0x04,0);
            st.vib=0;
          }
          int vt=n.start+n.lfodl;
          int vr=vt/speed;
          if (vt<end) {
            if (vr<=row) {
              vibOn(cell);
            } else {
              short* c2=cellAt(c,vr);
              if (c2[DIV_PAT_NOTE]==-1 || c2[DIV_PAT_NOTE]==DIV_NOTE_REL) {
                bool hasDelay=false;
                for (int i=0; i<DIV_MAX_EFFECTS; i++) if (c2[DIV_PAT_FX(i)]==0xed) hasDelay=true;
                if (!hasDelay && c2[DIV_PAT_NOTE]==-1 && (vt%speed)) putFx(c,c2,0xed,vt%speed);
                vibOn(c2);
              }
            }
          }
        } else if (vib!=st.vib || (vib!=0 && vibFine!=st.vibFine)) {
          if (vib!=0) {
            vibOn(cell);
          } else {
            putFx(c,cell,0x04,0);
            st.vib=0;
          }
        }
        // changes on the track while the note plays: volume, pan, pitch
        {
          int vol=n.vol, pan=n.pan, bend=n.bend, bendr=n.bendr, tune=n.tune;
          int lastRow=row;
          auto ci=std::lower_bound(mp.ctrls.begin(),mp.ctrls.end(),n.start+1,[](const MPCtrl& x, int t) {
            return x.tick<t;
          });
          for (; ci!=mp.ctrls.end() && ci->tick<end; ++ci) {
            if (ci->track!=n.track) continue;
            int r=ci->tick/speed;
            if (r<=row) r=row+1;
            if (r>=(end+speed-1)/speed) break;
            short* cc=cellAt(c,r);
            if (cc[DIV_PAT_NOTE]!=-1) continue;
            switch (ci->kind) {
              case 0:
                vol=ci->val;
                cc[DIV_PAT_VOL]=noteVol(o.gb,vol,n.vel);
                break;
              case 1: {
                pan=ci->val;
                int p2=panFx(o,pan);
                if (p2!=st.pan) {
                  putPan(o,c,cc,p2);
                  st.pan=p2;
                }
                break;
              }
              case 2: case 3: case 4: {
                if (fixedFreq) break;
                if (ci->kind==2) bend=ci->val;
                if (ci->kind==3) bendr=ci->val;
                if (ci->kind==4) tune=ci->val;
                int o2=pitchOff(tune,bend,bendr);
                int s2=semisOf(o2);
                int f2=o2-s2*128;
                if (s2!=semis) {
                  // (more than a semitone away: the note moves, legato)
                  int nn=std::max(0,std::min(179,note+(s2-semis)));
                  cc[DIV_PAT_NOTE]=nn;
                  if (!st.legato) {
                    putFx(c,cc,0xea,0x01);
                    st.legato=true;
                  }
                  legatoNotes++;
                  note=nn;
                  semis=s2;
                }
                if (f2!=st.fine) {
                  putFx(c,cc,0xe5,0x80+f2);
                  st.fine=f2;
                }
                break;
              }
              default:
                break;
            }
            lastRow=r;
          }
          (void)lastRow;
        }
        // the end: the release (the envelope's fall), unless the next note
        // comes first
        if (releases) {
          int rr=end/speed;
          int rd=end%speed;
          if (rr<=row) {
            rr=row+1;
            rd=0;
          }
          if (rr<numRows) {
            short* rc=cellAt(c,rr);
            if (rc[DIV_PAT_NOTE]==-1) {
              rc[DIV_PAT_NOTE]=DIV_NOTE_REL;
              // (one delay for the row: the release's, if the vibrato's
              // was there)
              bool set=false;
              for (int i=0; i<DIV_MAX_EFFECTS; i++) {
                if (rc[DIV_PAT_FX(i)]==0xed) {
                  rc[DIV_PAT_FXVAL(i)]=rd;
                  set=true;
                }
              }
              if (!set && rd>0) putFx(c,rc,0xed,rd);
            }
          }
        }
      }
    }
    if (skipped>0) logW("GSF import: %d notes left out",skipped);
    if (compressed>0) {
      addWarning(fmt::sprintf("%d samples are compressed and were left out",compressed));
    }

    // tempo changes (FDxx: the tempo argument over 75), and the loop
    {
      int cmdChan=0;
      for (size_t i=1; i<mp.tempos.size(); i++) {
        int r=mp.tempos[i].first/speed;
        if (r>=numRows) break;
        short* cc=cellAt(cmdChan,r);
        if (mp.tempos[i].second>0) putFx(cmdChan,cc,0xfd,std::min(255,mp.tempos[i].second));
      }
      int endRow=songEnd/speed;
      if (endRow>=numRows) endRow=numRows-1;
      if (loopTick>=0) {
        int lr=loopTick/speed;
        // (the last row before the end jumps back)
        int jr=std::max(0,(songEnd-1)/speed);
        short* cc=cellAt(cmdChan,jr);
        if ((lr%patLen)!=0) putFx(cmdChan,cc,0x0d,lr%patLen);
        putFx(cmdChan,cc,0x0b,lr/patLen);
        // (the tempo there, if it changed after the loop point)
        int tempoAtLoop=mp.tempos[0].second, tempoAtEnd=mp.tempos[0].second;
        for (auto& tp: mp.tempos) {
          if (tp.first<=loopTick) tempoAtLoop=tp.second;
          tempoAtEnd=tp.second;
        }
        if (tempoAtLoop!=tempoAtEnd) putFx(cmdChan,cellAt(cmdChan,lr),0xfd,std::min(255,std::max(1,tempoAtLoop)));
      } else {
        short* cc=cellAt(cmdChan,endRow);
        putFx(cmdChan,cc,0xff,0);
      }
    }
    for (int c=0; c<totalChans; c++) sub->pat[c].effectCols=std::min(DIV_MAX_EFFECTS,maxFx[c]);
    logI("GSF import: %d notes, %d instruments, %d samples, %d sample channels, %d ticks (%d per row), loop at %d, %d legato steps",(int)outs.size(),(int)ds.ins.size(),(int)ds.sample.size(),dsChans,songEnd,speed,loopTick,legatoNotes);

    ds.insLen=(int)ds.ins.size();
    ds.sampleLen=(int)ds.sample.size();
    ds.waveLen=(int)ds.wave.size();
    ds.recalcChans();

    if (active) quitDispatch();
    BUSY_BEGIN_SOFT;
    saveLock.lock();
    this->song.unload();
    this->song=ds;
    hasLoadedSomething=true;
    changeSong(0);
    saveLock.unlock();
    BUSY_END;
    if (active) {
      initDispatch();
      BUSY_BEGIN;
      renderSamples();
      reset();
      BUSY_END;
    }
    success=true;
  } catch (GSFError& e) {
    lastError=e.msg;
    logE("GSF import: %s",e.msg);
  }
  delete[] file;
  return success;
}
