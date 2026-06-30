#!/usr/bin/env python3
"""Extract the embedded AUDIO_PAGE dashboard from src/audio_capture.cpp and wrap
it with a mock fetch() so the real UI can be opened in a browser WITHOUT the
device. Synthetic data drives the level plot, events (incl. a spectral trigger
with a frequency), clips, spectrum, etc.

Outputs:
  tools/ui_mock.html   - open in any browser (incl. a phone) to exercise the UI
  tools/_page.js       - the page's own <script>, for `node --check` linting

Usage: python3 tools/make_ui_mock.py
"""
import re, pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC = (ROOT / "src" / "audio_capture.cpp").read_text()

# Pull the AUDIO_PAGE raw-string literal and splice DEVICE_NAME back in.
m = re.search(r'AUDIO_PAGE\[\]\s*PROGMEM\s*=\s*R"rawliteral\((.*?)\)rawliteral";',
              SRC, re.S)
if not m:
    raise SystemExit("could not find AUDIO_PAGE literal")
page = m.group(1)
page = page.replace(')rawliteral" DEVICE_NAME R"rawliteral(', "Cyclops")
if "rawliteral" in page:
    raise SystemExit("unhandled rawliteral splice remains - check the page")

# ---- mock fetch(): synthetic responses for every endpoint the page polls ----
MOCK = r"""
<script>
// ---- offline mock: serves synthetic data so the real dashboard runs w/o device ----
const NOW = Date.now();
function ab(u16){return {ok:true,status:200,
  headers:{get:k=>MOCK_HDRS[k.toLowerCase()]||null},
  arrayBuffer:async()=>u16.buffer, json:async()=>({}), text:async()=>""};}
let MOCK_HDRS={};
const EV_AGES=[40000,300000,1200000,1800000,2600000,3200000]; // ms-ago of the events
function hist(secs,points,endMs,thr){
  points=Math.max(2,Math.min(points|0,2000));
  const bms=Math.round(secs*1000/points);
  MOCK_HDRS={"x-bucket-ms":String(bms)};
  const a=new Uint16Array(points);
  // TIME-based (not index-based): value depends on each sample's age, so the
  // waveform scrolls coherently when you pan and the spikes sit under the event
  // markers - faithful to how the device's real history behaves.
  for(let i=0;i<points;i++){
    const age=endMs+(points-1-i)*bms;                  // ms-ago of this sample
    let v=210+60*Math.sin(age/90000)+25*Math.sin(age/7000); // smooth ambient floor
    if(!thr){ for(const ea of EV_AGES) if(Math.abs(age-ea)<bms*1.5){v=Math.max(v,9000+(ea%9000));break;} }
    else v=9000;                                       // threshold series ~flat
    a[i]=Math.min(65535,Math.max(0,Math.round(v)));
  }
  return ab(a);
}
function events(secs){
  // markers across the last ~hour incl. a SPECTRAL trigger with a frequency
  const E=[
    {age_ms:    40000,id:42,sd:42,rms:17717,thr:10000,hz:0,   source:"auto"},
    {age_ms:   300000,id:41,sd:41,rms:557,  thr:10000,hz:551, source:"spectral"},
    {age_ms:  1200000,id:40,sd:40,rms:12481,thr:10000,hz:0,   source:"auto"},
    {age_ms:  1800000,id:39,sd:-1,rms:0,    thr:0,    hz:0,   source:"http"},
    {age_ms:  2600000,id:38,sd:38,rms:24637,thr:10000,hz:0,   source:"auto"},
    {age_ms:  3200000,id:37,sd:37,rms:79581,thr:72384,hz:3520,source:"spectral"},
  ].filter(e=>e.age_ms<=secs*1000+5000);
  return {ok:true,status:200,headers:{get:()=>null},json:async()=>E,text:async()=>""};
}
const J=o=>({ok:true,status:200,headers:{get:()=>null},json:async()=>o,text:async()=>JSON.stringify(o)});
// Keys MUST match the device's /audio/status JSON (handleAudioStatus in
// src/audio_capture.cpp) exactly - the page reads e.g. s.noise_floor, s.max_slots,
// s.slot_bytes, s.sd_max_clips. headless_test.mjs asserts this stays in sync.
const STATUS={rms:240,noise_floor:190,threshold:10000,gain:16,min_thr:0,
  enabled:true,recording:false,auto:true,clip_ms:5000,preroll_ms:1000,
  factor:3,algo:0,trig_k:2.5,sigma:8,stat_win:1,manual_thr:0,lpf:false,lpf_hz:4000,hpf:true,hpf_hz:120,
  slots:6,max_slots:20,recommended_slots:20,slot_bytes:332844,free_psram:3900000,
  sd:true,save_to_sd:true,sd_used_mb:71,sd_total_mb:29838,sd_max_clips:200,viewer:false};
const SPECTRUM={enabled:true,strig:true,sfactor:8,sminmag:1000,bandwarm_ms:0,
  spectro_min:15,hist_win:1440,hist_bucket_ms:2000,fs:16000,dbref:132.4,hab_min:5,band_tc:2,
  f:Array.from({length:32},(_,i)=>Math.round(60*Math.pow(8000/60,i/31))),
  m:Array.from({length:32},(_,i)=>(20*Math.log10(200+8000*Math.exp(-((i-9)**2)/8)+1)).toFixed(1)),
  t:Array.from({length:32},()=> (20*Math.log10(9000)).toFixed(1))};
function spectro(){
  const cols=200,bands=32;
  MOCK_HDRS={"x-cols":String(cols),"x-bands":String(bands),"x-bucket-ms":"3750","x-db256":"1","x-dbref":"132.4"};
  const a=new Uint16Array(cols*bands);
  for(let c=0;c<cols;c++)for(let b=0;b<bands;b++){
    const db=-60+50*Math.exp(-((b-9)**2)/10)*(0.5+0.5*Math.sin(c/9));
    a[c*bands+b]=Math.max(0,Math.round((db+132.4)*256));
  }
  return ab(a);
}
const SDLIST=[
  {name:"clip_00042_a.wav",size:332844,mtime:Math.round((NOW-40000)/1000)},
  {name:"clip_00041_s.wav",size:332844,mtime:Math.round((NOW-300000)/1000)},
  {name:"clip_00037_s.wav",size:332844,mtime:Math.round((NOW-3200000)/1000)},
  {name:"clip_00038_a.wav",size:332844,mtime:Math.round((NOW-2600000)/1000)},
];
const CLIPS=[
  {id:42,age_ms:40000, size:332844,rms:17717,thr:10000,hz:0,  source:"auto"},
  {id:41,age_ms:300000,size:332844,rms:557,  thr:10000,hz:551,source:"spectral"},
];
const _f=window.fetch;
window.fetch=function(u){
  u=String(u);
  const q=new URLSearchParams((u.split("?")[1]||""));
  const secs=+q.get("secs")||3600, pts=+q.get("points")||240, end=+q.get("end")||0;
  if(u.startsWith("/audio/history")) return Promise.resolve(hist(secs,pts,end,q.get("thr")==="1"));
  if(u.startsWith("/audio/events"))  return Promise.resolve(events(secs));
  if(u.startsWith("/audio/clipmeta"))return Promise.resolve(J({"42":[17717,10000,0],"41":[557,10000,551],"38":[24637,10000,0],"37":[79581,72384,3520]}));
  if(u.startsWith("/audio/clips"))   return Promise.resolve(J(CLIPS));
  if(u.startsWith("/audio/status"))  return Promise.resolve(J(STATUS));
  if(u.startsWith("/audio/spectrum"))return Promise.resolve(J(SPECTRUM));
  if(u.startsWith("/audio/spectrogram"))return Promise.resolve(spectro());
  if(u.startsWith("/sd/list"))       return Promise.resolve(J(SDLIST));
  if(u.startsWith("/video/status"))  return Promise.resolve(J({enabled:false,sd:true,recording:false,clip_ms:10000,preroll_ms:2000,fps:5,vmax:50,on_audio:true,trig_audio:true,min_gap_ms:3000,xtrig_cd_ms:5000,thermal_en:true,temp_max:95,thermal_paused:false,motion:false,motion_blocks:20,motion_diff:12,motion_level:0,motion_global:false,ring_kb:1500,ring_frames:34}));
  if(u.startsWith("/video/list"))    return Promise.resolve(J([]));
  if(u.startsWith("/diag"))          return Promise.resolve(J({rssi:-72,ip:"192.168.4.65",uptime_ms:5400000,free_heap:124000,free_psram:3900000,camera:true,clients:1,temp_c:78.5,fps:25,net_fps:0,res:"800x600",quality:30,xclk:20,cam_stalls:0,stream_on:true,thermal_paused:false,sd:true,sd_drops:0}));
  if(u.startsWith("/audio/config")||u.startsWith("/audio/trigger")||u.startsWith("/audio/retune"))
    return Promise.resolve(J({ok:true,ready_in_ms:1000}));
  return Promise.resolve(J({}));
};
</script>
"""

html = page.replace("</head>", MOCK + "</head>", 1)
(ROOT / "tools" / "ui_mock.html").write_text(html)

# Extract inline <script> bodies (no src=) for syntax linting.
scripts = re.findall(r"<script>(.*?)</script>", page, re.S)
(ROOT / "tools" / "_page.js").write_text("\n;\n".join(scripts))
print("wrote tools/ui_mock.html (%d bytes) and tools/_page.js (%d scripts)" %
      ((ROOT/"tools"/"ui_mock.html").stat().st_size, len(scripts)))
