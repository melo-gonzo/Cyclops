
let _ac=null,_abort=null,_hpfNode=null,_lpfNode=null,_chain=null,_srcs=[];
let _hpfOn=false,_hpfHz=120,_lpfOn=false,_lpfHz=4000;
let _an=null,_raf=0; // analyser tap + rAF handle for the live "playing" meter
// Client-side filter preview: lets the audio page audition cutoffs by ear
// (hpfDrag/lpfDrag) before committing to the device. A fixed high-pass→low-pass
// chain sits ahead of the output (built once in flush); "off" just parks a node
// at a transparent cutoff (10Hz HP / 20kHz LP) so the graph never reconnects.
// The DEVICE filters are the real ones (clean stream+clips+metric) - these only
// color what this browser hears, and clear on stop.
function clientHpf(on,hz){_hpfOn=on;_hpfHz=hz;if(_hpfNode)_hpfNode.frequency.value=on?hz:10;}
function clientLpf(on,hz){_lpfOn=on;_lpfHz=hz;if(_lpfNode)_lpfNode.frequency.value=on?hz:20000;}
async function toggleListen(){
  if(_ac){stopListen();return}
  // "connecting" until the first audio is actually scheduled - otherwise the
  // ~0.4s buffer fill + scheduling lead looks like a working button playing
  // silence. Flips to "stop" the instant sound starts.
  listen.innerHTML="&#9203; connecting&hellip;";
  // iOS Safari throws if you force a 16kHz context rate, so use the hardware
  // rate (each createBuffer below declares 16000 and WebAudio resamples). iOS
  // also needs the context unlocked inside the tap gesture: resume() plus a
  // one-sample silent buffer, both synchronous here before any await.
  try{
    _ac=new (window.AudioContext||window.webkitAudioContext)();
  }catch(e){listen.innerHTML="&#128264; listen live";_ac=null;return}
  _ac.resume();
  try{
    const u=_ac.createBuffer(1,1,_ac.sampleRate),us=_ac.createBufferSource();
    us.buffer=u;us.connect(_ac.destination);us.start(0);
  }catch(e){}
  _abort=new AbortController();
  // Jitter buffer: schedule LEAD seconds ahead of the playhead and feed
  // WebAudio in AGG-sample pieces; WiFi hiccups shorter than LEAD are
  // inaudible (and bursty delivery banks extra cushion on top, since a burst
  // that lands faster than real-time pushes the schedule further ahead). Total
  // added latency ~= LEAD. LEAD is sized for *remote* streaming: the device's
  // lwIP TCP window only holds ~180ms of 256kbps PCM, so over a high-RTT
  // WAN/cellular link TCP delivers in multi-second bursts with stalls between
  // - a small buffer underruns into silence there. 2.5s covers those stalls;
  // on LAN it just adds harmless latency. The first buffer flushes eagerly
  // (after EAGER samples) so playback starts quickly; later ones use AGG for
  // resilience.
  // LEAD: initial cushion (covers WAN bursts). REARM: the cushion re-armed AFTER
  // an underrun - small, so a mid-stream stall costs a brief gap, NOT a fresh 2.5s
  // of silence (the old code re-armed the full LEAD on every underrun, which was
  // the periodic "blackout"). On LAN underruns are rare, so the short re-arm just
  // recovers fast.
  const LEAD=2.5,REARM=0.6,AGG=2048,EAGER=512;
  let nextT=0,pend=new Int16Array(AGG),fill=0,started=false,_uruns=0;
  const flush=()=>{
    if(!fill||!_ac)return;
    const ab=_ac.createBuffer(1,fill,16000);
    const ch=ab.getChannelData(0);
    for(let i=0;i<fill;i++)ch[i]=pend[i]/32768;
    const src=_ac.createBufferSource();
    src.buffer=ab;
    // Build the high-pass→low-pass chain once; "off" nodes sit at transparent
    // cutoffs so the graph is stable (no reconnects when toggling).
    if(!_chain){
      _hpfNode=_ac.createBiquadFilter();_hpfNode.type="highpass";_hpfNode.Q.value=0.707;_hpfNode.frequency.value=_hpfOn?_hpfHz:10;
      _lpfNode=_ac.createBiquadFilter();_lpfNode.type="lowpass";_lpfNode.Q.value=0.707;_lpfNode.frequency.value=_lpfOn?_lpfHz:20000;
      // AnalyserNode sits as a pass-through just before the output, so the meter
      // reflects exactly what you hear (post client filters). Audio still plays.
      _an=_ac.createAnalyser();_an.fftSize=1024;
      _hpfNode.connect(_lpfNode);_lpfNode.connect(_an);_an.connect(_ac.destination);_chain=_hpfNode;
    }
    src.connect(_chain);
    if(nextT<_ac.currentTime+0.05){
      nextT=_ac.currentTime+(started?REARM:LEAD); // small re-arm after an underrun
      if(started)console.warn("audio underrun #"+(++_uruns)+" - re-buffering "+REARM+"s");
    }
    src.start(nextT);nextT+=ab.duration;
    // Track scheduled sources so stopListen() can stop the ones queued ahead
    // (up to LEAD seconds out); drop each from the list when it finishes so the
    // array doesn't grow unbounded over a long listen.
    _srcs.push(src);
    src.onended=()=>{const i=_srcs.indexOf(src);if(i>=0)_srcs.splice(i,1)};
    fill=0;
    if(!started){started=true;listen.innerHTML="&#128266; stop";lvwitness.style.display="";meterTick();}
  };
  try{
    const r=await fetch("/audio/stream",{signal:_abort.signal});
    if(!r.ok)throw 0;
    const rd=r.body.getReader();
    let skip=44,carry=new Uint8Array(0);
    while(_ac){
      const{done,value}=await rd.read();
      if(done)break;
      let d=value;
      if(skip>0){const s=Math.min(skip,d.length);d=d.subarray(s);skip-=s;if(!d.length)continue}
      const b=new Uint8Array(carry.length+d.length);
      b.set(carry);b.set(d,carry.length);
      const n=b.length>>1;
      if(!n){carry=b;continue}
      carry=b.slice(n*2);
      const i16=new Int16Array(b.buffer,0,n);
      for(let i=0;i<n;i++){
        pend[fill++]=i16[i];
        if(fill===AGG||(!started&&fill>=EAGER))flush();
      }
    }
  }catch(e){}
  stopListen();
}
// Drives the "playing" level bar from the actual playback (proof audio is live,
// not just connected). Peak of the time-domain samples -> bar width, ~per frame.
function meterTick(){
  if(!_ac||!_an){_raf=0;return}
  const buf=new Uint8Array(_an.fftSize);
  _an.getByteTimeDomainData(buf);
  let peak=0;
  for(let i=0;i<buf.length;i++){const v=Math.abs(buf[i]-128);if(v>peak)peak=v}
  lvfill.style.width=Math.min(100,peak/128*140)+"%"; // 140 = a little headroom boost
  _raf=requestAnimationFrame(meterTick);
}
function stopListen(){
  if(_raf){cancelAnimationFrame(_raf);_raf=0}
  lvwitness.style.display="none";lvfill.style.width="0";_an=null;
  if(_abort){_abort.abort();_abort=null}
  // Stop sources scheduled into the future BEFORE closing the context, so iOS
  // WebKit (where close() can lag) doesn't keep the old context alive on rapid
  // toggle and stack contexts.
  for(const s of _srcs){try{s.stop()}catch(e){}}
  _srcs=[];
  if(_ac){_ac.close();_ac=null}
  _chain=null;_hpfNode=null;_lpfNode=null;_hpfOn=false;_lpfOn=false;
  listen.innerHTML="&#128264; listen live";
}
// Clip playback: download the whole WAV, decode it, and play through a WebAudio
// AudioContext. WebAudio (not <audio>) because the context is unlocked
// SYNCHRONOUSLY in the click gesture below, so play() isn't blocked by the loss
// of user-activation that happens after the download await - which made the
// first click silently fail (you had to click again). The device doesn't do
// HTTP Range anyway, so we fetch the whole clip. window._clipAudio is a truthy
// "playing" sentinel so table re-renders pause while a clip plays.
var _clipAc=null,_clipSrc=null,_clipBtn=null;
function _clipBtnIcon(b,html){if(b){b.innerHTML=html;b.disabled=false}}
// Download a clip's bytes, retrying TRANSIENT failures with backoff. The device
// serves clips off the shared SD bus and returns 503 "sd busy, retry" when a
// recorder holds the lock (common - clips are written constantly); a brief stall
// can also truncate the body, which makes fetch/arrayBuffer reject. Both are
// transient, so we retry a handful of times (~6s total) instead of failing the
// click - which is why playback used to need many manual clicks + showed the
// warning sign. A genuine hard error (404) stops immediately. `live()` lets us
// bail the moment the user clicks a different clip.
async function fetchClipBuf(url,live){
  let err,delay=300;
  for(let i=0;i<6;i++){
    if(!live())throw 0;                       // superseded by another click
    let r;
    try{r=await fetch(url)}catch(e){err=e}    // network error: retry
    if(r){
      if(r.ok){
        try{return await r.arrayBuffer()}     // success
        catch(e){err=e}                       // truncated/aborted body: retry
      }else if(r.status==503){err=new Error("sd busy")} // transient: retry
      else throw new Error("HTTP "+r.status); // 404 etc: hard fail, stop now
    }
    if(!live())throw 0;
    await new Promise(s=>setTimeout(s,delay));
    delay=Math.min(delay*1.6,1500);
  }
  throw err||new Error("clip unavailable");
}
async function playClip(url,btn){
  if(btn&&_clipBtn===btn){stopClip();return} // same button: toggle off (incl. mid-load)
  stopClip();
  _clipBtn=btn;
  if(btn){btn.innerHTML="<span class='spin'></span>";btn.disabled=true}
  try{
    // Unlock the context in the gesture, BEFORE any await. Re-create it if a
    // prior _clipRelease() (tab hidden) closed it, else decode would throw.
    if(!_clipAc||_clipAc.state=="closed")_clipAc=new (window.AudioContext||window.webkitAudioContext)();
    _clipAc.resume();
    const data=await fetchClipBuf(url,()=>_clipBtn===btn);
    if(_clipBtn!==btn)return;                 // cancelled while downloading
    const audio=await _clipAc.decodeAudioData(data);
    if(_clipBtn!==btn)return;                 // cancelled while decoding
    const src=_clipAc.createBufferSource();
    src.buffer=audio;src.connect(_clipAc.destination);
    src.onended=()=>{if(_clipSrc===src)stopClip()};
    src.start(0);
    _clipSrc=src;window._clipAudio=src;       // sentinel for the table-rebuild guards
    _clipBtnIcon(btn,"&#9209;");              // ■ stop
  }catch(e){
    if(e===0)return;                          // superseded by another click: leave its state alone
    // Never fail silently. A 404 means the clip was evicted out from under us:
    // the OLDEST clips churn constantly under the SD cap, so a row in a list that
    // was fetched seconds ago can already be gone. Drop the dead row (refresh)
    // and show ✖; any other failure flashes ⚠.
    if(_clipSrc){try{_clipSrc.stop()}catch(_){}}
    _clipSrc=null;window._clipAudio=null;
    const gone=/\b404\b/.test(e&&e.message||"");
    const b=_clipBtn;_clipBtn=null;
    if(b){b.disabled=false;b.innerHTML=gone?"&#10006;":"&#9888;";setTimeout(()=>{if(_clipBtn!==b)b.innerHTML="&#9654;"},1500)}
    if(gone){if(typeof refreshSd=="function")refreshSd();if(typeof refreshClips=="function")refreshClips()}
  }
}
function stopClip(){
  if(_clipSrc){try{_clipSrc.onended=null;_clipSrc.stop()}catch(e){}_clipSrc=null}
  window._clipAudio=null;
  if(_clipBtn){_clipBtnIcon(_clipBtn,"&#9654;");_clipBtn=null}
}
// Release the audio HW when the page goes away / is hidden: stop any clip and
// fully close the long-lived context (it's lazily re-created on next playClip).
function _clipRelease(){stopClip();if(_clipAc){try{_clipAc.close()}catch(e){}_clipAc=null}}
window.addEventListener("pagehide",_clipRelease);
document.addEventListener("visibilitychange",()=>{if(document.hidden)_clipRelease()});
