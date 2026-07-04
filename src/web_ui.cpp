// web_ui.cpp - see web_ui.h. Board-agnostic shared web assets.

#include "web_ui.h"
#include "capabilities.h"
#include "web_auth.h"
#include "branding.h"

static WebServer *uiServer = NULL;

// Shared front-end components. EventPlot is the unified level/event timeline used
// by BOTH the audio dashboard and the camera-only motion page: same code, same
// data contract (binary Uint16 history + JSON events), so the two boards converge
// on one plot. A host element gets the range buttons, canvas, zoom + pan sliders,
// and an info line; drag to zoom, double-click to reset, tap a point/marker for
// details. opts: {historyUrl(secs,points,end,thr), eventsUrl(secs), hasLog,
// markerColor(source), onMarker(ev), defSecs, refreshMs}.
static const char UI_JS[] PROGMEM = R"UIJS(
(function(){
'use strict';
function pad2(n){return ('0'+n).slice(-2)}
function fmtHM(t){const d=new Date(t);return pad2(d.getHours())+':'+pad2(d.getMinutes())}
function fmtTs(t){const s=Math.round((Date.now()-t)/1000);if(s<60)return s+'s ago';if(s<3600)return Math.round(s/60)+'m ago';if(s<86400)return (s/3600).toFixed(1)+'h ago';return (s/86400).toFixed(1)+'d ago'}
function fmtDur(s){return s>=3600?(s/3600).toFixed(s%3600?1:0)+'h':s>=60?Math.round(s/60)+'m':Math.round(s)+'s'}
const RANGES=[[600,'10M'],[1800,'30M'],[3600,'1H'],[10800,'3H'],[21600,'6H'],[43200,'12H'],[86400,'24H'],[259200,'72H']];
const STEPS=[60,300,900,1800,3600,7200,10800,21600,43200,86400];
const STY="ep-rng{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:6px}.ep-rng a{color:#8a94a0;text-decoration:none;padding:4px 9px;border-radius:6px;font-size:.85em;cursor:pointer}.ep-rng a.cur{background:#2b6cb0;color:#fff}.ep-cv{display:block;width:100%;background:#0d0f12;border-radius:8px;touch-action:none}.ep-row{display:flex;align-items:center;gap:8px;margin-top:6px;font-size:.85em;color:#8a94a0}.ep-row input{flex:1}.ep-info{font-size:.85em;color:#cdd6df;margin-top:6px;min-height:1.2em}";
let _styOnce=false;
function injectSty(){if(_styOnce)return;_styOnce=true;const s=document.createElement('style');s.textContent=STY.replace(/ep-rng\{/,'.ep-rng{');document.head.appendChild(s)}
function mk(t,c){const e=document.createElement(t);if(c)e.className=c;return e}

window.EventPlot=function(host,opts){
  opts=opts||{};injectSty();
  const ranges=opts.ranges||RANGES, hasLog=!!opts.hasLog;
  const markerCol=opts.markerColor||(s=>s=='auto'?'#ffd24d':s=='video'?'#c08cff':s=='spectral'?'#f05d5d':s=='motion'?'#4f9dff':'#5df0a0');
  host.innerHTML='';
  const rng=mk('div','ep-rng');
  ranges.forEach(([secs,lbl])=>{const a=mk('a');a.textContent=lbl;a.dataset.s=secs;if(secs==(opts.defSecs||3600))a.className='cur';a.onclick=e=>{e.preventDefault();[...rng.children].forEach(x=>x.className='');a.className='cur';plotSecs=secs;_zoom=null;pinHi=pinEv=null;info.textContent='';load()};rng.appendChild(a)});
  const cv=mk('canvas','ep-cv');cv.height=130;
  const zrow=mk('div','ep-row'),zsl=range(),zlbl=mk('span');zlbl.style.width='48px';zlbl.style.textAlign='right';zrow.append(txt('zoom'),zsl,zlbl);
  const prow=mk('div','ep-row'),psl=range();prow.append(txt('move'),psl,sp());
  const info=mk('div','ep-info');
  host.append(rng,cv,zrow,prow,info);

  let plotSecs=opts.defSecs||3600,_zoom=null,_log=hasLog;
  let D=null,T=null,Bms=2000,marks=[],pinHi=null,pinEv=null,sel=null,drag=false,dragX0=0,suppress=false;
  if(hasLog){const lg=mk('a');lg.textContent='LOG';lg.style.marginLeft='auto';lg.className=_log?'cur':'';lg.onclick=e=>{e.preventDefault();_log=!_log;lg.className=_log?'cur':'';render()};rng.appendChild(lg)}
  const span=()=>_zoom?_zoom.spanSecs:plotSecs;     // seconds shown (requested)
  // ms-ago of the right edge: 0 = live (right edge is "now"). The DEVICE now snaps
  // the pooling window to its own bucket grid (history.h planAlignedPool), so the
  // client no longer needs the old Date.now()%Bms snap - that browser-clock snap
  // both failed to match the device phase (residual line shimmer) and made markers
  // sawtooth. Keeping this at 0 is the clean single source of the live edge.
  const wend=()=>_zoom?_zoom.endMs:0;
  // Actual drawn span in ms: the level line fills the canvas with (D.length-1)
  // buckets of Bms. When the device has LESS history than requested (e.g. motion
  // keeps only 24h but you pick 72H), this is shorter than span()*1000 - markers
  // and drag-zoom must use THIS basis or they desync from the line + time axis.
  const vspan=()=>(D&&D.length>1)?(D.length-1)*Bms:span()*1000;
  // Markers track real event age from the right edge (wend()), which is "now" for
  // the live view - so a fresh event sits at the right and slides left smoothly.
  function evX(ev){const W=cv.width,sp=vspan();const off=ev.age_ms-wend();if(off<0||off>sp)return -1;return W*(1-off/sp)}

  function render(){
    cv.width=cv.clientWidth||700;const ctx=cv.getContext('2d'),W=cv.width,H=cv.height;ctx.clearRect(0,0,W,H);
    if(!D||D.length<2){ctx.fillStyle='#8a94a0';ctx.font='12px sans-serif';ctx.fillText('collecting data...',10,20);return}
    let mx=1;for(const v of D)if(v>mx)mx=v;if(T)for(const v of T)if(v*1.1>mx)mx=v*1.1;if(mx<1)mx=1;
    let lo=Infinity;for(const v of D)if(v>0&&v<lo)lo=v;if(!isFinite(lo))lo=1;
    const xs=W/(D.length-1),ys=(H-12)/mx,vmin=Math.max(1,lo*0.5),vmax=Math.max(vmin*2,mx),lvmin=Math.log(vmin),lspan=Math.log(vmax)-lvmin;
    const yOf=v=>{if(!_log)return H-v*ys;if(v<=vmin)return H;return H-Math.min(1,(Math.log(v)-lvmin)/lspan)*(H-12)};
    ctx.beginPath();ctx.moveTo(0,H);for(let i=0;i<D.length;i++)ctx.lineTo(i*xs,yOf(D[i]));ctx.lineTo(W,H);ctx.closePath();ctx.fillStyle='rgba(61,157,240,.3)';ctx.fill();
    ctx.beginPath();for(let i=0;i<D.length;i++)ctx[i?'lineTo':'moveTo'](i*xs,yOf(D[i]));ctx.strokeStyle='#3d9df0';ctx.stroke();
    const thrVal=(T&&T.length)?T[T.length-1]:0,tn=T?Math.min(T.length,D.length):0;
    if(tn>1){ctx.setLineDash([4,4]);ctx.strokeStyle='#ffd24d';ctx.beginPath();for(let i=0;i<tn;i++)ctx[i?'lineTo':'moveTo'](i*xs,yOf(T[i]));ctx.stroke();ctx.setLineDash([]);ctx.fillStyle='#ffd24d';ctx.font='11px sans-serif';ctx.fillText('thr '+Math.round(thrVal),4,Math.max(11,yOf(T[0])-4))}
    ctx.fillStyle='#8a94a0';ctx.font='11px sans-serif';ctx.fillText('scale max '+Math.round(mx)+(_log?' · log (floor '+Math.round(lo)+')':''),4,11);
    const spanMs=(D.length-1)*Bms,nowT=Date.now(),tR=nowT-wend(),tL=tR-spanMs;ctx.font='10px sans-serif';
    let step=STEPS[STEPS.length-1];for(const s of STEPS){if(spanMs/1000/s<=8){step=s;break}}
    const stepMs=step*1000,tz=new Date().getTimezoneOffset()*60000,first=Math.ceil((tL-tz)/stepMs)*stepMs+tz;
    for(let t=first;t<=tR;t+=stepMs){const x=(t-tL)/spanMs*W;if(x<1||x>W-1)continue;ctx.strokeStyle='rgba(138,148,160,.16)';ctx.beginPath();ctx.moveTo(x,12);ctx.lineTo(x,H-12);ctx.stroke();const lb=fmtHM(t),lw=ctx.measureText(lb).width;ctx.fillStyle='#8a94a0';ctx.fillText(lb,Math.max(1,Math.min(W-lw-1,x-lw/2)),H-2)}
    for(const ev of marks){const x=evX(ev);if(x<0)continue;const s=pinEv===ev,col=markerCol(ev.source);ctx.strokeStyle=col;ctx.globalAlpha=s?.55:.2;ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,H);ctx.stroke();ctx.globalAlpha=1;ctx.fillStyle=col;ctx.beginPath();ctx.moveTo(x-(s?7:5),0);ctx.lineTo(x+(s?7:5),0);ctx.lineTo(x,s?12:9);ctx.closePath();ctx.fill()}
    if(pinEv){const x=evX(pinEv);ctx.strokeStyle='#ffd24d';ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,H);ctx.stroke()}
    else if(pinHi!=null&&pinHi>=0&&pinHi<D.length){const x=pinHi*xs,y=yOf(D[pinHi]);ctx.strokeStyle='#8a94a0';ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,H);ctx.stroke();ctx.fillStyle='#dde3ea';ctx.beginPath();ctx.arc(x,y,3,0,7);ctx.fill()}
    if(sel){const a=Math.min(sel.x0,sel.x1),b=Math.max(sel.x0,sel.x1);ctx.fillStyle='rgba(111,179,255,.18)';ctx.fillRect(a,0,b-a,H);ctx.strokeStyle='rgba(111,179,255,.7)';ctx.strokeRect(a+.5,.5,b-a-1,H-1)}
    drawInfo();
  }
  function drawInfo(){
    if(pinEv){
      const od=(pinEv.rms!=null&&pinEv.thr>0)?' · '+(pinEv.rms/pinEv.thr).toFixed(1)+'×':'';
      const meta=(pinEv.source||'event')+(pinEv.id!=null?' #'+pinEv.id:'')+od+' · '+fmtTs(Date.now()-pinEv.age_ms);
      // When the host wires onMarker (audio clips), a pinned event gets a PLAY
      // button so you can hear that clip - rebuilt only when the selection changes
      // (keyed on id/sd) so its play/stop state survives the periodic redraw.
      if(opts.onMarker){
        const key=(pinEv.id!=null?'i'+pinEv.id:'s'+pinEv.sd);
        if(info._key!==key){
          info.innerHTML='';
          const b=mk('button');b.innerHTML='&#9654;';b.title='play this clip';
          b.style.cssText='background:#3d9df0;color:#fff;border:0;border-radius:6px;padding:2px 10px;margin-right:8px;cursor:pointer;font-size:.95em;vertical-align:middle';
          b.onclick=()=>opts.onMarker(pinEv,b);
          const s=mk('span');info.append(b,s);
          info._key=key;info._meta=s;
        }
        info._meta.textContent=meta;
      } else {info.textContent=meta;info._key=null}
    }
    else if(pinHi!=null&&D&&pinHi<D.length){const ago=wend()+(D.length-1-pinHi)*Bms;info.textContent=D[pinHi]+(T&&pinHi<T.length?' · thr '+T[pinHi]:'')+' · '+(ago<2000?'now':fmtTs(Date.now()-ago));info._key=null}
    else {info.textContent='';info._key=null}
  }
  function syncSliders(){const full=plotSecs,zf=span()/full;if(document.activeElement!==zsl)zsl.value=Math.round(zf*1000);zlbl.textContent=fmtDur(span());const maxEnd=Math.max(0,(full-span())*1000);if(document.activeElement!==psl)psl.value=maxEnd>0?Math.round((1-Math.min(wend(),maxEnd)/maxEnd)*1000):1000;psl.disabled=maxEnd<=0}
  zsl.oninput=()=>{const zf=Math.max(0.01,+zsl.value/1000),sp=Math.max(60,Math.round(plotSecs*zf));const maxEnd=Math.max(0,(plotSecs-sp)*1000);_zoom={spanSecs:sp,endMs:Math.min(wend(),maxEnd)};pinHi=pinEv=null;load()};
  psl.oninput=()=>{const maxEnd=Math.max(0,(plotSecs-span())*1000),end=Math.round((1-(+psl.value/1000))*maxEnd);_zoom={spanSecs:span(),endMs:end};pinHi=pinEv=null;load()};
  cv.ondblclick=()=>{if(_zoom){_zoom=null;pinHi=pinEv=null;load()}};
  function xy(e){const r=cv.getBoundingClientRect();return (e.clientX-r.left)*(cv.width/(r.width||cv.width))}
  cv.onmousedown=e=>{drag=true;dragX0=xy(e);sel=null};
  cv.onmousemove=e=>{const x=xy(e);if(drag){sel={x0:dragX0,x1:x};render();return}};
  window.addEventListener('mouseup',e=>{if(!drag)return;drag=false;const x=xy(e);const a=Math.min(dragX0,x),b=Math.max(dragX0,x);sel=null;if(b-a>6){const W=cv.width,sp=vspan();const newSpan=Math.max(60,Math.round(sp*(b-a)/W/1000));const newEnd=Math.round(wend()+sp*(W-b)/W);_zoom={spanSecs:newSpan,endMs:newEnd};suppress=true;pinHi=pinEv=null;load()}else render()});
  cv.onclick=e=>{if(suppress){suppress=false;return}const x=xy(e);let best=null,bd=10;for(const ev of marks){const ex=evX(ev);if(ex<0)continue;const d=Math.abs(ex-x);if(d<bd){bd=d;best=ev}}if(best){pinEv=best;pinHi=null}else if(D){const xs=cv.width/(D.length-1);pinEv=null;pinHi=Math.max(0,Math.min(D.length-1,Math.round(x/xs)))}render()};

  async function load(){
    try{
      const W=cv.clientWidth||700,pts=Math.min(W,2000);
      // The device snaps the pooling window to its own bucket grid (history.h
      // planAlignedPool), so the line scrolls in whole-column steps and historical
      // columns are byte-stable. We just send the requested end (0 = live).
      const q='?secs='+span()+'&points='+pts+'&end='+Math.round(wend());
      const [r,rt]=await Promise.all([fetch(opts.historyUrl(q)),fetch(opts.historyUrl(q+'&thr=1'))]);
      if(!r.ok)return;Bms=+r.headers.get('X-Bucket-Ms')||2000;
      D=new Uint16Array(await r.arrayBuffer());T=rt.ok?new Uint16Array(await rt.arrayBuffer()):null;
      let base=[];if(opts.eventsUrl){try{const er=await fetch(opts.eventsUrl(Math.ceil(wend()/1000+span())));base=er.ok?await er.json():[]}catch(e){base=[]}}
      if(opts.onEvents)opts.onEvents(base);
      if(opts.extraMarks){try{const ex=await opts.extraMarks(Math.ceil(wend()/1000+span()));if(ex&&ex.length)base=base.concat(ex)}catch(e){}}
      marks=base;
      syncSliders();render();
    }catch(e){}
  }
  function range(){const s=document.createElement('input');s.type='range';s.min=0;s.max=1000;s.value=1000;return s}
  function txt(t){const e=mk('span');e.textContent=t;e.style.width='34px';return e}
  function sp(){const e=mk('span');e.style.width='48px';return e}
  // Periodic refresh: skipped while the tab is hidden (a backgrounded dashboard
  // otherwise keeps 3 fetches per cycle hitting the device forever) and while a
  // previous load is still in flight (a slow SD/link would pile up requests).
  // On return to the tab, refresh immediately so the plot never looks stale.
  let busy=false;
  async function tick(){if(document.hidden||busy)return;busy=true;try{await load()}finally{busy=false}}
  document.addEventListener('visibilitychange',()=>{if(!document.hidden)tick()});
  tick();setInterval(tick,opts.refreshMs||2000);
  return {reload:load};
};

// Shared top nav, rendered into <div id=nav></div> on every page (one source of
// truth instead of a copy per page). cur = the current tab's href.
window.buildNav=function(cur){
  const nav=document.getElementById('nav');if(!nav)return;
  if(!window._navSty){window._navSty=1;const st=document.createElement('style');st.textContent='#vbadge{display:none;background:#2a313a;color:#7fd6a0;border-radius:8px;padding:3px 9px;font-size:.8em}body.viewer #vbadge{display:inline-block}';document.head.appendChild(st)}
  const tabs=[['/','Clips'],['/live','Live'],['/camera','Camera'],['/rec','Record'],['/graphs','Graphs'],['/wifi','WiFi'],['/docs','Docs'],['/jpg','Snapshot']];
  const render=name=>{
    let h='<b>'+name+'</b>';
    for(const[href,lbl]of tabs)h+=(cur===href?'<span class="cur">'+lbl+'</span>':'<a href="'+href+'">'+lbl+'</a>');
    h+='<span id="vbadge" title="read-only viewer: settings disabled">\u{1F441} read-only</span>';
    nav.innerHTML=h;
  };
  // Render at once with the session-cached device name (no blank nav while the
  // device answers /caps on a busy link), then refresh the name in the background.
  let name='Cyclops';try{name=sessionStorage.getItem('_navName')||name}catch(e){}
  render(name);
  fetch('/caps').then(r=>r.json()).then(j=>{
    if(!j.name)return;
    try{sessionStorage.setItem('_navName',j.name)}catch(e){}
    if(j.name!==name)render(j.name);
  }).catch(()=>{});
};
})();
)UIJS";

static void handleUiJs() {
  if (!webAuthCheck(*uiServer)) return;
  uiServer->send_P(200, "application/javascript", UI_JS);
}

static void handleCaps() {
  if (!webAuthCheck(*uiServer)) return;
  char j[128];
  snprintf(j, sizeof(j),
           "{\"name\":\"%s\",\"has_audio\":%s,\"has_sd\":%s,\"has_video\":%s}",
           DEVICE_NAME, HAS_AUDIO ? "true" : "false", HAS_SD ? "true" : "false", "true");
  uiServer->send(200, "application/json", j);
}

void webUiRegisterEndpoints(WebServer &server) {
  uiServer = &server;
  server.on("/ui.js", HTTP_GET, handleUiJs);
  server.on("/caps", HTTP_GET, handleCaps);
}
