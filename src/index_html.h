#pragma once
#include <Arduino.h>

// Super-rare delimiter so no accidental closure from HTML/JS:
static const char INDEX_HTML[] PROGMEM = R"IDX7f1f(
<!doctype html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Sensor-Prototype Config</title>
<style>
 body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;margin:0;background:#0b1220;color:#e2e8f0}
 .wrap{max-width:980px;margin:0 auto;padding:24px}
 h1{font-size:22px;margin:0 0 16px}
 .card{background:#111827;border:1px solid #1f2937;border-radius:14px;padding:18px;margin:14px 0}
 label{display:block;font-size:13px;margin:10px 0 6px}
 input,select{width:100%;padding:10px;border-radius:10px;border:1px solid #334155;background:#0b1220;color:#e2e8f0}
 .row{display:grid;gap:10px;grid-template-columns:repeat(auto-fit,minmax(180px,1fr))}
 .btn{background:#2563eb;border:none;color:#fff;padding:10px 14px;border-radius:10px;cursor:pointer}
 .btn:active{transform:translateY(1px)}
 .muted{opacity:.8}
 code{background:#0b1220;border:1px solid #334155;border-radius:8px;padding:2px 6px}
 .ok{color:#34d399}.bad{color:#f87171}
 table{width:100%;border-collapse:collapse;margin-top:10px}
 th,td{padding:8px 10px;border-bottom:1px solid #1f2937}
 a{color:#93c5fd;text-decoration:none}
 .row2{display:grid;gap:10px;grid-template-columns:1fr auto auto auto auto}
 .pill{display:inline-block;background:#0b1220;border:1px solid #334155;border-radius:999px;padding:2px 8px;font-size:12px}
 .modal{position:fixed;inset:0;background:rgba(0,0,0,.5);display:flex;align-items:center;justify-content:center;z-index:9999}
 .modal.hidden{display:none}
 .modal-card{background:#111827;border:1px solid #1f2937;border-radius:14px;padding:18px;max-width:460px;width:90%}
 .modal-card .btn{margin-right:8px}
</style>
</head><body><div class="wrap">
  <h1>Sensor Config</h1>

  <!-- CONFIG -->
  <div class="card">
    <form id="cfgForm" method="POST" action="/save" enctype="application/x-www-form-urlencoded">
      <label>Device name</label>
      <input name="devName" value="%DEVNAME%">

      <label>Server URL (where device talks to)</label>
      <input name="serverUrl" value="%SERVERURL%">

      <label>API key (optional)</label>
      <input name="apiKey" value="%APIKEY%">

      <label>Ethernet mode</label>
      <select name="mode" id="modeSel">
        <option value="dhcp" %DHCPSEL%>DHCP</option>
        <option value="static" %STATICSEL%>Static</option>
      </select>

      <div id="ipBox" style="display:%IPBOXDISP%">
        <div class="row">
          <div><label>IP</label><input name="ip"   value="%IP%"></div>
          <div><label>Mask</label><input name="mask" value="%MASK%"></div>
          <div><label>Gateway</label><input name="gw"  value="%GW%"></div>
          <div><label>DNS</label><input name="dns" value="%DNS%"></div>
        </div>
      </div>

      <div style="margin-top:14px">
        <button class="btn" type="submit">Save</button>
        <a class="btn" style="background:#10b981" href="#" id="rebootBtn">Reboot</a>
      </div>
      <p class="muted">AP: connect to <code>%APSSID%</code> -> <code>http://192.168.4.1</code></p>
    </form>
  </div>

  <!-- STATUS -->
  <div class="card">
    <h3 style="margin-top:0;font-size:16px">Status</h3>
    <div id="s"></div>
  </div>

  <!-- SENSOR (ADS1115, 4-20 mA dual) -->
  <div class="card" id="adsCard">
    <h3 style="margin-top:0;font-size:16px">Sensor (ADS1115, 4-20 mA)</h3>

    <!-- View selector -->
    <div class="row" style="margin-top:8px">
      <div>
        <label>View</label>
        <select id="adsSel">
          <option value="both" selected>Both</option>
          <option value="0">A0</option>
          <option value="1">A1</option>
        </select>
      </div>
    </div>

    <!-- Per-channel electrical config -->
    <div class="row" style="margin-top:8px">
      <div>
        <label>A0 Gain (Vfs)</label>
        <select id="g0">
          <option>6.144</option>
          <option selected>4.096</option>
          <option>2.048</option>
          <option>1.024</option>
          <option>0.512</option>
          <option>0.256</option>
        </select>
        <label>A0 Rate (SPS)</label>
        <select id="r0">
          <option>8</option><option>16</option><option>32</option><option>64</option>
          <option>128</option><option selected>250</option><option>475</option><option>860</option>
        </select>
        <label>A0 Shunt (Ω)</label>
        <input id="s0" type="number" step="0.1" value="160.0">
      </div>

      <div>
        <label>A1 Gain (Vfs)</label>
        <select id="g1">
          <option>6.144</option>
          <option selected>4.096</option>
          <option>2.048</option>
          <option>1.024</option>
          <option>0.512</option>
          <option>0.256</option>
        </select>
        <label>A1 Rate (SPS)</label>
        <select id="r1">
          <option>8</option><option>16</option><option>32</option><option>64</option>
          <option>128</option><option selected>250</option><option>475</option><option>860</option>
        </select>
        <label>A1 Shunt (Ω)</label>
        <input id="s1" type="number" step="0.1" value="160.0">
      </div>

      <div>
        <label>Channel0 Type</label>
        <select id="adsType0"><option value="40" selected>40 mm</option><option value="80">80 mm</option></select>
        <label>Channel1 Type</label>
        <select id="adsType1"><option value="40" selected>40 mm</option><option value="80">80 mm</option></select>
        <label>Offset0 (mm)</label>
        <input id="adsOff0" type="number" step="0.01" value="0">
        <label>Offset1 (mm)</label>
        <input id="adsOff1" type="number" step="0.01" value="0">
      </div>
    </div>

    <div style="margin-top:10px">
      <button id="btnAdsSaveElec" class="btn" type="button">Save electrical</button>
      <button id="btnAdsSaveUnits" class="btn" type="button" style="margin-left:8px">Save units</button>
    </div>

    <div id="adsRead" class="muted" style="margin-top:8px"></div>
  </div>

  <!-- SD BROWSER -->
  <div class="card">
    <h3 style="margin-top:0;font-size:16px">SD Browser</h3>
    <div class="row2">
      <input id="p" value="/" />
      <button class="btn" onclick="go()">Go</button>
      <button class="btn" onclick="up()">Up</button>
      <button class="btn" onclick="mk()">Mkdir</button>
      <label class="btn" style="display:inline-block">
        Upload<input id="upl" type="file" style="display:none" onchange="upload()">
      </label>
    </div>
    <div id="msg" class="muted" style="margin-top:8px"></div>
    <table id="t"><thead><tr><th>Name</th><th>Size</th><th>Type</th><th></th></tr></thead><tbody></tbody></table>
  </div>

  <!-- OTA -->
  <div class="card">
    <h3 style="margin-top:0;font-size:16px">Firmware Update (OTA)</h3>
    <div class="row">
      <div>
        <label>Expected MD5 (optional but recommended)</label>
        <input id="otaMd5" placeholder="e.g. b1946ac92492d2347c6235b4d2611184">
      </div>
      <div>
        <label>Firmware .bin</label>
        <input id="otaFile" type="file" accept=".bin">
      </div>
    </div>
    <div style="margin-top:10px">
      <button class="btn" onclick="doOTA()">Upload & Flash</button>
      <span id="otaMsg" class="muted" style="margin-left:10px"></span>
    </div>
  </div>

  <!-- LOGS -->
  <div class="card">
    <h3 style="margin-top:0;font-size:16px">Logs</h3>
    <div>
      <button class="btn" onclick="loadLogs()">Refresh</button>
      <button class="btn" onclick="clearLogs()">Clear all</button>
    </div>
    <table id="logs"><thead><tr><th>File</th><th>Size</th><th></th></tr></thead><tbody></tbody></table>
  </div>
</div>

<!-- LOG VIEWER (tail) -->
<div class="card">
  <h3 style="margin-top:0;font-size:16px">Log Viewer (tail)</h3>
  <div class="row" style="align-items:end">
    <div>
      <label>File</label>
      <select id="tailFile"></select>
    </div>
    <div>
      <label>Lines</label>
      <select id="tailLines">
        <option>100</option>
        <option selected>200</option>
        <option>500</option>
        <option>1000</option>
      </select>
    </div>
    <div>
      <label>&nbsp;</label>
      <button class="btn" id="tailPauseBtn" onclick="toggleTail()">Pause</button>
    </div>
  </div>
  <pre id="tailOut" style="margin-top:10px; background:#0b1220; border:1px solid #334155; border-radius:10px; padding:12px; max-height:360px; overflow:auto; white-space:pre-wrap;"></pre>
</div>

<script>
const el=(id)=>document.getElementById(id);
function fmt(n){ if(n==null) return ''; if(n<1024) return n+' B'; if(n<1024*1024) return (n/1024).toFixed(1)+' KB'; return (n/1024/1024).toFixed(1)+' MB'; }
function showMsg(m,c){ const e=el('msg'); e.textContent=m; e.style.color=c||'#e2e8f0'; }

let _adsInitOnce = false;

function setSelectValue(sel,v){ if(!sel) return; for(let i=0;i<sel.options.length;i++){ if(sel.options[i].value==v||sel.options[i].text==v){ sel.selectedIndex=i; return; } } }
function getAdsSel(){ const el=document.getElementById('adsSel'); return el?el.value:'both'; }

function fmt3(x){ return (x==null||isNaN(x))?'--':(+x).toFixed(3); }
function fmt2(x){ return (x==null||isNaN(x))?'--':(+x).toFixed(2); }
function fmt1(x){ return (x==null||isNaN(x))?'--':(+x).toFixed(1); }

function adsApply(){
  const p = new URLSearchParams();
  p.set('sel',   document.getElementById('adsSel').value);
  p.set('gain',  document.getElementById('adsGain').value);
  p.set('rate',  document.getElementById('adsRate').value);
  p.set('shunt', document.getElementById('adsShunt').value);

  fetch('/adsconf', {method:'POST', body:p})
    .then(r=>r.json())
    .then(_=>{
      // keep the user’s current controls; just refresh readings
      adsTick(false);
    }).catch(()=>{});
}

function adsApplyUnits(){
  const p = new URLSearchParams();
  p.set('type0', document.getElementById('adsType0').value);
  p.set('type1', document.getElementById('adsType1').value);
  p.set('off0',  document.getElementById('adsOff0').value);
  p.set('off1',  document.getElementById('adsOff1').value);

  // Also persist the core ADS settings so they always stick:
  p.set('sel',   document.getElementById('adsSel').value);
  p.set('gain',  document.getElementById('adsGain').value);
  p.set('rate',  document.getElementById('adsRate').value);
  p.set('shunt', document.getElementById('adsShunt').value);

  fetch('/adsconf', {method:'POST', body:p})
    .then(r=>r.json())
    .then(_=>{
      // do NOT re-init controls; just refresh the live view
      adsTick(false);
    }).catch(()=>{});
}

// Poll readings + (first call) initialize controls from device config
function adsTick(initControls=false){
  const selVal = getAdsSel(); // "both" | "0" | "1"
  const url = (selVal==='both')
    ? '/ads?sel=both&ts='+Date.now()
    : '/ads?ch='+encodeURIComponent(selVal)+'&ts='+Date.now();

  fetch(url, {cache:'no-store'})
    .then(r=>r.json())
    .then(j=>{
      const out = document.getElementById('adsRead');
      if (!j || j.ok===false){ out.textContent = 'ADS not ready'; return; }

      // One-time initialization of controls from device
      if (initControls && !_adsInitOnce){
        // Units
        if (j.units && j.units.fsmm && j.units.offmm){
          const fs = j.units.fsmm, off = j.units.offmm;
          setSelectValue(document.getElementById('adsType0'), (fs[0] && Math.abs(fs[0]-80)<1e-3)?'80':'40');
          setSelectValue(document.getElementById('adsType1'), (fs[1] && Math.abs(fs[1]-80)<1e-3)?'80':'40');
          document.getElementById('adsOff0').value = (off[0]||0);
          document.getElementById('adsOff1').value = (off[1]||0);
        }
        // Electrical per-channel
        if (j.cfg){
          const G=j.cfg.gain||[], R=j.cfg.rate||[], S=j.cfg.shunt||[];
          setSelectValue(document.getElementById('g0'), G[0]||'4.096');
          setSelectValue(document.getElementById('g1'), G[1]||'4.096');
          setSelectValue(document.getElementById('r0'), R[0]||'250');
          setSelectValue(document.getElementById('r1'), R[1]||'250');
          document.getElementById('s0').value = (S[0]!=null)?S[0]:'160';
          document.getElementById('s1').value = (S[1]!=null)?S[1]:'160';
        }
        // Device-persisted selection (optional)
        if (typeof j.sel === 'number'){
          setSelectValue(document.getElementById('adsSel'), (j.sel===2)?'both':String(j.sel));
        }
        _adsInitOnce = true;
      }

      // Render readings
      const f3=(x)=> (x==null||isNaN(x))?'--':(+x).toFixed(3);
      const f2=(x)=> (x==null||isNaN(x))?'--':(+x).toFixed(2);
      const f1=(x)=> (x==null||isNaN(x))?'--':(+x).toFixed(1);

      if (j.mode==='both' && Array.isArray(j.readings)){
        const a=j.readings[0], b=j.readings[1];
        out.innerHTML =
          `A0: <code>${f3(a.ma)} mA</code> (${f1(a.pct)}%) - <code>${f2(a.mm)} mm</code> `
          + `<span class="muted">${f3(a.mv)} mV, raw ${a.raw}</span><br>`
          + `A1: <code>${f3(b.ma)} mA</code> (${f1(b.pct)}%) - <code>${f2(b.mm)} mm</code> `
          + `<span class="muted">${f3(b.mv)} mV, raw ${b.raw}</span>`;
      } else {
        const ch = (j.ch==null)?0:j.ch;
        out.innerHTML =
          `A${ch}: <code>${f3(j.ma)} mA</code> (${f1(j.pct)}%) - <code>${f2(j.mm)} mm</code> `
          + `<span class="muted">${f3(j.mv)} mV, raw ${j.raw}</span>`;
      }
    })
    .catch(e=>{ /* ignore transient fetch errors */ });
}

// Save per-channel electrical config
function adsApplyCh(){
  const p = new URLSearchParams();
  p.set('gain0',  document.getElementById('g0').value);
  p.set('gain1',  document.getElementById('g1').value);
  p.set('rate0',  document.getElementById('r0').value);
  p.set('rate1',  document.getElementById('r1').value);
  p.set('shunt0', document.getElementById('s0').value);
  p.set('shunt1', document.getElementById('s1').value);
  p.set('sel',    getAdsSel());

  console.log('[ADS] save electrical → /adsconf', p.toString());
  fetch('/adsconf', {method:'POST', body:p})
    .then(r=>r.json())
    .then(j=>{
      console.log('[ADS] resp', j);
      adsTick(false);
    })
    .catch(e=>console.error('[ADS] save error', e));
}

// Save units only
function adsApplyUnits(){
  const p = new URLSearchParams();
  p.set('type0', document.getElementById('adsType0').value);
  p.set('type1', document.getElementById('adsType1').value);
  p.set('off0',  document.getElementById('adsOff0').value);
  p.set('off1',  document.getElementById('adsOff1').value);
  p.set('sel',   getAdsSel());

  console.log('[ADS] save units → /adsconf', p.toString());
  fetch('/adsconf', {method:'POST', body:p})
    .then(r=>r.json())
    .then(j=>{
      console.log('[ADS] resp', j);
      adsTick(false);
    })
    .catch(e=>console.error('[ADS] save error', e));
}

// Wire up UI
window.addEventListener('load', ()=>{
  const sel = document.getElementById('adsSel');
  if (sel) sel.addEventListener('change', ()=>adsTick(false));

  const bElec  = document.getElementById('btnAdsSaveElec');
  const bUnits = document.getElementById('btnAdsSaveUnits');
  if (bElec)  bElec.addEventListener('click', adsApplyCh);
  if (bUnits) bUnits.addEventListener('click', adsApplyUnits);

  adsTick(true);
  setInterval(()=>adsTick(false), 3000);
});

// When just changing the view, refresh readings, do NOT reset controls
document.addEventListener('change', (e)=>{
  if (e && e.target && e.target.id === 'adsSel') adsTick(false);
});

// ---- SD browser ----
function up(){
  let p=el('p').value.trim(); if(!p.startsWith('/')) p='/'+p;
  if(p=='/') return; const i=p.lastIndexOf('/'); p = (i<=0?'/':p.substring(0,i));
  el('p').value=p; go();
}
function go(){
  let p=el('p').value.trim(); if(!p.startsWith('/')) p='/'+p;
  fetch('/fs?path='+encodeURIComponent(p)).then(r=>r.json()).then(j=>{
    if(!j.ok){ showMsg(j.err||'Error', '#f87171'); return; }
    const tb=el('t').querySelector('tbody'); tb.innerHTML='';
    j.items.forEach(it=>{
      const tr=document.createElement('tr');
      const name=document.createElement('td');
      const a=document.createElement('a'); a.textContent=it.name; a.href='javascript:void(0)';
      if(it.type=='dir'){ a.onclick=()=>{ el('p').value=(p.endsWith('/')?p:p+'/')+it.name; go(); } }
      else { a.href='/dl?path='+encodeURIComponent(p+(p=='/'?'':'/')+it.name); }
      name.appendChild(a);
      const size=document.createElement('td'); size.textContent=it.type=='dir'?'':fmt(it.size);
      const type=document.createElement('td'); type.innerHTML = it.type=='dir' ? '<span class="pill">dir</span>' : '<span class="pill">file</span>';
      const act=document.createElement('td');
      const del=document.createElement('a'); del.textContent='Delete'; del.href='javascript:void(0)';
      del.onclick=()=>{
        if(confirm('Delete '+it.name+'?')) {
          fetch('/rm?path='+encodeURIComponent(p+(p=='/'?'':'/')+it.name), {method:'POST'})
            .then(r=>r.json()).then(r=>{ showMsg(r.ok?'Deleted':'Delete failed','#f87171'); go(); });
        }
      };
      act.appendChild(del);
      tr.appendChild(name); tr.appendChild(size); tr.appendChild(type); tr.appendChild(act);
      tb.appendChild(tr);
    });
    showMsg('Listed '+j.items.length+' item(s) in '+j.path);
  }).catch(_=>showMsg('Fetch error','#f87171'));
}
function mk(){
  let d=prompt('New folder name:'); if(!d) return;
  let p=el('p').value.trim(); if(!p.startsWith('/')) p='/'+p;
  fetch('/mkdir?path='+encodeURIComponent(p+(p=='/'?'':'/')+d), {method:'POST'})
   .then(r=>r.json()).then(j=>{ showMsg(j.ok?'Folder created':'Create failed','#f87171'); go(); });
}
function upload(){
  const f = el('upl').files[0];
  if (!f) return;
  let p = el('p').value.trim(); if (!p.startsWith('/')) p = '/' + p;
  const form = new FormData(); form.append('file', f, f.name);
  fetch('/upload?dir=' + encodeURIComponent(p), { method:'POST', body: form })
    .then(r => r.json())
    .then(j => { showMsg(j.ok ? 'Uploaded' : 'Upload failed', '#f87171'); go(); })
    .catch(_ => showMsg('Upload error', '#f87171'));
}

// ---- Status ----
function upd(){
  fetch('/status')
    .then(r => r.json())
    .then(j => {
      const ok = (b,t)=>`<span class="${b?'ok':'bad'}">${t}</span>`;
      const alarmBadge = (s)=>{
        if (s === 'OVER' || s === 'UNDER') return `<span class="bad">${s}</span>`;
        if (s === 'NORMAL') return `<span class="ok">NORMAL</span>`;
        return `<span class="muted">${s||'?'}</span>`;
      };
      const alarmAny = (b)=> b ? `<span class="bad">ACTIVE</span>` : `<span class="ok">none</span>`;

      document.getElementById('s').innerHTML =
        `ETH: ${ok(j.ethUp,'up')} | LINK: ${ok(j.link,''+j.link)} | IP: <code>${j.ip}</code> | INET: ${ok(j.inet,'ok')}<br>`+
        `SD: ${ok(j.sd,'mounted')} | Time: <code>${j.time}</code> | Uptime: <code>${j.uptime}</code> | Reboot: <code>${j.reboot}</code> | mDNS: <code>${j.mdns}</code><br>`+
        `ADS: ${ok(j.adsReady,'ok')} | Fails: <code>${(typeof j.adsFail==='number')?j.adsFail:0}</code> | Last: <code>${j.adsLastErr||'none'}</code><br>`+
        `ALARM: A0=${alarmBadge(j.a0)} - A1=${alarmBadge(j.a1)} - Any=${alarmAny(!!j.aAny)}`;
    })
    .catch(()=>{ /* ignore transient errors */ });
}

upd(); setInterval(upd,1000);
document.getElementById('modeSel').addEventListener('change',e=>{
  document.getElementById('ipBox').style.display = (e.target.value==='static')?'block':'none';
});

window.onload = ()=>{
  go(); loadLogs();
  adsTick(true);                 // initialize controls from device ONCE
  setInterval(()=>adsTick(false), 1500);  // periodic poll: never resets controls
};

// ---- Save/Reboot UX ----
function $(id){return document.getElementById(id);}
function showModal(title, text, buttons=[]) {
  $('modal-title').textContent = title;
  $('modal-text').innerHTML = text;
  const box = $('modal-actions'); box.innerHTML = '';
  buttons.forEach(b=>{
    const el = document.createElement('button');
    el.className = 'btn' + (b.class?(' '+b.class):'');
    el.type='button'; el.textContent = b.label; el.onclick = b.onClick;
    box.appendChild(el);
  });
  $('modal').classList.remove('hidden');
}
function hideModal(){ $('modal').classList.add('hidden'); }
$('cfgForm').addEventListener('submit', async (e)=>{
  e.preventDefault();
  const fd = new FormData(e.target);
  const body = new URLSearchParams(fd);
  try{
    const res = await fetch('/save', {method:'POST', body});
    const j = await res.json();
    if(!j.ok) throw new Error('Save failed');
    showModal('Settings saved ✅', 'Reboot to apply networking.', [
      {label:'Reboot now', onClick: ()=>rebootNow()},
      {label:'Close', onClick: ()=>hideModal()}
    ]);
  }catch(err){
    showModal('Save failed ❌', `<code>${(err && err.message)||'Error'}</code>`, [{label:'Close', onClick: ()=>hideModal()}]);
  }
});
$('rebootBtn').addEventListener('click', (e)=>{ e.preventDefault(); rebootNow(); });
async function rebootNow(){
  showModal('Rebooting...', 'Back in a few seconds.<br><span id="cd">10</span> s', []);
  fetch('/reboot').catch(()=>{});
  let t=10; const timer=setInterval(()=>{ t--; if($('cd')) $('cd').textContent=t; if(t<=0) clearInterval(timer); },1000);
  await waitForBack(30); location.reload();
}
async function waitForBack(timeoutSec){
  const end = Date.now() + timeoutSec*1000;
  while(Date.now() < end){
    try{ const r = await fetch('/status?ts='+Date.now(), {cache:'no-store'}); if(r.ok) return true; }catch(_){}
    await new Promise(r=>setTimeout(r,1000));
  } return false;
}

// ---- OTA ----
async function doOTA(){
  const f = el('otaFile').files[0];
  const md5 = el('otaMd5').value.trim();
  if(!f){ el('otaMsg').textContent='Pick a .bin file first.'; return; }
  el('otaMsg').textContent='Uploading...';
  const form = new FormData(); form.append('fw', f, f.name);
  try{
    const r = await fetch('/ota?md5='+encodeURIComponent(md5), {method:'POST', body:form});
    const j = await r.json();
    el('otaMsg').textContent = j.ok ? ('Flashed OK. MD5='+j.md5) : ('Failed: '+(j.err||'error'));
    if(j.ok){
      showModal('Firmware updated ✅', 'Reboot to run the new firmware.', [
        {label:'Reboot now', onClick: ()=>rebootNow()},
        {label:'Close', onClick: ()=>hideModal()}
      ]);
    }
  }catch(e){
    el('otaMsg').textContent = 'Upload error';
  }
}

// ---- LOGS ----
function basename(p){ const i=p.lastIndexOf('/'); return (i>=0)? p.substring(i+1) : p; }
function loadLogs(){
  fetch('/logs?ts='+Date.now(), {cache:'no-store'})
    .then(r=>r.json())
    .then(j=>{
      const tb = document.getElementById('logs').querySelector('tbody');
      tb.innerHTML='';
      j.items.forEach(it=>{
        const nm = basename(it.name);
        const tr = document.createElement('tr');
        const n  = document.createElement('td'); n.textContent = nm;
        const s  = document.createElement('td'); s.textContent = fmt(it.size||0);
        const a  = document.createElement('td');
        const dl = document.createElement('a'); dl.textContent='Download';
        dl.href = '/logdl?file=' + encodeURIComponent(nm);
        a.appendChild(dl);
        tr.appendChild(n); tr.appendChild(s); tr.appendChild(a);
        tb.appendChild(tr);
      });
    });
}

function clearLogs(){
  if(!confirm('Delete all log files?')) return;
  fetch('/logclear', {method:'POST'}).then(_=>loadLogs());
}

let es = null;
let tailPaused = false;

function sseStart(){
  const file  = (document.getElementById('tailFile').value || 'log0.log');
  const lines = (document.getElementById('tailLines').value || '200');

  if (es) { es.close(); es = null; }

  // Build SSE URL; browser will auto-reconnect on drop
  const url = '/logstream?file=' + encodeURIComponent(file) + '&lines=' + encodeURIComponent(lines);
  es = new EventSource(url);

  es.onmessage = (ev)=>{
    const pre = document.getElementById('tailOut');
    const atBottom = pre.scrollTop + pre.clientHeight >= pre.scrollHeight - 4;
    // Data may contain \n; already formatted by server as a single event block
    pre.textContent = ev.data || '(empty)';
    if (atBottom) pre.scrollTop = pre.scrollHeight;
  };

  es.onerror = (_)=>{
    // Let EventSource handle reconnects (honors server "retry: 1000")
    // You could show a small "reconnecting…" indicator here if desired
  };
}

function toggleTail(){
  tailPaused = !tailPaused;
  const btn = document.getElementById('tailPauseBtn');
  if (tailPaused) {
    if (es) { es.close(); es = null; }
    btn.textContent = 'Resume';
  } else {
    btn.textContent = 'Pause';
    sseStart();
  }
}

function tailPopulateFilesFromLogs(items){
  const sel = document.getElementById('tailFile');
  const cur = sel.value;
  sel.innerHTML = '';
  // Default to log0.log first
  let names = items.map(it => basename(it.name));
  names.sort();
  if (!names.includes('log0.log')) names.unshift('log0.log');
  names = [...new Set(names)];
  names.forEach(n=>{
    const opt=document.createElement('option'); opt.value=n; opt.textContent=n; sel.appendChild(opt);
  });
  if (cur && names.includes(cur)) sel.value = cur;
}

document.addEventListener('change', (e)=>{
  if (e.target && (e.target.id === 'tailFile' || e.target.id === 'tailLines')) {
    tailRefresh();
  }
});

// Hook into your existing loadLogs() so the dropdown is populated.
const _origLoadLogs = loadLogs;
loadLogs = function(){
  fetch('/logs?ts='+Date.now(), {cache:'no-store'})
    .then(r=>r.json())
    .then(j=>{
      // Fill the table as before
      const tb = document.getElementById('logs').querySelector('tbody'); tb.innerHTML='';
      const names = [];
      (j.items||[]).forEach(it=>{
        const nm = basename(it.name);
        names.push(nm);
        const tr = document.createElement('tr');
        const n  = document.createElement('td'); n.textContent = nm;
        const s  = document.createElement('td'); s.textContent = fmt(it.size||0);
        const a  = document.createElement('td');
        const dl = document.createElement('a'); dl.textContent='Download';
        dl.href = '/logdl?file=' + encodeURIComponent(nm);
        a.appendChild(dl);
        tr.appendChild(n); tr.appendChild(s); tr.appendChild(a);
        tb.appendChild(tr);
      });

      // Populate viewer <select id="tailFile">
      const sel = document.getElementById('tailFile');
      const cur = sel.value;
      sel.innerHTML = '';
      let list = [...new Set(names.sort())];
      if (!list.includes('log0.log')) list.unshift('log0.log');
      list.forEach(n=>{ const o=document.createElement('option'); o.value=n; o.textContent=n; sel.appendChild(o); });
      if (cur && list.includes(cur)) sel.value = cur;

      // Start SSE if not paused
      if (!tailPaused) sseStart();
    })
    .catch(()=>{});
};

document.addEventListener('change', (e)=>{
  if (e.target && (e.target.id === 'tailFile' || e.target.id === 'tailLines')) {
    if (!tailPaused) sseStart();
  }
});
</script>

<div id="modal" class="modal hidden">
  <div class="modal-card">
    <h3 id="modal-title" style="margin:0 0 8px;font-size:16px"></h3>
    <div id="modal-text" class="muted" style="margin-bottom:14px"></div>
    <div id="modal-actions" class="row"></div>
  </div>
</div>
</body>
</html>
)IDX7f1f";
