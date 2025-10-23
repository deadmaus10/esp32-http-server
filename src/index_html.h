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

      <label><input type="checkbox" id="cloud" name="cloud" %CLOUDCHK%> Push to cloud</label>

      <label>Period (s)</label>
      <input id="period" name="period" type="number" min="5" step="1" value="%PERIOD%">

      <label>TLS SHA1 fingerprint (optional)</label>
      <input id="tlsfp" name="tlsfp" value="%SHAFINGERPRINT%">

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

    <div style="margin-top:8px">
      <h4 style="margin:0 0 6px;font-size:14px;font-weight:600">Electrical</h4>
      <div class="row">
        <div>
          <label>Gain (Vfs)</label>
          <select id="adsGain">
            <option>6.144</option>
            <option selected>4.096</option>
            <option>2.048</option>
            <option>1.024</option>
            <option>0.512</option>
            <option>0.256</option>
          </select>
        </div>
        <div>
          <label>Rate (SPS)</label>
          <select id="adsRate">
            <option>8</option><option>16</option><option>32</option><option>64</option>
            <option>128</option><option selected>250</option><option>475</option><option>860</option>
          </select>
        </div>
      </div>
      <p class="muted" style="margin:8px 0 0">Shunt resistance: 160 Ohm (fixed in firmware)</p>
      <div style="margin-top:10px">
        <button id="btnAdsSaveElec" class="btn" type="button">Save electrical</button>
      </div>
    </div>

    <div style="margin-top:18px">
      <h4 style="margin:0 0 6px;font-size:14px;font-weight:600">Units</h4>
      <div class="row">
        <div>
          <label>Channel0 Type</label>
          <select id="adsType0"><option value="40" selected>40 mm</option><option value="80">80 mm</option></select>
        </div>
        <div>
          <label>Channel1 Type</label>
          <select id="adsType1"><option value="40" selected>40 mm</option><option value="80">80 mm</option></select>
        </div>
      </div>
      <div style="margin-top:10px">
        <button id="btnAdsSaveUnits" class="btn" type="button">Save units</button>
      </div>
    </div>

    <div id="adsRead" class="muted" style="margin-top:12px"></div>
  </div>

    <!-- MEASUREMENT CONTROL -->
  <div class="card" id="measCard">
    <h3 style="margin-top:0;font-size:16px">Measurement</h3>

    <div class="row" style="margin-top:8px">
      <div>
        <label>Folder (on SD)</label>
        <input id="measDir" value="/meas" placeholder="/meas">
      </div>
    </div>

    <div class="row" style="margin-top:8px">
      <div>
        <label>Effective period (A0)</label>
        <input id="measDt0" disabled value="--">
      </div>
      <div>
        <label>Effective period (A1)</label>
        <input id="measDt1" disabled value="--">
      </div>
      <div>
        <label>Derived from ADS rate (SPS)</label>
        <input id="measSpsNote" disabled value="--">
      </div>
    </div>

    <div style="margin-top:10px">
      <button class="btn" id="measStartBtn" style="background:#10b981">Start</button>
      <button class="btn" id="measStopBtn"  style="background:#ef4444">Stop</button>
      <span id="measMsg" class="muted" style="margin-left:10px"></span>
    </div>

    <div id="measState" class="muted" style="margin-top:8px"></div>
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
    <p class="muted" style="margin-top:8px">Tip: Click a measurement to download the raw <code>.am1</code> capture. Use the CSV action links for quick conversions.</p>
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
</div>

<script>
const el=(id)=>document.getElementById(id);
function fmt(n){ if(n==null) return ''; if(n<1024) return n+' B'; if(n<1024*1024) return (n/1024).toFixed(1)+' KB'; return (n/1024/1024).toFixed(1)+' MB'; }
function showMsg(m,c){ const e=el('msg'); e.textContent=m; e.style.color=c||'#e2e8f0'; }

let _adsInitOnce = false;
let _adsActiveChannels = 2;

function setSelectValue(sel,v){ if(!sel) return; for(let i=0;i<sel.options.length;i++){ if(sel.options[i].value==v||sel.options[i].text==v){ sel.selectedIndex=i; return; } } }

function fmt3(x){ return (x==null||isNaN(x))?'--':(+x).toFixed(3); }
function fmt2(x){ return (x==null||isNaN(x))?'--':(+x).toFixed(2); }
function fmt1(x){ return (x==null||isNaN(x))?'--':(+x).toFixed(1); }

function cloudUIInit(){
  const chk = document.getElementById('cloud');
  const per = document.getElementById('period');
  const fp  = document.getElementById('tlsfp');
  const update = ()=>{ const on = chk.checked; per.disabled = !on; /* keep fp editable */ };
  chk.addEventListener('change', update);
  update(); // set initial state from %CLOUDCHK%
}
window.addEventListener('load', cloudUIInit);

// Poll readings + (first call) initialize controls from device config
function adsTick(initControls=false){
  const url = '/ads?sel=both&ts=' + Date.now();

  fetch(url, {cache:'no-store'})
    .then(r=>r.json())
    .then(j=>{
      const out = document.getElementById('adsRead');
      if (!j || j.ok===false){ out.textContent = 'ADS not ready'; return; }

      // One-time initialization of controls from device
      if (initControls && !_adsInitOnce){
        if (j.units && j.units.fsmm){
          const fs = j.units.fsmm;
          setSelectValue(document.getElementById('adsType0'), (fs[0] && Math.abs(fs[0]-80)<1e-3)?'80':'40');
          setSelectValue(document.getElementById('adsType1'), (fs[1] && Math.abs(fs[1]-80)<1e-3)?'80':'40');
        }
        if (j.cfg){
          const G=j.cfg.gain||[], R=j.cfg.rate||[];
          setSelectValue(document.getElementById('adsGain'), G[0]||'4.096');
          setSelectValue(document.getElementById('adsRate'), R[0]||'250');
        }
        _adsInitOnce = true;
      }

      if (j.mode==='both'){
        const count = Array.isArray(j.readings) ? j.readings.length : 2;
        _adsActiveChannels = Math.max(1, count);
      } else {
        _adsActiveChannels = 1;
      }
      measRefreshDerivedFromSelectors();

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
// MEAS HELPERS
// --- Helpers
function _fmtms(x){ return (isNaN(x) ? '--' : x.toFixed(2)+' ms'); }

async function measStatus(){
  try{
    const r = await fetch('/measure/status?ts='+Date.now(), {cache:'no-store'});
    const j = await r.json();
    const st = document.getElementById('measState');
    const msg = document.getElementById('measMsg');

    if (!j || j.active===undefined){ st.textContent='Status: n/a'; return; }

    const runBadge = j.active ? 'RUNNING' : 'IDLE';
    const file = j.file ? `<code>${j.file}</code>` : '(none)';
    const totalSps = j.sps0 || j.sps1 || 0;
    const chCount = (j.sps1 && j.sps1>0) ? 2 : 1;
    if (totalSps>0) _adsActiveChannels = chCount;
    const perChSps = (totalSps>0 && chCount>0) ? (totalSps / chCount) : 0;
    const perChFmt = (perChSps>0)
      ? perChSps.toFixed(perChSps >= 100 ? 0 : 1)
      : '0';
    const spsPart = totalSps
      ? ` | rate: ${totalSps} SPS total (~${perChFmt} SPS/ch)`
      : '';
    const dt0Fmt = (j.dt_ms0) ? _fmtms(j.dt_ms0) : '--';
    const dt1Fmt = (j.dt_ms1) ? _fmtms(j.dt_ms1) : dt0Fmt;
    const dtPart  = (j.dt_ms0||j.dt_ms1)
      ? ` | deltat: A0 ${dt0Fmt}${chCount>1 ? ' - A1 ' + dt1Fmt : ''}`
      : '';
    const smp = (typeof j.samples === 'number') ? ` | samples: ${j.samples}` : '';
    const byt = (typeof j.bytes   === 'number') ? ` | bytes: ${j.bytes}`   : '';

    st.innerHTML = `${runBadge} | file: ${file}${spsPart}${dtPart}${smp}${byt}`;

    // lock buttons appropriately
    const bStart = document.getElementById('measStartBtn');
    const bStop  = document.getElementById('measStopBtn');
    if (bStart) bStart.disabled = !!j.active;
    if (bStop)  bStop.disabled  = !j.active;

    // also reflect read-only Δt fields from live status (truth source)
    if (j.dt_ms0) document.getElementById('measDt0').value = dt0Fmt;
    if (j.dt_ms1) document.getElementById('measDt1').value = dt1Fmt;
    const note = document.getElementById('measSpsNote');
    if (totalSps>0){
      if (note){
        note.value = (perChSps>0)
          ? `${totalSps} SPS total (~${perChFmt} SPS per channel)`
          : `${totalSps} SPS total`;
      }
    } else if (note){
      note.value = '--';
    }

    if (msg && j.note) msg.textContent = j.note;
  }catch(_){}
}

async function measStart(){
  const dir  = document.getElementById('measDir').value.trim() || '/meas';

  const body = new URLSearchParams();
  body.set('dir', dir);                    // optional — firmware creates /meas anyway

  const msg = document.getElementById('measMsg');
  msg.textContent = 'Starting...';
  try{
    const r = await fetch('/measure/start', {method:'POST', body});
    const j = await r.json();
    msg.textContent = j.ok ? 'Started' : ('Failed: '+(j.err||''));
    measStatus();
  }catch(e){
    msg.textContent = 'Start error';
  }
}

async function measStop(){
  const msg = document.getElementById('measMsg');
  msg.textContent = 'Stopping...';
  try{
    const r = await fetch('/measure/stop', {method:'POST'});
    const j = await r.json();
    msg.textContent = j.ok ? ('Stopped'+(j.file?(' ('+j.file+')'):'') ) : ('Failed: '+(j.err||''));
    measStatus();
  }catch(e){
    msg.textContent = 'Stop error';
  }
}

function measRefreshDerivedFromSelectors(){
  const rateEl = document.getElementById('adsRate');
  const totalSps = rateEl ? parseFloat(rateEl.value) : NaN;
  const chCount = Math.max(1, _adsActiveChannels||1);
  const perChSps = (!isNaN(totalSps) && totalSps>0) ? (totalSps / chCount) : NaN;
  const dtMs = (!isNaN(perChSps) && perChSps>0) ? (1000 / perChSps) : NaN;
  const dtStr = _fmtms(dtMs);

  const dt0El = document.getElementById('measDt0');
  const dt1El = document.getElementById('measDt1');
  if (dt0El) dt0El.value = dtStr;
  if (dt1El) dt1El.value = (chCount>=2) ? dtStr : '--';

  const note = document.getElementById('measSpsNote');
  if (note){
    if (!isNaN(totalSps) && totalSps>0){
      if (!isNaN(perChSps) && perChSps>0){
        const perFmt = perChSps.toFixed(perChSps >= 100 ? 0 : 1);
        note.value = `${totalSps} SPS total (~${perFmt} SPS per channel)`;
      } else {
        note.value = `${totalSps} SPS total`;
      }
    } else {
      note.value = '--';
    }
  }
}

function measInit(){
  const bStart = document.getElementById('measStartBtn');
  const bStop  = document.getElementById('measStopBtn');
  if (bStart) bStart.addEventListener('click', measStart);
  if (bStop)  bStop.addEventListener('click', measStop);

  // Show derived periods from the ADS rate selectors (read-only)
  measRefreshDerivedFromSelectors();
  const rateEl = document.getElementById('adsRate');
  if (rateEl) rateEl.addEventListener('change', measRefreshDerivedFromSelectors);

  // Poll status while page is open
  measStatus();
  setInterval(measStatus, 1000);
}
window.addEventListener('load', measInit);

// Save electrical config (shared gain/rate)
function adsSaveElectrical(){
  const p = new URLSearchParams();
  const gainEl = document.getElementById('adsGain');
  const rateEl = document.getElementById('adsRate');
  if (gainEl) p.set('gain', gainEl.value);
  if (rateEl) p.set('rate', rateEl.value);
  p.set('sel', 'both');

  console.log('[ADS] save electrical → /adsconf', p.toString());
  fetch('/adsconf', {method:'POST', body:p})
    .then(r=>r.json())
    .then(j=>{
      console.log('[ADS] resp', j);
      measRefreshDerivedFromSelectors();
      adsTick(false);
    })
    .catch(e=>console.error('[ADS] save error', e));
}

// Save units only
function adsApplyUnits(){
  const p = new URLSearchParams();
  p.set('type0', document.getElementById('adsType0').value);
  p.set('type1', document.getElementById('adsType1').value);

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
  const bElec  = document.getElementById('btnAdsSaveElec');
  const bUnits = document.getElementById('btnAdsSaveUnits');
  if (bElec)  bElec.addEventListener('click', adsSaveElectrical);
  if (bUnits) bUnits.addEventListener('click', adsApplyUnits);

  adsTick(true);
  setInterval(()=>adsTick(false), 3000);
});

// ---- SD browser ----
function renderFsLists(list){
  const path = (list && typeof list.path === 'string') ? list.path : '/';
  const items = (list && Array.isArray(list.items)) ? list.items : [];
  const tb = document.getElementById('t').querySelector('tbody');
  tb.innerHTML = '';

  items.forEach(it=>{
    const tr = document.createElement('tr');

    // ---- Name column (with primary action) ----
    const nameTd = document.createElement('td');
    const a = document.createElement('a');
    a.textContent = it.name;

    const full = path + (path === '/' ? '' : '/') + it.name;
    if (it.type === 'dir') {
      a.href = 'javascript:void(0)';
      a.onclick = ()=>{ document.getElementById('p').value = full; go(); };
    } else {
      if (it.name.toLowerCase().endsWith('.am1')) {
        // Primary click → binary download; CSV exports remain in action links
        a.href  = '/dl?path=' + encodeURIComponent(full);
        a.title = 'Download binary (.am1). Use CSV actions for conversions.';
      } else {
        // Other files → normal download
        a.href = '/dl?path=' + encodeURIComponent(full);
      }
    }
    nameTd.appendChild(a);

    // ---- Size column ----
    const sizeTd = document.createElement('td');
    sizeTd.textContent = it.type === 'dir' ? '' : fmt(it.size);

    // ---- Type column ----
    const typeTd = document.createElement('td');
    typeTd.innerHTML = (it.type === 'dir')
      ? '<span class="pill">dir</span>'
      : '<span class="pill">file</span>';

    // ---- Actions column ----
    const actTd = document.createElement('td');

    if (it.type === 'file') {
      if (it.name.toLowerCase().endsWith('.am1')) {
        // Extra quick actions for binary measurement files
        const aBin  = document.createElement('a');
        aBin.textContent = 'BIN';
        aBin.href = '/dl?path=' + encodeURIComponent(full);
        aBin.style.marginRight = '8px';
        actTd.appendChild(aBin);

        const aRaw  = document.createElement('a');
        aRaw.textContent = 'CSV raw';
        aRaw.href = '/export_csv?path=' + encodeURIComponent(full) + '&cols=raw';
        aRaw.style.marginRight = '8px';
        actTd.appendChild(aRaw);

        const aFull = document.createElement('a');
        aFull.textContent = 'CSV full';
        aFull.href = '/export_csv?path=' + encodeURIComponent(full) + '&cols=full';
        aFull.style.marginRight = '8px';
        actTd.appendChild(aFull);
      } else {
        const dl = document.createElement('a');
        dl.textContent = '';
        dl.href = '/dl?path=' + encodeURIComponent(full);
        dl.style.marginRight = '8px';
        actTd.appendChild(dl);
      }
    }

    // Delete (files & dirs)
    const del = document.createElement('a');
    del.textContent = 'Delete';
    del.href = 'javascript:void(0)';
    del.onclick = ()=>{
      if (confirm('Delete ' + it.name + '?')) {
        fetch('/rm?path=' + encodeURIComponent(full), {method:'POST'})
          .then(r=>r.json())
          .then(r=>{ showMsg(r.ok ? 'Deleted' : 'Delete failed', '#f87171'); go(); });
      }
    };
    actTd.appendChild(del);

    tr.appendChild(nameTd);
    tr.appendChild(sizeTd);
    tr.appendChild(typeTd);
    tr.appendChild(actTd);
    tb.appendChild(tr);
  });

  showMsg('Listed ' + items.length + ' item(s) in ' + path);
}

function up(){
  let p=el('p').value.trim(); if(!p.startsWith('/')) p='/'+p;
  if(p=='/') return; const i=p.lastIndexOf('/'); p = (i<=0?'/':p.substring(0,i));
  el('p').value=p; go();
}
function go(){
  let p = el('p').value.trim(); if (!p.startsWith('/')) p = '/' + p;
  fetch('/fs?path=' + encodeURIComponent(p))
    .then(r => r.json())
    .then(j => {
      if (!j.ok) { showMsg(j.err || 'Error', '#f87171'); return; }
      // Use the new renderer:
      renderFsLists(j);
    })
    .catch(_ => showMsg('Fetch error', '#f87171'));
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
    showModal('Settings saved', 'Reboot to apply networking.', [
      {label:'Reboot now', onClick: ()=>rebootNow()},
      {label:'Close', onClick: ()=>hideModal()}
    ]);
  }catch(err){
    showModal('Save failed', `<code>${(err && err.message)||'Error'}</code>`, [{label:'Close', onClick: ()=>hideModal()}]);
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
      showModal('Firmware updated', 'Reboot to run the new firmware.', [
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
    // You could show a small "reconnecting..." indicator here if desired
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
