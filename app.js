/* ===========================================================
   F350 Diesel Dashboard — Web Bluetooth + ELM327 driver
   Talks to a BLE OBD2 adapter (tested against Veepeak OBDCheck BLE)
   =========================================================== */

// ---- PID TABLE ------------------------------------------------
// mode 01 = standard OBD2, mode 22 = Ford manufacturer-specific.
// Formulas verified against community Torque Pro PID logs for the
// 2011+ Ford 6.7L Powerstroke — sanity-check against real readings
// once connected (e.g. coolant should settle ~190-210F warmed up).
// Live barometric pressure, read once at connect from PID 0133 (kPa).
// Boost is a *gauge* pressure = manifold absolute - ambient, so using a
// hardcoded 14.7 psi under-reads at altitude. Calgary sits ~1045 m, where
// ambient is nearer 13.0 psi — that alone was ~1.7 psi of error.
let BARO_PSI = 14.7;

const C = f => (f - 32) * 5 / 9;              // °F formula -> °C
const KPH = mph => mph * 1.609344;

const PIDS = {
  speed:   { mode: '01', pid: '0D', bytes: 1, decode: b => b[0] * 1.609344, unit: 'km/h', gauge: 'speedVal', kind: 'text' },
  rpm:     { mode: '01', pid: '0C', bytes: 2, decode: b => ((b[0] * 256) + b[1]) / 4, unit: 'RPM', gauge: 'rpmVal', kind: 'rpm' },
  coolant: { mode: '01', pid: '05', bytes: 1, decode: b => b[0] - 40, unit: '°C', gauge: 'g-coolant', kind: 'gauge', min: 40, max: 120 },
  fuel:    { mode: '01', pid: '2F', bytes: 1, decode: b => (b[0] / 255) * 100, unit: '%', gauge: 'fuelPct', kind: 'bar' },
  egt:     { mode: '22', pid: 'F478', bytes: 8, decode: b => C((((b[0] * 256) + b[1]) * 0.18) - 40), unit: '°C', gauge: 'g-egt', kind: 'gauge', min: 150, max: 900 },
  // FIXED: the community equation is ((((B*256)+C)*0.00393)+2.25)-Baro().
  // B and C are the SECOND and THIRD data bytes, not the first two — we were
  // decoding A,B, which is why this sat pinned near -11 psi all drive.
  boost:   { mode: '01', pid: '87', bytes: 4, decode: b => ((((b[1] * 256) + b[2]) * 0.00393) + 2.25) - BARO_PSI, unit: 'PSI', gauge: 'g-boost', kind: 'gauge', min: 0, max: 35 },
  soot:    { mode: '22', pid: '042C', bytes: 2, decode: b => ((((b[0] * 256) + b[1]) * (100 / 65535)) - 1) / 1.75 * 100, unit: '%', gauge: 'g-soot', kind: 'gauge', min: 0, max: 100 },
  trans:   { mode: '22', pid: '1E1C', bytes: 2, decode: b => C((((b[0] << 24 >> 24) * 256) + b[1]) * (9 / 80) + 32), unit: '°C', gauge: 'g-trans', kind: 'gauge', min: 40, max: 140 },
  def:     { mode: '22', pid: 'F485', bytes: 1, decode: b => (b[0] / 15) * 100, unit: '%', gauge: 'defPct', kind: 'bar' },
  // 015C is a standard OBD2 PID (engine oil temperature, A-40 in °C).
  oiltemp: { mode: '01', pid: '5C', bytes: 1, decode: b => b[0] - 40, unit: '°C', gauge: 'g-oiltemp', kind: 'gauge', min: 40, max: 140 },
  // Oil pressure is NOT standard OBD2 — resolved by probe at connect.
  oilpress:{ mode: '22', pid: '1668', bytes: 2, decode: b => ((b[0] * 256) + b[1]) * 0.0145038, unit: 'PSI', gauge: 'g-oilpress', kind: 'gauge', min: 0, max: 90 },
};

// 22042C answered NO DATA on this 2021 — that PID list dates from 2014-era
// trucks. Rather than guess, try each candidate once at connect and keep
// whichever the ECU actually answers. The log names the winner.
// DEF read a flat 100% all drive with (A/15)*100 — that divisor is almost
// certainly wrong for a 2021. Probe the usual scalings instead.
const DEF_CANDIDATES = [
  { mode: '22', pid: 'F485', bytes: 1, decode: b => (b[0] / 255) * 100 },
  { mode: '22', pid: 'F485', bytes: 1, decode: b => b[0] * 0.4 },
  { mode: '22', pid: 'F485', bytes: 1, decode: b => (b[0] / 15) * 100 },
  { mode: '01', pid: '9B',   bytes: 4, decode: b => (b[1] / 255) * 100 },
];

const OILPRESS_CANDIDATES = [
  { mode: '22', pid: '1668', bytes: 2, decode: b => ((b[0] * 256) + b[1]) * 0.0145038 },
  { mode: '22', pid: '0470', bytes: 2, decode: b => ((b[0] * 256) + b[1]) * 0.0145038 },
  { mode: '22', pid: 'F454', bytes: 1, decode: b => b[0] * 0.580151 },
  { mode: '01', pid: '0A',   bytes: 1, decode: b => b[0] * 0.145038 },
];

const SOOT_CANDIDATES = [
  { mode: '22', pid: '042C', bytes: 2, decode: b => ((((b[0] * 256) + b[1]) * (100 / 65535)) - 1) / 1.75 * 100 },
  { mode: '22', pid: '045C', bytes: 2, decode: b => ((b[0] * 256) + b[1]) * (100 / 65535) },
  { mode: '22', pid: 'F45C', bytes: 2, decode: b => ((b[0] * 256) + b[1]) * (100 / 65535) },
  { mode: '22', pid: '1C4F', bytes: 2, decode: b => ((b[0] * 256) + b[1]) * (100 / 65535) },
  { mode: '01', pid: '7C',   bytes: 4, decode: b => ((b[0] * 256) + b[1]) * (100 / 65535) },
];

const POLL_ORDER = ['speed', 'rpm', 'coolant', 'fuel', 'egt', 'boost', 'soot', 'trans', 'def', 'oiltemp', 'oilpress'];

// ---- BLE UART auto-detect --------------------------------------
// Cheap ELM327 BLE modules (incl. many Veepeak units) expose a
// UART-style service rather than a standard GATT profile, and the
// exact UUIDs vary by vendor. We ask for common candidates, then
// once connected we scan all services/characteristics for a
// writable + notifiable pair rather than hard-coding one UUID set.
const KNOWN_UART_SERVICES = [
  '6e400001-b5a3-f393-e0a9-e50e24dcca9e', // Nordic UART Service (very common)
  '0000fff0-0000-1000-8000-00805f9b34fb', // generic FFF0 UART clone
  '0000ffe0-0000-1000-8000-00805f9b34fb', // HM-10/CC41 UART clone
  '0000fee7-0000-1000-8000-00805f9b34fb', // additional common OBD BLE clone service
  'e7810a71-73ae-499d-8c15-faa9aef0c3f2', // some OBDLink/Veepeak BLE clones
  // --- Veepeak 66:1E:87:06:30:B3 (confirmed via nRF Connect, Aug 2026) ---
  '0000d0ff-3c17-d293-8e48-14fe2e4da212', // vendor service A
  '00006287-3c17-d293-8e48-14fe2e4da212', // vendor service B
];

// Exact characteristic layout confirmed on this unit.
// A Ford module's response ID is its request ID + 8 (BCM 726 -> 72E).
const FORD_RESPONSE_ID = (h) => (parseInt(h, 16) + 8).toString(16).toUpperCase();

const VEEPEAK = {
  service: '0000fff0-0000-1000-8000-00805f9b34fb',
  tx:      '0000fff2-0000-1000-8000-00805f9b34fb', // WRITE / WRITE NO RESPONSE
  rx:      '0000fff1-0000-1000-8000-00805f9b34fb', // NOTIFY
};

class ELM327 {
  constructor() {
    this.device = null;
    this.txChar = null; // write
    this.rxChar = null; // notify
    this.buffer = '';
    this.pending = null; // {resolve, reject}
    this.connected = false;
  }

  // Shows the browser's device picker — required the first time (a real user
  // tap is mandatory here), but the permission it grants is remembered by
  // Chrome afterward, which is what lets connectToKnownDevice() skip it later.
  async connect(log) {
    log('Requesting BLE device…');
    // NOTE: this adapter advertises as "VEEPEAK", not "OBD...", so a
    // namePrefix:'OBD' filter hides it from the picker entirely. Show
    // everything and let the user pick; optionalServices is what actually
    // grants access to the vendor UART services once connected.
    const device = await navigator.bluetooth.requestDevice({
      acceptAllDevices: true,
      optionalServices: KNOWN_UART_SERVICES,
    });
    await this._connectToDevice(device, log);
  }

  // Silently reconnects to a device Chrome already has permission for,
  // with no picker dialog — used for auto-connect on app launch.
  async connectToKnownDevice(device, log) {
    await this._connectToDevice(device, log);
  }

  async _connectToDevice(device, log) {
    this.device = device;
    log(`Connecting to ${this.device.name || 'device'}…`);
    this.device.addEventListener('gattserverdisconnected', () => {
      this.connected = false;
      log('Disconnected.');
      if (typeof releaseWakeLock === 'function') releaseWakeLock();
    });

    const server = await this.device.gatt.connect();
    let services = await server.getPrimaryServices();

    // Order matters. The old code took the FIRST service exposing any
    // notify + any write characteristic, which on this adapter can latch
    // onto a vendor service that isn't the ELM327 UART at all — the link
    // comes up fine and then every write returns "GATT Error Unknown".
    // Try the known UART services in preference order first.
    const rank = (uuid) => {
      const i = KNOWN_UART_SERVICES.indexOf(uuid.toLowerCase());
      return i === -1 ? 999 : i;
    };
    services = services.slice().sort((a, b) => rank(a.uuid) - rank(b.uuid));

    // Dump everything we can see — if auto-detect still picks wrong, this
    // log tells us exactly which UUIDs to hardcode.
    for (const service of services) {
      const chars = await service.getCharacteristics();
      for (const ch of chars) {
        const p = ch.properties;
        const flags = [p.write && 'write', p.writeWithoutResponse && 'writeNR',
                       p.notify && 'notify', p.indicate && 'indicate',
                       p.read && 'read'].filter(Boolean).join(',');
        log(`  svc ${service.uuid.slice(0, 8)} char ${ch.uuid.slice(0, 8)} [${flags}]`);
      }
    }

    // --- Confirmed layout for this Veepeak (nRF Connect, Aug 2026) ---
    //   service 0xFFF0
    //     0xFFF2  WRITE, WRITE NO RESPONSE   -> TX (commands to the ELM327)
    //     0xFFF1  NOTIFY (CCCD 0x2902)       -> RX (responses back)
    // Try this exact pairing first; fall back to auto-detect if a future
    // adapter looks different.
    let found = false;
    try {
      const svc = await server.getPrimaryService(VEEPEAK.service);
      this.txChar = await svc.getCharacteristic(VEEPEAK.tx);
      this.rxChar = await svc.getCharacteristic(VEEPEAK.rx);
      found = true;
      log('Comms: FFF0 / tx FFF2 / rx FFF1 (known layout)');
    } catch (e) {
      log('Known FFF0 layout not present, falling back to auto-detect…');
    }

    for (const service of found ? [] : services) {
      const chars = await service.getCharacteristics();
      // Prefer write-without-response for TX (what these modules expect),
      // and never reuse the TX characteristic as RX.
      const txCandidates = chars.filter(c => c.properties.writeWithoutResponse || c.properties.write);
      txCandidates.sort((a, b) => (b.properties.writeWithoutResponse ? 1 : 0) - (a.properties.writeWithoutResponse ? 1 : 0));
      const candidateTx = txCandidates[0] || null;
      const candidateRx = chars.find(c => c.properties.notify && c !== candidateTx)
                       || chars.find(c => c.properties.notify)
                       || null;
      if (candidateTx && candidateRx) {
        this.txChar = candidateTx;
        this.rxChar = candidateRx;
        found = true;
        log(`Comms: svc ${service.uuid.slice(0, 8)} tx ${candidateTx.uuid.slice(0, 8)} rx ${candidateRx.uuid.slice(0, 8)}`);
        break;
      }
    }

    if (!found) {
      throw new Error('No writable+notifiable UART-style characteristic found on this device. ' +
        'Use nRF Connect to find the correct service/characteristic UUIDs and hardcode them if auto-detect fails.');
    }

    await this.rxChar.startNotifications();
    this.rxChar.addEventListener('characteristicvaluechanged', (e) => this._onData(e));

    this.connected = true;
    log('BLE link up. Initializing ELM327…');
    await this._initElm();
    log('ELM327 ready.');
    await this._readBaro(log);
    await this._probeSoot(log);
  }

  _onData(event) {
    const value = event.target.value;
    const text = new TextDecoder().decode(value);
    this.buffer += text;
    if (this.buffer.includes('>')) {
      const full = this.buffer;
      this.buffer = '';
      if (this.pending) {
        const { resolve } = this.pending;
        this.pending = null;
        resolve(full);
      }
    }
  }

  async _write(str) {
    const data = new TextEncoder().encode(str + '\r');
    // chunk to 20 bytes for BLE MTU safety
    for (let i = 0; i < data.length; i += 20) {
      const chunk = data.slice(i, i + 20);
      try {
        if (this.txChar.properties.writeWithoutResponse) {
          await this.txChar.writeValueWithoutResponse(chunk);
        } else {
          await this.txChar.writeValue(chunk);
        }
      } catch (err) {
        // Some stacks reject the "correct" write type; try the other one
        // before giving up, and name the characteristic in the error.
        try {
          if (this.txChar.properties.writeWithoutResponse) {
            await this.txChar.writeValue(chunk);
          } else {
            await this.txChar.writeValueWithoutResponse(chunk);
          }
        } catch (err2) {
          throw new Error(`write failed on char ${this.txChar.uuid.slice(0, 8)}: ${err.message}`);
        }
      }
    }
  }

  sendCommand(cmd, timeoutMs = 4000) {
    return new Promise(async (resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending = null;
        reject(new Error(`Timeout waiting for response to ${cmd}`));
      }, timeoutMs);
      this.pending = {
        resolve: (raw) => { clearTimeout(timer); resolve(raw); },
        reject,
      };
      try {
        await this._write(cmd);
      } catch (err) {
        clearTimeout(timer);
        this.pending = null;
        reject(err);
      }
    });
  }

  async _initElm() {
    const initCmds = ['ATZ', 'ATE0', 'ATL0', 'ATS0', 'ATH0', 'ATSP0'];
    for (const cmd of initCmds) {
      try { await this.sendCommand(cmd, 3000); } catch (e) { /* keep going, some adapters skip ATZ reply */ }
    }
  }

  // PID 0133 = absolute barometric pressure, one byte, kPa.
  async _readBaro(log) {
    try {
      const v = await this.readPid({ mode: '01', pid: '33', bytes: 1, decode: b => b[0] });
      if (v > 50 && v < 115) {
        BARO_PSI = v * 0.145038;
        log(`Barometric ${Math.round(v)} kPa (${BARO_PSI.toFixed(1)} psi) — boost zeroed to ambient.`);
      }
    } catch (e) {
      log('Baro PID unsupported, boost falls back to 14.7 psi reference.');
    }
  }

  // Ask the truck which PID it actually answers instead of assuming.
  async _probe(key, candidates, label, log, sane) {
    for (const cand of candidates) {
      try {
        const v = await this.readPid(cand);
        if (Number.isFinite(v) && (!sane || sane(v))) {
          PIDS[key] = { ...PIDS[key], ...cand };
          log(`${label}: ${cand.mode}${cand.pid} answered ${v.toFixed(1)} — using it.`);
          return true;
        }
      } catch (e) { /* try the next candidate */ }
    }
    log(`${label}: nothing answered. FORScan can show the right PID for a 2021.`);
    return false;
  }

  async _probeSoot(log) {
    await this._probe('soot', SOOT_CANDIDATES, 'Soot', log, v => v >= -5 && v <= 105);
    await this._probe('def', DEF_CANDIDATES, 'DEF', log, v => v >= 0 && v <= 100);
    await this._probe('oilpress', OILPRESS_CANDIDATES, 'Oil pressure', log, v => v >= 0 && v <= 120);
  }

  // Ford modules other than the PCM answer on their own CAN IDs — FORScan
  // switches with ATSH (e.g. ATSH 726 = BCM, 7E0 = PCM). Any PID carrying a
  // `header` gets the ELM re-pointed before the request. This is what an
  // upfitter-switch readout would need.
  async _setHeader(header) {
    if (this._curHeader === header) return;
    await this.sendCommand('ATSH ' + header, 2000);
    if (header !== '7E0') await this.sendCommand('ATCRA ' + FORD_RESPONSE_ID(header), 2000).catch(() => {});
    else await this.sendCommand('ATCRA', 2000).catch(() => {});
    this._curHeader = header;
  }

  async readPid(def) {
    await this._setHeader(def.header || '7E0');
    const cmd = def.mode + def.pid;
    const raw = await this.sendCommand(cmd, 2500);
    return this._parseResponse(raw, def);
  }

  _parseResponse(raw, def) {
    // Strip whitespace/CR/prompt, keep hex pairs only
    const cleaned = raw.replace(/[\r\n>]/g, ' ').trim();
    const hexOnly = cleaned.replace(/[^0-9A-Fa-f ]/g, '').split(/\s+/).filter(Boolean);

    const expectedHeader = (def.mode === '01' ? '41' : '62') + def.pid.toUpperCase();
    const joined = hexOnly.join('');
    const idx = joined.toUpperCase().indexOf(expectedHeader);
    if (idx === -1) throw new Error(`No valid response for ${def.mode}${def.pid}: "${raw}"`);

    const dataHex = joined.slice(idx + expectedHeader.length, idx + expectedHeader.length + def.bytes * 2);
    const bytes = [];
    for (let i = 0; i < dataHex.length; i += 2) {
      bytes.push(parseInt(dataHex.slice(i, i + 2), 16));
    }
    if (bytes.length < def.bytes) throw new Error(`Short response for ${def.mode}${def.pid}`);
    return def.decode(bytes);
  }
}

// ---- Dashboard wiring -------------------------------------------
const elm = new ELM327();
const statusContainer = document.getElementById('connStatus');
const statusText = document.getElementById('statusText');
const connectBtn = document.getElementById('connectBtn');
const logEl = document.getElementById('debugLog');

function log(msg) {
  console.log(msg);
  if (logEl) {
    logEl.textContent = msg + '\n' + logEl.textContent;
  }
}

// Elements are now found by [data-metric="key"] instead of a single id,
// since the same reading (e.g. coolant temp) can appear on more than one
// swipeable page at once. querySelectorAll + forEach keeps every copy in sync.

function setGaugeValue(els, value, def) {
  els.forEach((el) => {
    const valEl = el.querySelector('.val');
    if (valEl) {
      // full gauge widget (dial + fill arc)
      valEl.textContent = Math.round(value);
      const pct = Math.max(0, Math.min(1, (value - def.min) / (def.max - def.min)));
      const fill = el.querySelector('.gauge-fill');
      if (fill) fill.style.strokeDashoffset = 314 - pct * 314;
    } else {
      // plain text readout (e.g. a temp shown as a simple number on a summary page)
      el.textContent = Math.round(value) + def.unit;
    }
  });
}

// 4WD page: drive mode is a state, not a number, so it lights one of the
// three chips rather than driving a gauge. Wired up but inert until we have
// real PIDs — see the note in the README about pulling them from FORScan.
function applyDriveMode(mode) {
  document.querySelectorAll('.mode').forEach((el) => {
    el.classList.toggle('on', el.dataset.mode === mode);
  });
  const tc = document.querySelector('[data-metric="tcase"]');
  if (tc) tc.textContent = mode || '--';
}

function applyDiffLock(locked) {
  const el = document.querySelector('[data-metric="difflock"]');
  if (!el) return;
  el.textContent = locked ? 'LOCKED' : 'OPEN';
  el.classList.toggle('on', !!locked);
}

// The turbo animation lives in an iframe, so boost is handed over by
// postMessage rather than a direct call. Guarded so the dash still runs fine
// if turbo.html or its assets aren't uploaded yet.
function pushBoostToTurbo(psi) {
  const f = document.getElementById('turboFrame');
  if (f && f.contentWindow) f.contentWindow.postMessage({ type: 'boost', psi }, '*');
}

function applyReading(key, value) {
  if (key === 'drivemode') return applyDriveMode(value);
  if (key === 'difflock') return applyDiffLock(value);
  if (key === 'boost') pushBoostToTurbo(value);
  const def = PIDS[key];
  const els = document.querySelectorAll(`[data-metric="${key}"]`);
  if (def.kind === 'text') {
    els.forEach((el) => { el.textContent = Math.round(value); });
  } else if (def.kind === 'rpm') {
    els.forEach((el) => { el.textContent = Math.round(value).toLocaleString(); });
    document.querySelectorAll('[data-metric="rpmBar"]').forEach((bar) => {
      bar.style.width = Math.min(100, (value / 5000) * 100) + '%';
    });
  } else if (def.kind === 'gauge') {
    setGaugeValue(els, value, def);
  } else if (def.kind === 'bar') {
    els.forEach((el) => {
      const fill = el.querySelector('.fuel-bar-fill');
      const num = el.querySelector('.num');
      if (fill) fill.style.width = Math.max(0, Math.min(100, value)) + '%';
      if (num) num.textContent = Math.round(value) + '%';
    });
  }
}

async function pollLoop() {
  while (elm.connected) {
    for (const key of POLL_ORDER) {
      if (!elm.connected) break;
      try {
        const value = await elm.readPid(PIDS[key]);
        applyReading(key, value);
      } catch (err) {
        log(`${key}: ${err.message}`);
      }
      await new Promise(r => setTimeout(r, 60));
    }
  }
}

connectBtn.addEventListener('click', async () => {
  if (!navigator.bluetooth) {
    statusText.textContent = 'Web Bluetooth not supported in this browser';
    return;
  }
  connectBtn.disabled = true;
  statusText.textContent = 'Connecting…';
  try {
    await elm.connect(log);
    await markConnected();
  } catch (err) {
    statusText.textContent = 'Connection failed: ' + err.message;
    log('ERROR: ' + err.message);
  } finally {
    connectBtn.disabled = false;
  }
});

async function markConnected() {
  statusText.textContent = 'Connected — live data';
  statusContainer.classList.add('live');
  requestWakeLock();
  pollLoop();
}

// Silent auto-connect: Chrome remembers devices you've granted permission
// to via the picker (navigator.bluetooth.getDevices()). If the OBDII dongle
// is already authorized and currently in range/on, reconnect with zero taps.
// If not (first run ever, or the dongle isn't visible yet), this just does
// nothing and the CONNECT OBD button remains available as normal.
async function tryAutoConnect() {
  if (!navigator.bluetooth || !navigator.bluetooth.getDevices) return;
  try {
    const known = await navigator.bluetooth.getDevices();
    // Same trap as the picker: this adapter is named "VEEPEAK", so a
    // startsWith('OBD') test silently skipped it and auto-reconnect never
    // fired. Prefer a known-looking name, otherwise just take the first
    // device we've been granted — there's normally only one.
    const looksRight = (d) => /OBD|VEEPEAK|ELM/i.test(d.name || '');
    const obd = known.find(looksRight) || known[0];
    if (!obd) return;
    statusText.textContent = 'Reconnecting to ' + obd.name + '…';
    await elm.connectToKnownDevice(obd, log);
    await markConnected();
  } catch (err) {
    // Dongle not powered/in range, or needs the picker again — silently fall
    // back to manual connect, no error shown since this was an automatic attempt.
    log('Auto-connect skipped: ' + err.message);
  }
}
window.addEventListener('load', tryAutoConnect);

// ---- Keep the screen awake -------------------------------------
// Screen Wake Lock is dropped by the browser whenever the page is hidden
// (app switch, screen off, incoming call), and it does NOT come back on its
// own — so re-request it every time we become visible again.
let wakeLock = null;

async function requestWakeLock() {
  if (!('wakeLock' in navigator)) {
    log('Screen Wake Lock not supported by this browser.');
    return;
  }
  if (document.visibilityState !== 'visible') return;
  try {
    wakeLock = await navigator.wakeLock.request('screen');
    log('Screen will stay on.');
    wakeLock.addEventListener('release', () => { wakeLock = null; });
  } catch (err) {
    // Most common cause is the OS refusing under battery saver.
    log('Could not keep screen on: ' + err.message);
  }
}

async function releaseWakeLock() {
  try { if (wakeLock) await wakeLock.release(); } catch (e) { /* already gone */ }
  wakeLock = null;
}

document.addEventListener('visibilitychange', () => {
  if (document.visibilityState === 'visible' && !wakeLock) requestWakeLock();
});

window.addEventListener('load', requestWakeLock);

// register service worker for offline install
if ('serviceWorker' in navigator) {
  window.addEventListener('load', () => {
    navigator.serviceWorker.register('./sw.js').catch(() => {});
  });
}
