/* ===========================================================
   F350 Diesel Dashboard — Web Bluetooth + ELM327 driver
   Talks to a BLE OBD2 adapter (tested against Veepeak OBDCheck BLE)
   =========================================================== */

// ---- PID TABLE ------------------------------------------------
// mode 01 = standard OBD2, mode 22 = Ford manufacturer-specific.
// Formulas verified against community Torque Pro PID logs for the
// 2011+ Ford 6.7L Powerstroke — sanity-check against real readings
// once connected (e.g. coolant should settle ~190-210F warmed up).
const PIDS = {
  speed:   { mode: '01', pid: '0D', bytes: 1, decode: b => b[0], unit: 'MPH', gauge: 'speedVal', kind: 'text' },
  rpm:     { mode: '01', pid: '0C', bytes: 2, decode: b => ((b[0] * 256) + b[1]) / 4, unit: 'RPM', gauge: 'rpmVal', kind: 'rpm' },
  coolant: { mode: '01', pid: '05', bytes: 1, decode: b => (b[0] - 40) * 9 / 5 + 32, unit: '°F', gauge: 'g-coolant', kind: 'gauge', min: 100, max: 260 },
  fuel:    { mode: '01', pid: '2F', bytes: 1, decode: b => (b[0] / 255) * 100, unit: '%', gauge: 'fuelPct', kind: 'bar' },
  egt:     { mode: '22', pid: 'F478', bytes: 8, decode: b => (((b[0] * 256) + b[1]) * 0.18) - 40, unit: '°F', gauge: 'g-egt', kind: 'gauge', min: 300, max: 1600 },
  boost:   { mode: '01', pid: '87', bytes: 2, decode: b => ((((b[0] * 256) + b[1]) * 0.00393) + 2.25) - 14.7, unit: 'PSI', gauge: 'g-boost', kind: 'gauge', min: 0, max: 35 },
  soot:    { mode: '22', pid: '042C', bytes: 2, decode: b => ((((b[0] * 256) + b[1]) * (100 / 65535)) - 1) / 1.75 * 100, unit: '%', gauge: 'g-soot', kind: 'gauge', min: 0, max: 100 },
  trans:   { mode: '22', pid: '1E1C', bytes: 2, decode: b => (((b[0] << 24 >> 24) * 256) + b[1]) * (9 / 80) + 32, unit: '°F', gauge: 'g-trans', kind: 'gauge', min: 100, max: 260 },
  def:     { mode: '22', pid: 'F485', bytes: 1, decode: b => (b[0] / 15) * 100, unit: '%', gauge: 'defPct', kind: 'bar' },
};

const POLL_ORDER = ['speed', 'rpm', 'coolant', 'fuel', 'egt', 'boost', 'soot', 'trans', 'def'];

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
];

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
    const device = await navigator.bluetooth.requestDevice({
      filters: [{ namePrefix: 'OBD' }],
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
    });

    const server = await this.device.gatt.connect();
    const services = await server.getPrimaryServices();

    let found = false;
    for (const service of services) {
      const chars = await service.getCharacteristics();
      let candidateTx = null, candidateRx = null;
      for (const ch of chars) {
        if (ch.properties.notify) candidateRx = ch;
        if (ch.properties.write || ch.properties.writeWithoutResponse) candidateTx = ch;
      }
      if (candidateTx && candidateRx) {
        this.txChar = candidateTx;
        this.rxChar = candidateRx;
        found = true;
        log(`Using service ${service.uuid.slice(0, 8)}… for comms.`);
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
      if (this.txChar.properties.writeWithoutResponse) {
        await this.txChar.writeValueWithoutResponse(chunk);
      } else {
        await this.txChar.writeValue(chunk);
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

  async readPid(def) {
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

function applyReading(key, value) {
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
    const obd = known.find((d) => (d.name || '').toUpperCase().startsWith('OBD'));
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

// register service worker for offline install
if ('serviceWorker' in navigator) {
  window.addEventListener('load', () => {
    navigator.serviceWorker.register('./sw.js').catch(() => {});
  });
}
