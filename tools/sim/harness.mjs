// IS25 firmware-in-the-loop harness on rp2040js (single core, SMP=0 build).
// Simulates the car around the RP2040: motors (PWM slices 4/5) -> wheel speed
// -> encoder edges on GP0..GP3, robot pose -> IR sensors on ADC0..2, HC-SR04
// on GP4/GP5, LSM303D IMU on I2C0 (or the old LSM303DLHC / GY-511), buttons
// GP20/GP21.  Nov 2026: LSM303D model, lift / overturn, current sensor layout.
// Usage: node harness.mjs <image.uf2> <seconds> [scenario-json]
import fs from 'fs';
import { RP2040, USBCDC, ConsoleLogger, LogLevel } from 'rp2040js';
import { SimulationClock } from './node_modules/rp2040js/dist/esm/clock/simulation-clock.js';
import { bootromB1 } from './bootrom.mjs';

const [imagePath, secondsArg, scenarioArg] = process.argv.slice(2);
const seconds = Number(secondsArg ?? 10);
const sc = Object.assign({
  battery: true,          // motors can turn
  leftPin: 0,             // GPIO carrying the left encoder pulses (0 or 1)
  rightPin: 3,            // GPIO carrying the right encoder pulses (2 or 3)
  edgesPerRev: 1270,      // encoder edges per wheel turn (car: 1270, 30 Oct)
  imuChip: 'd',           // 'd' = LSM303D (the car's module), 'dlhc' = GY-511
  imuAddr: 0x1d,          // LSM303D address: 0x1d (SA0 high) or 0x1e (SA0 low)
  imu: 'ok',              // 'ok' | 'absent' (nothing on Grove 4) | 'unpowered'
                          // (on the bus without power: both lines held low,
                          // hardware transfers hang) | 'swapped' (SDA/SCL crossed)
  imuOffFromS: -1,        // s: GY-511 unplugged from this time ...
  imuOffToS: -1,          // ... and plugged back in at this time
  obstacleCm: 120,        // distance ahead (cm); 0 = nothing (38 ms echo)
  boxX: -1,               // mm: a box obstacle on the track from this x (near
  boxD: 100,              //   face) ... this deep (along the line) ...
  boxW: 100,              //   ... this wide, centred on y = boxY.  The sonar
  boxY: 0,                //   on the servo (GP15) is ray-cast against it, so
  sonarFwdMm: 80,         //   the scan and the bypass can be tested; it sits
                          //   this far ahead of the axle.
  servoFrontUs: 2200,     // GP15 pulse that points the sonar straight ahead
                          // (IS25 car, 17 Nov servo trim; 1500 = textbook)
  servoMinUs: 1850,       // safe limits on the car: a pulse outside them
  servoMaxUs: 2500,       // would press the sensor against the car (reported)
  pressStartAt: -1,       // s: press GP20 (start) at this time
  pressResetAt: -1,       // s: press GP21 (reset)
  liftedFromS: -1,        // s: robot lifted (wheels free, IR sees black):
  liftMg: 200,            //    a hand pushes it up at this many mg ...
  liftPushS: 0.3,         //    ... for this long, then brakes it as long
  overturnFromS: -1,      // s: robot rolled over (1 s) onto its roof
  layout: 'arm',          // 'arm' (Nov 2026): GP26 just right of the line,
                          // GP27 on its left edge, GP28 = barcode arm armMm
                          // further left, labels 20 mm left of the line.
                          // 'old': the 28 Oct layout (barcode on the right)
  armMm: 40,              // arm sensor, mm left of the centre sensor
  narrowMm: 3,            // Code 39 narrow element (wide = 3 x)
  handTurnFromS: -1,      // s: someone turns both wheels by hand ...
  handTurnToS: -1,        // ... until this time ...
  handTurnRevs: 10,       // ... exactly this many turns each
  quiet: false,
  profile: false,         // sample the PC every 10 us -> profile.json
  obstacleFromS: 0,       // s: obstacleCm applies from this time (before: none)
  lineEndX: -1,           // mm: the line stops here (> 0)
  barcode: '',            // e.g. 'A': Code 39 *A* beside the line at x = 300 mm
  branchLeftAtX: -1,      // mm: a branch line leaving to the left at this x
  spotMm: 3,              // IR sensor spot diameter on the floor (optics)
  rightMm: 20,            // mm: right guard sensor right of the centre one ('arm' layout)
  cornerAtX: -1,          // mm: the line turns 90 deg here instead of going on ...
  cornerDir: 'left',      // ... 'left' or 'right' ...
  cornerR: 100,           // ... on an arc of this radius (mm, line centre)
  motorScaleL: 1,         // left motor strength (0.9 = 10 % weaker, a real car drifts)
  traceMs: -1,            // > 0: print 'T t x y heading vL vR' this often (wobble checks)
  accNoiseMg: 15,         // accelerometer noise (motor vibration), mg rms-ish
  pushFromS: -1,          // s: robot pushed straight ahead by hand ...
  pushToS: -1,            // ... until this time ...
  pushMmS: 150,           // ... at this speed (mm/s)
  startX: 0,              // mm: start position along the line
  sweepFromS: -1,         // s: car slid sideways by hand across the line ...
  sweepToS: -1,           // ... until this time (y = 12 +- 45 mm, 2 s period)
  spinFromS: -1,          // s: car turned on the spot by hand ...
  spinToS: -1,            // ... until this time ...
  spinRevs: 1,            // ... this many full circles
  noseUpFromS: -1,        // s: nose lifted ...
  noseUpToS: -1,          // ... until this time ...
  noseUpDeg: 10,          // ... by this angle
  leftOkFromS: -1,        // s: left encoder gives no pulses before this time
  sensorFwdMm: 60,        // mm: IR sensors ahead of the wheel axle
  mirror: false,          // true: barcode sensor on the car's LEFT, the
                          // inner line sensor on the line's RIGHT edge
}, scenarioArg ? JSON.parse(scenarioArg) : {});

// ---------------------------------------------------------------- MCU
const clock = new SimulationClock();
const mcu = new RP2040(clock);
mcu.loadBootrom(bootromB1);
mcu.logger = new ConsoleLogger(LogLevel.Error);

// UF2 loader
{
  const buf = fs.readFileSync(imagePath);
  for (let off = 0; off + 512 <= buf.length; off += 512) {
    const b = buf.subarray(off, off + 512);
    const m0 = b.readUInt32LE(0), m1 = b.readUInt32LE(4);
    if (m0 !== 0x0a324655 || m1 !== 0x9e5d5157) continue;
    const addr = b.readUInt32LE(12), size = b.readUInt32LE(16);
    mcu.flash.set(b.subarray(32, 32 + size), addr - 0x10000000);
  }
}

// ---------------------------------------------------------------- I2C fix-ups
// rp2040js's I2C has no IC_INTR_MASK write and edge-only TX_EMPTY/RX_FULL;
// the micro T-Kernel driver is interrupt driven, so make them level based.
const R_TX_EMPTY = 1 << 4, R_RX_FULL = 1 << 2;
for (const i2c of mcu.i2c) {
  Object.defineProperty(i2c, 'intStatus', {
    get() {
      let raw = this.intRaw & ~(R_TX_EMPTY | R_RX_FULL);
      if ((this.enable & 1) && this.txFIFO.itemCount <= this.txThreshold && !this.busy) raw |= R_TX_EMPTY;
      if (this.rxFIFO.itemCount > this.rxThreshold) raw |= R_RX_FULL;
      return raw & this.intEnable;
    },
  });
  const origWrite = i2c.writeUint32.bind(i2c);
  i2c.writeUint32 = (offset, value) => {
    if (offset === 0x30) { i2c.intEnable = value & 0x1fff; i2c.checkInterrupts(); return; }
    if (offset === 0x88 || offset === 0x7c) return;   // DMA_CR, SDA_HOLD: ignore
    origWrite(offset, value);
    i2c.checkInterrupts();
  };
  const origRead = i2c.readUint32.bind(i2c);
  i2c.readUint32 = (offset) => { const v = origRead(offset); i2c.checkInterrupts(); return v; };
  for (const name of ['completeStart', 'completeConnect', 'completeWrite', 'completeRead', 'completeStop', 'nextCommand']) {
    const f = i2c[name].bind(i2c);
    i2c[name] = (...a) => { const r = f(...a); i2c.checkInterrupts(); return r; };
  }
}

// IMU.  LSM303D: one address (0x1D or 0x1E), WHO_AM_I 0x0F = 0x49, accel
// 16-bit at 0x28 (0.122 mg/LSB at +-4 g), mag 16-bit at 0x08, little-endian
// X, Y, Z.  LSM303DLHC (GY-511): accel 0x19, mag 0x1E (X, Z, Y big-endian).
const D = sc.imuChip === 'd';
const ADDRS = D ? [sc.imuAddr] : [0x19, 0x1e];
const imu = { regs: {}, addr: 0, ptr: 0, first: false, transfers: 0 };
if (D) { imu.regs[sc.imuAddr] = new Uint8Array(0x40); imu.regs[sc.imuAddr][0x0f] = 0x49; }
else {
  imu.regs[0x19] = new Uint8Array(0x40); imu.regs[0x1e] = new Uint8Array(0x10);
  imu.regs[0x1e][0x0a] = 0x48; imu.regs[0x1e][0x0b] = 0x34; imu.regs[0x1e][0x0c] = 0x33;
}
{
  const i2c = mcu.i2c[0];
  // A line held low (module without power): the controller waits for the
  // bus forever, so the kernel driver's 1 s timeout ends the transfer.
  i2c.onStart = () => { if (sc.imu !== 'unpowered') i2c.completeStart(); };
  i2c.onConnect = (address) => {
    const present = sc.imu === 'ok' && imuPlugged() && ADDRS.includes(address);
    imu.addr = address; imu.first = true; imu.transfers++;
    i2c.completeConnect(present);
  };
  i2c.onWriteByte = (value) => {
    const r = imu.regs[imu.addr];
    if (imu.first) { imu.ptr = value; imu.first = false; }
    else if (r) { r[imu.ptr & (r.length - 1) & 0x7f] = value; imu.ptr = autoInc(imu.ptr); }
    i2c.completeWrite(true);
  };
  i2c.onReadByte = () => {
    refreshImu();
    const r = imu.regs[imu.addr];
    const v = r ? r[(imu.ptr & 0x7f) & (r.length - 1)] : 0xff;
    imu.ptr = autoInc(imu.ptr);
    i2c.completeRead(v);
  };
  i2c.onStop = () => i2c.completeStop();
}
function imuPlugged() {
  const t = clock.nanos / 1e9;
  return !(sc.imuOffFromS >= 0 && t >= sc.imuOffFromS && t < sc.imuOffToS);
}
// The two Grove 4 wires (GP16, GP17) as an open-drain bus, for the firmware's
// bit-banged bus clear, presence check and wiring check (the emulated I2C
// controller does not move the pins).  The module answers its address(es)
// with an ACK; with 'swapped' it sees SDA on GP17 and SCL on GP16.
const bus = { sdaPin: sc.imu === 'swapped' ? 17 : 16, sclPin: sc.imu === 'swapped' ? 16 : 17,
  state: 'idle', bits: 0, byte: 0, ack: false, sda: true, scl: true, acks: 0 };
function wireLevel(p) {
  if (sc.imu === 'unpowered' && imuPlugged()) return false;   // its pull-ups to 0 V
  if (mcu.gpio[p].value === 0) return false;                   // the Pico drives it low
  if (bus.ack && p === bus.sdaPin) return false;               // the module's ACK
  return true;
}
function busUpdate() {
  for (let pass = 0; pass < 2; pass++) {
    const sda = wireLevel(bus.sdaPin), scl = wireLevel(bus.sclPin);
    const live = (sc.imu === 'ok' || sc.imu === 'swapped') && imuPlugged();
    if (scl && bus.scl && sda !== bus.sda) {            // START or STOP
      if (!sda) { bus.state = 'addr'; bus.bits = 0; bus.byte = 0; }
      else { bus.state = 'idle'; }
      bus.ack = false;
    } else if (scl && !bus.scl) {                       // SCL rising
      if (bus.state === 'addr' && bus.bits < 8) { bus.byte = (bus.byte << 1) | (sda ? 1 : 0); bus.bits++; }
    } else if (!scl && bus.scl) {                       // SCL falling
      if (bus.state === 'addr' && bus.bits === 8) {
        const a = bus.byte >> 1;
        if (live && ADDRS.includes(a)) { bus.ack = true; bus.state = 'ack'; bus.acks++; }
        else bus.state = 'idle';
      } else if (bus.state === 'ack') { bus.ack = false; bus.state = 'data'; }
    }
    bus.sda = sda; bus.scl = scl;
  }
  mcu.gpio[16].setInputValue(wireLevel(16));
  mcu.gpio[17].setInputValue(wireLevel(17));
}
function autoInc(p) {
  if (D || imu.addr === 0x19) return (p & 0x80) ? (((p & 0x7f) + 1) | 0x80) : p;
  return (p + 1) & 0x0f;
}
function liftAccMg(tS) {
  // A hand lifting the car: up at liftMg for liftPushS, then the same down.
  if (sc.liftedFromS < 0 || tS < sc.liftedFromS) return 0;
  const k = tS - sc.liftedFromS;
  if (k < sc.liftPushS) return sc.liftMg;
  if (k < 2 * sc.liftPushS) return -sc.liftMg;
  return 0;
}
function rollRad(tS) {
  if (sc.overturnFromS < 0 || tS < sc.overturnFromS) return 0;
  return Math.PI * Math.min(1, tS - sc.overturnFromS);
}
function refreshImu() {
  const nz = () => (Math.random() - 0.5) * 2 * sc.accNoiseMg;
  const tS = clock.nanos / 1e9;
  const pitch = (sc.noseUpFromS >= 0 && tS >= sc.noseUpFromS && tS < sc.noseUpToS) ? sc.noseUpDeg * Math.PI / 180 : 0;
  const roll = rollRad(tS);
  const vert = 1000 + liftAccMg(tS);
  // an axis pointing up reads +1 g: nose up -> +X (forward) reads +sin(pitch)
  const ax = Math.round(vert * Math.sin(pitch) + robot.accFwd / 9.80665 + nz());
  const ay = Math.round(vert * Math.sin(roll) + nz());
  const az = Math.round(vert * Math.cos(pitch) * Math.cos(roll) + nz());
  const mx = Math.round(400 * Math.cos(-robot.th)), my = Math.round(400 * Math.sin(-robot.th)), mz = -300;
  if (D) {
    const r = imu.regs[sc.imuAddr];
    [ax, ay, az].forEach((mg, i) => { const raw = Math.max(-32768, Math.min(32767, Math.round(mg / 0.122))) & 0xffff; r[0x28 + 2 * i] = raw & 0xff; r[0x29 + 2 * i] = raw >> 8; });
    [mx * 6, my * 6, mz * 6].forEach((v, i) => { const raw = v & 0xffff; r[0x08 + 2 * i] = raw & 0xff; r[0x09 + 2 * i] = raw >> 8; });
  } else {
    const a = imu.regs[0x19];
    [ax, ay, az].forEach((mg, i) => { const raw = (Math.round(mg / 2) << 4) & 0xffff; a[0x28 + 2 * i] = raw & 0xff; a[0x29 + 2 * i] = raw >> 8; });
    const m = imu.regs[0x1e];
    [mx, mz, my].forEach((v, i) => { const raw = v & 0xffff; m[0x03 + 2 * i] = raw >> 8; m[0x04 + 2 * i] = raw & 0xff; });
  }
}

// ---------------------------------------------------------------- USB console
const cdc = new USBCDC(mcu.usbCtrl);
// The emulated host answers every OUT poll at once with 0 bytes, which turns
// into an interrupt storm a real USB host never causes (it NAKs in hardware).
// Hold OUT polls while the host has nothing to send.
{
  const hostRead = mcu.usbCtrl.onEndpointRead;
  mcu.usbCtrl.onEndpointRead = (endpoint, size) => {
    if (endpoint !== 0 && cdc.txFIFO.itemCount === 0) return;
    hostRead(endpoint, size);
  };
}
let line = '';
const lines = [];
const t0 = () => (clock.nanos / 1e9).toFixed(3);
cdc.onSerialData = (data) => {
  for (const byte of data) {
    if (byte === 10) { lines.push(`[${t0()}] ${line}`); if (!sc.quiet) console.log(`[${t0()}] ${line}`); line = ''; }
    else if (byte !== 13) line += String.fromCharCode(byte);
  }
};

// ---------------------------------------------------------------- robot model
const M = sc.mirror ? -1 : 1;
const robot = { x: sc.startX, y: 9 * M, th: 0, vL: 0, vR: 0, edgeAccL: 0, edgeAccR: 0, levelL: true, levelR: true, accFwd: 0 };
const WHEEL_CIRC = Math.PI * 65, TRACK = 120, VMAX = 450, DEAD = 0.12, TAU = 0.06;
const pins = mcu.gpio;
for (const p of [0, 1, 2, 3, 20, 21]) pins[p].setInputValue(true);   // pulled up
pins[5].setInputValue(false);
pins[16].addListener(() => busUpdate());
pins[17].addListener(() => busUpdate());
busUpdate();

function pwmDuty(slice) {
  const base = 0x40050000 + 0x14 * slice;
  const csr = mcu.readUint32(base), top = mcu.readUint32(base + 0x10) & 0xffff, cc = mcu.readUint32(base + 0x0c);
  if (!(csr & 1)) return [0, 0];
  const a = Math.min(1, (cc & 0xffff) / (top + 1)), b = Math.min(1, (cc >>> 16) / (top + 1));
  return [a, b];
}
function motorTarget(a, b) {
  if (!sc.battery) return 0;
  if (a >= 1 && b >= 1) return null;       // coast
  const drive = a - b;
  const mag = Math.max(0, (Math.abs(drive) - DEAD) / (1 - DEAD));
  return Math.sign(drive) * mag * VMAX;
}
// World: black line |y| <= 9 along x; optional Code 39 barcode beside it on
// the right (y -55..-25) from x = 300 mm; optional branch to the left.
const C39 = { '*': 0x094, A: 0x109, B: 0x049, C: 0x148, D: 0x019, E: 0x118 };
const bars = [];
if (sc.barcode) {
  let x = 300;
  const chars = ['*', sc.barcode, '*'];
  chars.forEach((ch, ci) => {
    for (let e = 0; e < 9; e++) {
      const w = (C39[ch] >> (8 - e)) & 1 ? 3 * sc.narrowMm : sc.narrowMm;
      if (e % 2 === 0) bars.push([x, x + w]);
      x += w;
    }
    if (ci < 2) x += sc.narrowMm;
  });
}
function onCorner(xw, yw) {
  // Mirror a right corner onto the left one (y -> -y).
  const X = sc.cornerAtX, R = sc.cornerR, y = sc.cornerDir === 'right' ? -yw : yw;
  if (xw <= X) return Math.abs(y) <= 9;                       // straight part
  if (y <= R) {                                               // the arc
    const d = Math.hypot(xw - X, y - R);
    return xw <= X + R + 9 && Math.abs(d - R) <= 9;
  }
  return Math.abs(xw - (X + R)) <= 9;                         // after the turn
}
function black(xw, yw) {
  if (sc.cornerAtX >= 0) { if (onCorner(xw, yw)) return true; }
  else if (Math.abs(yw) <= 9 && !(sc.lineEndX > 0 && xw > sc.lineEndX)) return true;
  if (sc.branchLeftAtX >= 0 && Math.abs(xw - sc.branchLeftAtX) <= 9 && yw >= 0 && yw <= 400) return true;
  const inLabel = sc.layout === 'arm' ? (M * yw >= 29 && M * yw <= 99) : (M * yw >= -55 && M * yw <= -25);
  if (inLabel) for (const [a, b] of bars) if (xw >= a && xw <= b) return true;
  return false;
}
function irValue(lateralMm) {
  const fwd = sc.sensorFwdMm;
  const xw = robot.x + fwd * Math.cos(robot.th) - lateralMm * Math.sin(robot.th);
  const yw = robot.y + fwd * Math.sin(robot.th) + lateralMm * Math.cos(robot.th);
  // round spot of sc.spotMm diameter, sampled on a 0.5 mm grid
  let hits = 0, n = 0;
  const r = sc.spotMm / 2;
  for (let i = -r; i <= r + 1e-9; i += 0.5) for (let j = -r; j <= r + 1e-9; j += 0.5) {
    if (i * i + j * j > r * r + 1e-9) continue;
    n++; if (black(xw + i, yw + j)) hits++;
  }
  const cover = n ? hits / n : 0;
  const lifted = sc.liftedFromS >= 0 && clock.nanos / 1e9 >= sc.liftedFromS;
  return Math.round(lifted ? 3200 : 300 + 2700 * cover + (Math.random() - 0.5) * 40);
}
let lastNs = 0;
const events = [];        // scheduled pin changes: {ns, pin, value}
let trigHighAt = -1;
// Servo angle from the GP15 pulse (PWM slice 7 B, 1 MHz tick), 11.1 us per
// degree: servoFrontUs = ahead (90 deg), less = the car's right, more = left.
const servoSeen = { min: Infinity, max: -Infinity, outside: 0 };
function servoCc() { return mcu.readUint32(0x40050000 + 0x14 * 7 + 0x0c) >>> 16; }
function servoWatch() {
  const cc = servoCc();
  if (cc === 0) return;                     // not driven yet
  servoSeen.min = Math.min(servoSeen.min, cc); servoSeen.max = Math.max(servoSeen.max, cc);
  if (cc < sc.servoMinUs || cc > sc.servoMaxUs) servoSeen.outside++;
}
function servoDeg() {
  const cc = servoCc();
  if (cc < 400 || cc > 2600) return 90;
  return 90 + (cc - sc.servoFrontUs) * 180 / 2000;
}
// Distance (cm) the HC-SR04 reports for the box: rays across its ~15 deg
// cone from the sonar, nearest hit; 0 = nothing within 2.5 m.
function boxRangeCm() {
  const a0 = robot.th + (servoDeg() - 90) * Math.PI / 180;
  const ox = robot.x + sc.sonarFwdMm * Math.cos(robot.th), oy = robot.y + sc.sonarFwdMm * Math.sin(robot.th);
  const x0 = sc.boxX, x1 = sc.boxX + sc.boxD, y0 = sc.boxY - sc.boxW / 2, y1 = sc.boxY + sc.boxW / 2;
  let best = Infinity;
  for (let k = -7.5; k <= 7.5; k += 1.5) {
    const a = a0 + k * Math.PI / 180, dx = Math.cos(a), dy = Math.sin(a);
    let tmin = 0, tmax = 2500;                    // slab test, mm
    for (const [o, d, lo, hi] of [[ox, dx, x0, x1], [oy, dy, y0, y1]]) {
      if (Math.abs(d) < 1e-9) { if (o < lo || o > hi) { tmin = Infinity; } continue; }
      let t1 = (lo - o) / d, t2 = (hi - o) / d;
      if (t1 > t2) [t1, t2] = [t2, t1];
      tmin = Math.max(tmin, t1); tmax = Math.min(tmax, t2);
    }
    if (tmin <= tmax && tmin < best) best = tmin;
  }
  return best === Infinity ? 0 : Math.max(2, Math.round(best / 10));
}
pins[4].addListener((value) => {
  if (value) trigHighAt = clock.nanos;
  else if (trigHighAt >= 0) {
    const obstacle = sc.boxX >= 0 ? boxRangeCm() :
                     (clock.nanos / 1e9 >= sc.obstacleFromS ? sc.obstacleCm : 0);
    const widthUs = obstacle > 0 ? obstacle * 58 : 38000;
    events.push({ ns: clock.nanos + 200e3, pin: 5, value: true });
    events.push({ ns: clock.nanos + 200e3 + widthUs * 1e3, pin: 5, value: false });
    trigHighAt = -1;
  }
});
function press(pin, atS) { if (atS >= 0) { events.push({ ns: atS * 1e9, pin, value: false }); events.push({ ns: (atS + 0.3) * 1e9, pin, value: true }); } }
press(20, sc.pressStartAt); press(21, sc.pressResetAt);

function envUpdate() {
  const now = clock.nanos, dt = (now - lastNs) / 1e9; lastNs = now;
  for (let i = events.length - 1; i >= 0; i--) {
    if (events[i].ns <= now) { const e = events[i]; events.splice(i, 1); pins[e.pin].setInputValue(e.value); }
  }
  if (sc.imuOffFromS >= 0) busUpdate();               // plug/unplug changes the lines
  if (dt <= 0) return;
  const lifted = (sc.liftedFromS >= 0 && now / 1e9 >= sc.liftedFromS) ||
                 (sc.overturnFromS >= 0 && now / 1e9 >= sc.overturnFromS);
  const [la, lb] = pwmDuty(4), [ra, rb] = pwmDuty(5);
  const tS = now / 1e9;
  const byHand = sc.handTurnFromS >= 0 && tS >= sc.handTurnFromS && tS < sc.handTurnToS;
  for (const side of ['L', 'R']) {
    const t = side === 'L' ? motorTarget(la, lb) : motorTarget(ra, rb);
    const v = side === 'L' ? robot.vL : robot.vR;
    const target = t === null ? v * 0.98 : (side === 'L' ? t * sc.motorScaleL : t);
    let nv = v + (target - v) * Math.min(1, dt / TAU);
    if (byHand) nv = sc.handTurnRevs * WHEEL_CIRC / (sc.handTurnToS - sc.handTurnFromS);
    else if (sc.pushFromS >= 0 && tS >= sc.pushFromS && tS < sc.pushToS) {
      if (!robot.pushed) { robot.pushed = true; robot.x = sc.startX; robot.y = 9 * M; robot.th = 0; }
      nv = sc.pushMmS;
    }
    else if (sc.handTurnFromS >= 0 && tS >= sc.handTurnToS && tS < sc.handTurnToS + 0.05) nv = 0;
    if (side === 'L') robot.vL = nv; else robot.vR = nv;
  }
  const v = (robot.vL + robot.vR) / 2, w = (robot.vR - robot.vL) / TRACK;
  // accelerometer sees the car's own acceleration (mm/s^2), lightly filtered
  const acc = (v - (robot.vPrev ?? 0)) / dt;
  robot.vPrev = v;
  robot.accFwd += (acc - robot.accFwd) * Math.min(1, dt / 0.005);
  if (!lifted) { robot.x += v * Math.cos(robot.th) * dt; robot.y += v * Math.sin(robot.th) * dt; robot.th += w * dt; }
  if (sc.traceMs > 0 && now - (robot.traceNs ?? 0) >= sc.traceMs * 1e6) {
    robot.traceNs = now;
    console.error(`T ${tS.toFixed(2)} ${robot.x.toFixed(1)} ${robot.y.toFixed(2)} ${(robot.th * 180 / Math.PI).toFixed(2)} ${robot.vL.toFixed(1)} ${robot.vR.toFixed(1)}`);
  }
  if (sc.sweepFromS >= 0 && tS >= sc.sweepFromS && tS < sc.sweepToS) {
    robot.y = M * 12 + 45 * Math.sin(2 * Math.PI * (tS - sc.sweepFromS) / 2);
    robot.th = 0;
  }
  if (sc.spinFromS >= 0 && tS >= sc.spinFromS && tS < sc.spinToS) {
    robot.th += 2 * Math.PI * sc.spinRevs / (sc.spinToS - sc.spinFromS) * dt;
  }
  // encoders: one pulse train per wheel on the chosen pin
  for (const side of ['L', 'R']) {
    const vs = side === 'L' ? robot.vL : robot.vR;
    const key = side === 'L' ? 'edgeAccL' : 'edgeAccR';
    robot[key] += Math.abs(vs) * dt / WHEEL_CIRC * sc.edgesPerRev;
    while (robot[key] >= 1) {
      robot[key] -= 1;
      if (side === 'L' && sc.leftOkFromS >= 0 && tS < sc.leftOkFromS) continue;   // dead left encoder
      const pin = side === 'L' ? sc.leftPin : sc.rightPin;
      const lk = side === 'L' ? 'levelL' : 'levelR';
      robot[lk] = !robot[lk];
      pins[pin].setInputValue(robot[lk]);
    }
  }
  if (sc.layout === 'arm') {
    // GP26 just right of the line, GP27 on its left edge, GP28 the arm.
    mcu.adc.channelValues[0] = irValue(-sc.rightMm * M);
    mcu.adc.channelValues[1] = irValue(0);
    mcu.adc.channelValues[2] = irValue(sc.armMm * M);
  } else {
    mcu.adc.channelValues[0] = irValue(20 * M);
    mcu.adc.channelValues[1] = irValue(0);
    mcu.adc.channelValues[2] = irValue(-45 * M);
  }
}

// ---------------------------------------------------------------- run
mcu.core.PC = 0x10000000;
const ENV_NS = 20000;          // 20 us environment step
let nextEnv = 0;
const endNs = seconds * 1e9;
const CYCLE_NS = 8;
const started = Date.now();
const prof = new Map();
let nextProf = 0;
let idleNs = 0;
while (clock.nanos < endNs) {
  if (clock.nanos >= nextEnv) { envUpdate(); servoWatch(); nextEnv = clock.nanos + ENV_NS; }
  if (sc.profile && clock.nanos >= nextProf) {
    const pc = mcu.core.waiting ? 0 : (mcu.core.PC >>> 0);
    prof.set(pc, (prof.get(pc) ?? 0) + 1);
    nextProf = clock.nanos + 10000;
  }
  if (mcu.core.waiting) {
    const toAlarm = clock.nanosToNextAlarm;
    const step = Math.max(CYCLE_NS, Math.min(toAlarm > 0 ? toAlarm : CYCLE_NS, nextEnv - clock.nanos));
    clock.tick(step);
    // SysTick only sets pendingSystick: wake from WFI here (rp2040js does it
    // only inside executeInstruction, which is not called while waiting).
    if (mcu.core.interruptsUpdated && mcu.core.checkForInterrupts()) mcu.core.waiting = false;
  } else {
    const cycles = mcu.core.executeInstruction();
    clock.tick(cycles * CYCLE_NS);
  }
}
if (line) lines.push(`[${t0()}] ${line}`);
if (sc.profile) fs.writeFileSync('profile.json', JSON.stringify([...prof.entries()]));
fs.writeFileSync('last_run.log', lines.join('\n') + '\n');
console.error(`\n# simulated ${seconds} s in ${((Date.now() - started) / 1000).toFixed(1)} s; robot x=${robot.x.toFixed(0)} y=${robot.y.toFixed(0)} th=${(robot.th * 180 / Math.PI).toFixed(0)} deg; imu transfers ${imu.transfers}, bit-banged ACKs ${bus.acks}; servo pulse ${servoSeen.min}..${servoSeen.max} us, ${servoSeen.outside} steps outside ${sc.servoMinUs}..${sc.servoMaxUs}`);
