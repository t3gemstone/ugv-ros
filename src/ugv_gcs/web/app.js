'use strict';

// UGV host UI: talks to the robot over rosbridge (roslib.js).
// Everything the robot exposes is collected below by ROS topic/service name.

const API = {
  mode: ['/ugv/mode', 'std_msgs/msg/String'],
  estopActive: ['/ugv/estop_active', 'std_msgs/msg/Bool'],
  obstacle: ['/ugv/obstacle_distance', 'std_msgs/msg/Float32'],
  pose: ['/ugv/robot_pose', 'geometry_msgs/msg/PoseStamped'],
  odom: ['/wheel/odom', 'nav_msgs/msg/Odometry'],
  map: ['/map', 'nav_msgs/msg/OccupancyGrid'],
  plan: ['/plan', 'nav_msgs/msg/Path'],
  line: ['/line/state', 'ugv_interfaces/msg/LineState'],
  lineStatus: ['/line_follower/status', 'std_msgs/msg/String'],
  calibrationStatus: ['/line/calibration_status', 'std_msgs/msg/String'],
  calibrateWhite: ['/line/calibrate_white', 'std_srvs/srv/Trigger'],
  calibrateBlack: ['/line/calibrate_black', 'std_srvs/srv/Trigger'],
  rosout: ['/rosout', 'rcl_interfaces/msg/Log'],
  teleop: ['/cmd_vel/teleop', 'geometry_msgs/msg/Twist'],
  goal: ['/goal_pose', 'geometry_msgs/msg/PoseStamped'],
  setMode: ['/ugv/set_mode', 'ugv_interfaces/srv/SetMode'],
  estop: ['/ugv/estop', 'std_srvs/srv/SetBool'],
  saveMap: ['/ugv/save_map', 'ugv_interfaces/srv/SaveMap'],
  resetMap: ['/ugv/reset_map', 'std_srvs/srv/Trigger'],
  lidar: ['/ugv/lidar', 'ugv_interfaces/msg/LidarStatus'],
  setLidar: ['/ugv/set_lidar', 'ugv_interfaces/srv/SetLidar'],
};
const ROSBRIDGE_PORT = 9090;
const OUR_NODES = /mode_manager|map_manager|line_follower|ugv_base|ugv_imu|line_sensor|lidar_manager/;

const $ = (id) => document.getElementById(id);
let ros = null;
let connected = false;
const state = {
  mode: 'idle', estop: false, map: null, pose: null, plan: [], goal: null, obstacleAt: 0,
  lidar: null, lidarBusy: false,
};

// --- log -----------------------------------------------------------------
function log(text, level = 'info') {
  const el = $('log');
  const line = document.createElement('div');
  line.className = level;
  line.textContent = `[${new Date().toLocaleTimeString('en-GB', { hour12: false })}] ${text}`;
  el.appendChild(line);
  while (el.children.length > 300) el.removeChild(el.firstChild);
  el.scrollTop = el.scrollHeight;
}

// --- connection ----------------------------------------------------------
function storedHost() {
  try { return localStorage.getItem('ugv-host'); } catch (e) { return null; }
}

function connect() {
  const host = $('host').value.trim() || 'localhost';
  try { localStorage.setItem('ugv-host', host); } catch (e) { /* private window */ }
  const url = `ws://${host.includes(':') ? host : `${host}:${ROSBRIDGE_PORT}`}`;
  calibration.connection(false);
  heldKeys.clear();
  heldPads.clear();
  teleopTopic = null;
  if (ros) ros.close();
  ros = new ROSLIB.Ros({ url });
  const session = ros;
  ros.on('connection', () => {
    if (ros !== session) return;
    connected = true;
    calibration.connection(true);
    $('conn-state').className = 'dot on';
    log(`connected: ${host}`);
    subscribeAll();
  });
  ros.on('close', () => {
    if (ros !== session) return;
    if (connected) log('connection lost', 'warn');
    connected = false;
    calibration.connection(false);
    heldKeys.clear();
    heldPads.clear();
    teleopTopic = null;
    $('conn-state').className = 'dot off';
  });
  ros.on('error', () => log(`cannot reach rosbridge: ${url}`, 'error'));
}

function topic([name, type], options = {}) {
  return new ROSLIB.Topic({ ros, name, messageType: type, ...options });
}

function call([name, type], request, done, failed = () => {}) {
  const service = new ROSLIB.Service({ ros, name, serviceType: type });
  service.callService(new ROSLIB.ServiceRequest(request), done,
    (err) => { log(`${name}: ${err}`, 'error'); failed(); });
}

function subscribeAll() {
  topic(API.mode).subscribe((m) => {
    state.mode = m.data;
    calibration.modeChanged(m.data);
    renderMode();
  });
  topic(API.estopActive).subscribe((m) => { state.estop = m.data; renderMode(); });
  topic(API.obstacle).subscribe((m) => {
    state.obstacleAt = Date.now();
    $('obstacle').textContent = `${m.data.toFixed(2)} m`;
  });
  topic(API.pose, { throttle_rate: 100 }).subscribe((m) => {
    state.pose = m;
    const p = m.pose.position;
    $('pose').textContent = `${p.x.toFixed(2)}, ${p.y.toFixed(2)} (${m.header.frame_id})`;
    drawMap();
  });
  topic(API.odom, { throttle_rate: 200 }).subscribe((m) => {
    const t = m.twist.twist;
    $('speed').textContent = `${t.linear.x.toFixed(2)} m/s · ${t.angular.z.toFixed(2)} rad/s`;
  });
  // CBOR: the map arrives as binary without JSON conversion; converting it to
  // JSON kept rosbridge busy for seconds and delayed drive commands.
  topic(API.map, { throttle_rate: 2000, compression: 'cbor' }).subscribe((m) => { setMap(m); });
  topic(API.plan, { throttle_rate: 500 }).subscribe((m) => {
    state.plan = m.poses.map((p) => p.pose.position);
    drawMap();
  });
  topic(API.line, { throttle_rate: 100 }).subscribe(renderLine);
  topic(API.lineStatus).subscribe((m) => { $('line-status').textContent = m.data; });
  topic(API.calibrationStatus).subscribe((m) => {
    try { calibration.update(JSON.parse(m.data)); }
    catch (e) { log('Could not read the calibration status message', 'warn'); }
  });
  topic(API.lidar).subscribe((m) => { state.lidar = m; renderLidar(); });
  topic(API.rosout, { queue_length: 20 }).subscribe((m) => {
    if (m.level >= 30 || (m.level >= 20 && OUR_NODES.test(m.name))) {
      log(`${m.name}: ${m.msg}`, m.level >= 40 ? 'error' : m.level >= 30 ? 'warn' : 'info');
    }
  });
}

// --- mode / emergency stop -----------------------------------------------
function renderMode() {
  const badge = $('mode-badge');
  badge.textContent = state.estop ? 'EMERGENCY STOP' : state.mode.toUpperCase();
  badge.className = `badge mode-${state.estop ? 'estop' : state.mode}`;
  document.body.classList.toggle('estop-active', state.estop);
  $('estop').classList.toggle('hidden', state.estop);
  $('release').classList.toggle('hidden', !state.estop);
  document.querySelectorAll('[data-mode]').forEach((b) => {
    b.classList.toggle('active', b.dataset.mode === state.mode);
  });
}

function setMode(mode) {
  if (!connected || calibration.active) return;
  call(API.setMode, { mode }, (res) => log(`mode: ${res.message}`, res.success ? 'info' : 'warn'));
}

function setEstop(active) {
  if (!connected) return;
  if (active) publishTwist(0, 0);
  call(API.estop, { data: active }, () => {});
}

// --- lidar ---------------------------------------------------------------
const LIDAR_NAMES = { usb: 'USB', gpio: 'GPIO (UART)' };

function renderLidar() {
  const s = state.lidar;
  document.querySelectorAll('[data-lidar]').forEach((b) => {
    b.classList.toggle('active', !!s && b.dataset.lidar === s.source);
    b.disabled = state.lidarBusy;
  });
  const el = $('lidar-status');
  if (!s) { el.textContent = '—'; el.className = 'muted'; return; }
  const ok = s.running && s.rate_hz > 1;
  el.textContent = s.running
    ? `${s.port} · ${s.rate_hz.toFixed(1)} Hz${ok ? '' : ' (waiting for data)'}`
    : `not running: ${s.message}`;
  el.className = `muted ${ok ? 'ok' : 'warn'}`;
}

function setLidar(source) {
  if (!connected || state.lidarBusy) return;
  if (state.lidar && state.lidar.source === source && state.lidar.running) return;
  state.lidarBusy = true;
  renderLidar();
  log(`lidar: selecting ${LIDAR_NAMES[source]}...`);
  const done = () => { state.lidarBusy = false; renderLidar(); };
  call(API.setLidar, { source }, (res) => {
    done();
    log(res.message, res.success ? 'info' : 'warn');
  }, done);
}

// --- driving -------------------------------------------------------------
const KEYS = {
  w: [1, 0], arrowup: [1, 0], s: [-1, 0], arrowdown: [-1, 0],
  a: [0, 1], arrowleft: [0, 1], d: [0, -1], arrowright: [0, -1],
};
const heldKeys = new Set();
const heldPads = new Set();
let teleopTopic = null;
let wasMoving = false;
let calibrationDriveBlocked = false;

function publishTwist(v, w) {
  if (!connected) return;
  if (!teleopTopic) teleopTopic = topic(API.teleop);
  teleopTopic.publish(new ROSLIB.Message({
    linear: { x: v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: w },
  }));
}

function gamepadInput() {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  for (const pad of pads) {
    if (!pad) continue;
    const dead = (x) => (Math.abs(x) < 0.15 ? 0 : x);
    const v = -dead(pad.axes[1] || 0);
    const w = -dead(pad.axes[0] || 0);
    if (v || w) return [v, w];
  }
  return [0, 0];
}

// Key/stick direction -> angular velocity. Going forward, left = turn left (+w).
// Reversing works like a car's steering: the left key swings the rear of the
// vehicle to the left, which means the robot turns clockwise (-w). This mapping
// exists ONLY in this UI's manual driving; the command sent to /cmd_vel/teleop
// and all autonomous modes (line, Nav2) follow the standard convention
// (REP-103: +w = counter-clockwise).
function steering(v, w) {
  return v < 0 ? -w : w;
}

function teleopTick() {
  if (calibration.active) return;
  let v = 0;
  let w = 0;
  for (const vec of [...[...heldKeys].map((k) => KEYS[k]), ...heldPads]) {
    v += vec[0];
    w += vec[1];
  }
  const [gv, gw] = gamepadInput();
  v = Math.max(-1, Math.min(1, v + gv));
  w = Math.max(-1, Math.min(1, w + gw));
  const moving = v !== 0 || w !== 0;
  if (calibrationDriveBlocked) {
    if (!moving) calibrationDriveBlocked = false;
    return;
  }
  if (state.estop) return;
  if (moving) {
    publishTwist(v * parseFloat($('v-max').value), steering(v, w) * parseFloat($('w-max').value));
  } else if (wasMoving) {
    publishTwist(0, 0);
  }
  wasMoving = moving;
}

function typing(e) {
  return e.target instanceof HTMLInputElement && e.target.type !== 'range' && e.target.type !== 'checkbox';
}

window.addEventListener('keydown', (e) => {
  if (typing(e)) return;
  if (e.code === 'Space') { e.preventDefault(); setEstop(true); return; }
  if (calibration.active) return;
  const key = e.key.toLowerCase();
  if (KEYS[key]) { e.preventDefault(); heldKeys.add(key); }
});
window.addEventListener('keyup', (e) => heldKeys.delete(e.key.toLowerCase()));
window.addEventListener('blur', () => heldKeys.clear());

document.querySelectorAll('.key').forEach((btn) => {
  const vec = [parseFloat(btn.dataset.v), parseFloat(btn.dataset.w)];
  const release = () => { heldPads.delete(vec); btn.classList.remove('pressed'); };
  btn.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    if (calibration.active) return;
    btn.setPointerCapture(e.pointerId);
    heldPads.add(vec);
    btn.classList.add('pressed');
  });
  btn.addEventListener('pointerup', release);
  btn.addEventListener('pointercancel', release);
});

// --- line sensor ---------------------------------------------------------
const calibration = new LineCalibration(calibrationRequest, renderCalibration);

function calibrationRequest(action) {
  const session = ros;
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error('The operation timed out. Check the sensor status.')), 8000);
    const finish = (result, error) => {
      clearTimeout(timer);
      if (!connected || ros !== session) { reject(new Error('The robot connection changed.')); return; }
      if (error) reject(new Error('Could not reach the calibration service.'));
      else resolve(result);
    };
    const endpoint = action === 'idle' ? API.setMode
      : action === 'white' ? API.calibrateWhite : API.calibrateBlack;
    call(endpoint, action === 'idle' ? { mode: 'idle' } : {},
      (result) => finish(result), () => finish(null, true));
  });
}

function renderCalibration() {
  const c = calibration;
  const available = c.available();
  $('calibration-start').disabled = !available || c.active;
  $('calibration-start').classList.toggle('hidden', c.active);
  $('calibration-cancel').classList.toggle('hidden', !c.active);
  $('calibration-capture').classList.toggle('hidden', !c.active || c.step === 'starting');
  $('calibration-capture').disabled = !available || c.busy || !c.status?.ready;
  $('calibration-capture').textContent = c.busy ? 'Measuring…'
    : c.step === 'black' ? 'Black ready — measure and save' : 'White ready — measure';
  $('calibration-message').textContent = c.message;
  $('calibration-saved').textContent = !available
    ? 'No calibration data. Check the connection; this feature requires the Deneyap serial sensor driver.'
    : c.status.calibrated ? 'Saved calibration found · loaded automatically at startup.'
      : 'No saved calibration · calibrate before line following.';
  document.querySelectorAll('[data-mode], .key').forEach((b) => { b.disabled = c.active; });
  const values = c.status;
  const hasTable = available && ['white_mv', 'black_mv', 'threshold_mv']
    .every((key) => Array.isArray(values[key]) && values[key].length === 8);
  $('calibration-table').classList.toggle('hidden', !hasTable);
  if (hasTable) {
    $('calibration-values').replaceChildren(...values.white_mv.map((white, i) => {
      const row = document.createElement('tr');
      const difference = Math.abs(white - values.black_mv[i]);
      if (difference < 200) row.className = 'warn';
      for (const value of [`D${i + 1}`, white, values.black_mv[i], values.threshold_mv[i], difference]) {
        const cell = document.createElement('td');
        cell.textContent = typeof value === 'number' ? value.toFixed(0) : value;
        row.appendChild(cell);
      }
      row.title = difference < 200 ? 'Narrow separation margin; keep the height and ambient light constant.' : '';
      return row;
    }));
  }
}

function renderLine(msg) {
  const box = $('line-sensors');
  if (box.children.length !== msg.sensors.length) {
    box.replaceChildren(...msg.sensors.map(() => document.createElement('span')));
  }
  msg.sensors.forEach((on, i) => {
    box.children[i].classList.toggle('on', on);
    box.children[i].title = `raw: ${msg.raw[i]}`;
  });
  $('line-raw').textContent = msg.raw.length ? `raw: ${msg.raw.join(' ')}` : '';
}

// --- map -----------------------------------------------------------------
const canvas = $('map');
const ctx = canvas.getContext('2d');
const mapImage = document.createElement('canvas');
let view = null;   // { scale, ox, oy } — world <-> pixel transform

function setMap(msg) {
  const { width: w, height: h, resolution, origin } = msg.info;
  mapImage.width = w;
  mapImage.height = h;
  const mctx = mapImage.getContext('2d');
  const img = mctx.createImageData(w, h);
  const css = getComputedStyle(document.documentElement);
  const rgb = (name) => css.getPropertyValue(name).trim().match(/\w\w/g).map((x) => parseInt(x, 16));
  const free = rgb('--free');
  const occupied = rgb('--occupied');
  const unknown = rgb('--unknown');
  for (let j = 0; j < h; j++) {
    for (let i = 0; i < w; i++) {
      const v = msg.data[j * w + i];
      const c = v < 0 ? unknown : v >= 65 ? occupied : free;
      const k = ((h - 1 - j) * w + i) * 4;   // OccupancyGrid row 0 = bottom
      img.data[k] = c[0]; img.data[k + 1] = c[1]; img.data[k + 2] = c[2]; img.data[k + 3] = 255;
    }
  }
  mctx.putImageData(img, 0, 0);
  state.map = { w, h, resolution, origin: origin.position };
  $('map-info').textContent = `${w}×${h} cells · ${resolution.toFixed(2)} m/cell`;
  drawMap();
}

function toCanvas(x, y) {
  const m = state.map;
  return [view.ox + (x - m.origin.x) / m.resolution * view.scale,
    view.oy + (m.h - (y - m.origin.y) / m.resolution) * view.scale];
}

function toWorld(cx, cy) {
  const m = state.map;
  return [m.origin.x + (cx - view.ox) / view.scale * m.resolution,
    m.origin.y + (m.h - (cy - view.oy) / view.scale) * m.resolution];
}

function yawOf(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = canvas.clientWidth * dpr;
  canvas.height = canvas.clientHeight * dpr;
  drawMap();
}

function drawMap() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const m = state.map;
  if (!m) return;
  const scale = Math.min(canvas.width / m.w, canvas.height / m.h);
  view = { scale, ox: (canvas.width - m.w * scale) / 2, oy: (canvas.height - m.h * scale) / 2 };
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapImage, view.ox, view.oy, m.w * scale, m.h * scale);
  const px = (meters) => meters / m.resolution * scale;

  if (state.plan.length > 1) {
    ctx.strokeStyle = '#f0a830';
    ctx.lineWidth = 2;
    ctx.beginPath();
    state.plan.forEach((p, i) => {
      const [x, y] = toCanvas(p.x, p.y);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }
  if (state.goal) {
    const [x, y] = toCanvas(state.goal[0], state.goal[1]);
    ctx.strokeStyle = '#f0a830';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(x, y, Math.max(6, px(0.08)), 0, 2 * Math.PI);
    ctx.stroke();
  }
  if (state.pose && state.pose.header.frame_id === 'map') {
    const p = state.pose.pose.position;
    const [x, y] = toCanvas(p.x, p.y);
    const r = Math.max(6, px(0.12));
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-yawOf(state.pose.pose.orientation));
    ctx.fillStyle = '#4c8dff';
    ctx.beginPath();
    ctx.moveTo(r * 1.4, 0);
    ctx.lineTo(-r, r);
    ctx.lineTo(-r * 0.5, 0);
    ctx.lineTo(-r, -r);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
}

canvas.addEventListener('click', (e) => {
  if (calibration.active) return;
  if (!$('goal-mode').checked || !state.map || !view || !connected) return;
  if (state.estop) { log('cannot set a goal while the emergency stop is active', 'warn'); return; }
  const rect = canvas.getBoundingClientRect();
  const dpr = canvas.width / rect.width;
  const [x, y] = toWorld((e.clientX - rect.left) * dpr, (e.clientY - rect.top) * dpr);
  let yaw = 0;
  if (state.pose && state.pose.header.frame_id === 'map') {
    const p = state.pose.pose.position;
    yaw = Math.atan2(y - p.y, x - p.x);
  }
  // Switch the mode to nav explicitly first, so that mode_manager picking up
  // /goal_pose does not hinge on a single message.
  call(API.setMode, { mode: 'nav' }, (res) => {
    if (!res.success) { log(`could not set goal: ${res.message}`, 'warn'); return; }
    state.goal = [x, y];
    topic(API.goal).publish(new ROSLIB.Message({
      header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
      pose: {
        position: { x, y, z: 0 },
        orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
      },
    }));
    log(`goal: ${x.toFixed(2)}, ${y.toFixed(2)}`);
    drawMap();
  });
});

// --- events --------------------------------------------------------------
$('calibration-start').addEventListener('click', () => {
  if (!calibration.available() || calibration.active) return;
  calibrationDriveBlocked = true;
  heldKeys.clear();
  heldPads.clear();
  wasMoving = false;
  publishTwist(0, 0);
  calibration.start();
});
$('calibration-capture').addEventListener('click', () => calibration.capture());
$('calibration-cancel').addEventListener('click', () => {
  heldKeys.clear();
  heldPads.clear();
  calibration.cancel();
});
$('host').value = storedHost() || window.location.hostname || 'localhost';
$('connect').addEventListener('click', () => { $('host').blur(); connect(); });
// In a text box W/A/S/D go into the text; Enter/Esc drop focus and return to driving.
document.querySelectorAll('input:not([type=range]):not([type=checkbox])').forEach((el) => {
  el.addEventListener('keydown', (e) => {
    if (e.key !== 'Enter' && e.key !== 'Escape') return;
    el.blur();
    if (e.key === 'Enter' && el.id === 'host') connect();
  });
});
$('estop').addEventListener('click', () => setEstop(true));
$('release').addEventListener('click', () => setEstop(false));
document.querySelectorAll('[data-mode]').forEach((b) => {
  b.addEventListener('click', () => setMode(b.dataset.mode));
});
document.querySelectorAll('[data-lidar]').forEach((b) => {
  b.addEventListener('click', () => setLidar(b.dataset.lidar));
});
$('goal-mode').addEventListener('change', (e) => canvas.classList.toggle('goal', e.target.checked));
// Two-step confirmation (no browser dialog): the first click turns the button
// into "Are you sure?", a second click within 3 s resets.
let resetArmed = null;
$('reset-map').addEventListener('click', () => {
  const btn = $('reset-map');
  if (!connected) return;
  if (!resetArmed) {
    btn.textContent = 'Are you sure? Click again';
    btn.classList.add('armed');
    resetArmed = setTimeout(() => {
      resetArmed = null;
      btn.textContent = 'Reset map';
      btn.classList.remove('armed');
    }, 3000);
    return;
  }
  clearTimeout(resetArmed);
  resetArmed = null;
  btn.textContent = 'Reset map';
  btn.classList.remove('armed');
  log('resetting map...');
  call(API.resetMap, {}, (res) => {
    log(res.message, res.success ? 'info' : 'error');
    if (!res.success) return;
    state.map = null;
    state.plan = [];
    state.goal = null;
    $('map-info').textContent = 'Waiting for map (move the robot a little)';
    drawMap();
  });
});

$('save-map').addEventListener('click', () => {
  if (!connected) return;
  log('saving map...');
  call(API.saveMap, { name: $('map-name').value.trim() },
    (res) => log(res.message, res.success ? 'info' : 'error'));
});
for (const id of ['v-max', 'w-max']) {
  const update = () => {
    $(`${id}-label`).textContent = `${$(id).value} ${id === 'v-max' ? 'm/s' : 'rad/s'}`;
  };
  $(id).addEventListener('input', update);
  update();
}
window.addEventListener('resize', resizeCanvas);
setInterval(teleopTick, 100);
setInterval(() => {
  renderCalibration();
  if (Date.now() - state.obstacleAt > 1000) $('obstacle').textContent = '—';
}, 500);

resizeCanvas();
renderMode();
connect();
