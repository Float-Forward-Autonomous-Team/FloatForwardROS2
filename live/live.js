// DTU Float Forward public live page.
// Energy boat: replays the Monaco 2026 GPS data (Database2026/race_data) as if live, until the read-only feed exists.
// USV-01 (AI boat): live from foxglove_bridge (simulator now, the Jetson later).
// Only position, speed, heading and camera data are shown; nothing here can command a boat.

import { connectBridge } from "./bridge.js";

const BRIDGE_URL = new URLSearchParams(location.search).get("ws") ?? "ws://localhost:8765";
const ENERGY_DATA = "data/energy_monaco_2026.json";
const DEFAULT_RACE = "speedrecord";

const KNOTS_PER_MS = 1.94384;
const WINDOW_S = 600; // short track and the "last 10 minutes" figures
const SIGNAL_LOST_S = 15; // the 2026 link delivered data in bursts with gaps of about 8 s
const GAP_SKIP_S = 60; // replay jumps over longer outages instead of waiting them out
const GAP_PAUSE_MS = 3000; // ...after showing "no signal" for this long
const MOVING_MS = 1.0; // below this the boat counts as stopped for the average speed
const STRIP_BINS = 600;
const CHART_BINS = 240;
const AI_LIVE_S = 3;
const AI_CHART_BIN_S = 5;
const OBJECT_TIMEOUT_MS = 1000;
const TRACK_STEP_M = 0.5;
const CAMERA_FOV_DEG = 80;
const CAMERA_RANGE_M = 30;
const BOAT_LENGTH_M = 4.9; // WAM-V; drawn at least 26 px long
const ENERGY_SCALE = 1.2; // px per metre: about 300 m across a phone
const AI_SCALE = 4;
const MAP_FRAME_MS = 33;
const TEXT_FRAME_MS = 150;
const CHART_FRAME_MS = 1000;
const TILE_URL = (z, x, y) => `https://tile.openstreetmap.org/${z}/${x}/${y}.png`;
const TILE_CACHE = 300;
const MAX_TILES = 64; // more than this on screen and the grid stands in for the map

const AI_TOPICS = {
  gps: "/wamv/sensors/gps/gps/fix",
  track: "/gps_track",
  imu: "/wamv/sensors/imu/imu/data",
  obstacles: "/obstacles/markers",
  detections: "/detections", // published by the future detection node
  inference: "/detections/inference_ms",
};
const cameraTopic = (name) => `/wamv/sensors/cameras/${name}_camera_sensor/optical/image_raw`;

const $ = (id) => document.getElementById(id);
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

// ---------- formatting ----------

const compassName = (deg) => ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][Math.round(deg / 45) % 8];
const kn = (ms) => (ms * KNOTS_PER_MS).toFixed(1);
const knots = (ms) => `${kn(ms)}<small>kn</small>`;
const headingText = (deg) => `${String(Math.round(deg) % 360).padStart(3, "0")}°<small>${compassName(deg)}</small>`;
const distanceText = (m) => (m < 1000 ? `${m.toFixed(0)}<small>m</small>` : `${(m / 1000).toFixed(2)}<small>km</small>`);
const plain = (html) => html.replace(/<small>/g, " ").replace(/<\/small>/g, "");
const pct = (x) => `${Math.round(x * 100)}<small>%</small>`;
const quiet = (text) => `<span class="quiet">${text}</span>`;
const DASH = quiet("—"); // a missing value: a small grey dash, never one at full figure size

function duration(s) {
  s = Math.max(0, Math.round(s));
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const sec = String(s % 60).padStart(2, "0");
  return h ? `${h}:${String(m).padStart(2, "0")}:${sec}` : `${m}:${sec}`;
}

function age(s) {
  if (s < 10) return `${s.toFixed(1)} s`;
  if (s < 60) return `${Math.round(s)} s`;
  if (s < 3600) return `${Math.floor(s / 60)} min ${Math.round(s % 60)} s`;
  return `${Math.floor(s / 3600)} h ${Math.floor((s % 3600) / 60)} min`;
}

const monaco = (unix, opts) => new Date(unix * 1000).toLocaleString("en-GB", { timeZone: "Europe/Monaco", ...opts });
const clock = (unix) => monaco(unix, { hour: "2-digit", minute: "2-digit" });
const clockS = (unix) => monaco(unix, { hour: "2-digit", minute: "2-digit", second: "2-digit" });
const longDate = (unix) => monaco(unix, { day: "numeric", month: "long", year: "numeric" });
const isoDate = (unix) => new Date(unix * 1000).toLocaleDateString("sv-SE", { timeZone: "Europe/Monaco" });

// Where an object is, seen from the boat (x forward, y to port), in sailor's words.
function bearingWords(x, y) {
  const a = (Math.atan2(y, x) * 180) / Math.PI;
  if (Math.abs(a) <= 30) return "ahead";
  if (Math.abs(a) >= 150) return "astern";
  return a > 0 ? "to port" : "to starboard";
}

function objectName(label) {
  const base = (label ?? "object").split(/\s+/)[0].replace(/_\d+$/, "");
  return { red: "Red buoy", green: "Green buoy", black: "Black buoy", white: "White buoy" }[base] ?? base.replace(/_/g, " ");
}

// Flat-earth metres around a reference point: fine over a race course.
function projection(lat0, lon0) {
  const kx = 111320 * Math.cos((lat0 * Math.PI) / 180);
  return (lat, lon) => [(lon - lon0) * kx, (lat - lat0) * 110540];
}

// ---------- page state ----------

const app = { boat: "energy", view: "map", layers: { chart: true, recent: true, past: true, objects: true, fov: true } };
const map = { cx: 0, cy: 0, scale: ENERGY_SCALE, follow: true, lastDraw: 0 };
let lastText = 0;
let lastChart = 0;
let lastAiChart = 0;
let chartKey = "";

// ---------- Energy boat: replay of Monaco 2026 ----------

const energy = { races: [], race: null, simT: 0, playing: true, rate: 10, gapSince: null, lastReal: null };

// Everything the page shows about a run is computed here, once, from its fixes.
function prepareRace(race) {
  const fixes = race.fixes;
  race.geo = { lat0: fixes[0][1], lon0: fixes[0][2] }; // the origin of the run's metres, for the harbour map
  const toXY = projection(race.geo.lat0, race.geo.lon0);
  let speed = null;
  let dist = 0;
  let heard = 0;
  let top = 0;
  let movSum = 0;
  let movCount = 0;
  Object.assign(race, { xy: [], cumDist: [], cumHeard: [], topSoFar: [], avgSoFar: [] });
  fixes.forEach((f, i) => {
    f[3] = speed = f[3] ?? speed; // carry the last known speed over fixes without one
    race.xy.push(toXY(f[1], f[2]));
    if (i) {
      const dt = f[0] - fixes[i - 1][0];
      if (dt <= GAP_SKIP_S) dist += Math.hypot(race.xy[i][0] - race.xy[i - 1][0], race.xy[i][1] - race.xy[i - 1][1]);
      if (dt <= SIGNAL_LOST_S) heard += dt;
    }
    if (speed != null) {
      top = Math.max(top, speed);
      if (speed > MOVING_MS) {
        movSum += speed;
        movCount++;
      }
    }
    race.cumDist.push(dist);
    race.cumHeard.push(heard);
    race.topSoFar.push(top);
    race.avgSoFar.push(movCount ? movSum / movCount : null);
  });
  const end = fixes.at(-1)[0];
  race.end = end;
  // replays open a few seconds before the boat first moves, not on the slipway
  const firstMove = fixes.find((f) => f[3] != null && f[3] > MOVING_MS);
  race.openAt = firstMove ? Math.max(0, firstMove[0] - 5) : 0;
  race.totals = { distance: dist, top, avg: movCount ? movSum / movCount : null, heard: end ? heard / end : 1 };

  // the signal strip: a bin is "heard" if a fix landed in it or it sits inside a gap shorter than SIGNAL_LOST_S
  race.heardBins = new Uint8Array(STRIP_BINS);
  const bin = (t) => Math.min(STRIP_BINS - 1, Math.floor((t / end) * STRIP_BINS));
  fixes.forEach((f, i) => {
    race.heardBins[bin(f[0])] = 1;
    if (i && f[0] - fixes[i - 1][0] <= SIGNAL_LOST_S) for (let b = bin(fixes[i - 1][0]); b <= bin(f[0]); b++) race.heardBins[b] = 1;
  });
}

// Index of the last fix at or before t (binary search).
function fixIndexAt(fixes, t) {
  if (t <= fixes[0][0]) return 0;
  let lo = 0;
  let hi = fixes.length - 1;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (fixes[mid][0] <= t) lo = mid;
    else hi = mid - 1;
  }
  return lo;
}

function selectRace(id) {
  const race = energy.races.find((r) => r.id === id) ?? energy.races[0];
  energy.race = race;
  energy.simT = race.openAt;
  energy.gapSince = null;
  energy.playing = true;
  $("e-race").value = race.id;
  $("strip").setAttribute("aria-valuemax", String(Math.round(race.end)));
  map.scale = ENERGY_SCALE;
  setFollow(true);
  chartKey = "";
  renderRaceFacts();
  renderRaceTable();
}

function seek(t) {
  if (!energy.race) return;
  energy.simT = Math.min(energy.race.end, Math.max(0, t));
  energy.gapSince = null;
}

function tickEnergy(now) {
  const race = energy.race;
  if (!race) return;
  if (energy.playing && energy.lastReal !== null) {
    energy.simT += ((now - energy.lastReal) / 1000) * energy.rate;
    const fixes = race.fixes;
    const i = fixIndexAt(fixes, energy.simT);
    const next = fixes[i + 1];
    const inLongGap = next && energy.simT - fixes[i][0] > SIGNAL_LOST_S && next[0] - fixes[i][0] > GAP_SKIP_S;
    if (!inLongGap) energy.gapSince = null;
    else if (energy.gapSince === null) energy.gapSince = now;
    else if (now - energy.gapSince > GAP_PAUSE_MS) {
      energy.simT = next[0] - 2;
      energy.gapSince = null;
    }
    if (energy.simT >= race.end) {
      energy.simT = race.end;
      energy.playing = false;
    }
  }
  energy.lastReal = now;
}

// What the shared map and readouts need from the Energy boat right now.
function energyState() {
  const race = energy.race;
  if (!race) return null;
  const fixes = race.fixes;
  const i = fixIndexAt(fixes, energy.simT);
  const start = fixIndexAt(fixes, energy.simT - WINDOW_S);
  const [x, y] = race.xy[i];
  return {
    pos: { x, y },
    heading: fixes[i][4],
    speed: fixes[i][3],
    past: { pts: race.xy, start: 0, end: i },
    recent: { pts: race.xy, start, end: i },
    objects: [],
    fov: false,
    geo: race.geo,
    distance: race.cumDist[i],
    silent: energy.simT - fixes[i][0],
    i,
    start,
  };
}

function renderRaceFacts() {
  const race = energy.race;
  const t0 = race.start;
  $("f-date").textContent = longDate(t0);
  $("f-window").textContent = `${clock(t0)}–${clock(t0 + race.end)}`;
  $("f-duration").textContent = duration(race.end);
  $("f-length").innerHTML = distanceText(race.totals.distance);
  $("f-top").innerHTML = knots(race.totals.top);
  $("signal-meta").textContent = `Heard ${Math.round(race.totals.heard * 100)}% of the recorded time`;
  $("strip-start").textContent = clock(t0);
  $("strip-end").textContent = clock(t0 + race.end);
}

function renderRaceTable() {
  $("races").innerHTML = energy.races
    .map((r) => {
      const t = r.totals;
      return `<tr data-race="${r.id}" aria-selected="${r === energy.race}" tabindex="0">
        <td class="num">${isoDate(r.start)}</td>
        <td class="name">${r.name}<span class="sub">${longDate(r.start)} · ${duration(r.end)} recorded</span></td>
        <td class="n num">${duration(r.end)}</td>
        <td class="n num">${plain(distanceText(t.distance))}</td>
        <td class="n num top">${kn(t.top)} kn</td>
        <td class="n num">${t.avg != null ? `${kn(t.avg)} kn` : "—"}</td>
        <td class="n num">${Math.round(t.heard * 100)}%</td>
      </tr>`;
    })
    .join("");
}

function pickRace(row) {
  selectRace(row.dataset.race);
  setView("map");
}
$("races").addEventListener("click", (e) => {
  const row = e.target.closest("tr[data-race]");
  if (row) pickRace(row);
});
$("races").addEventListener("keydown", (e) => {
  const row = e.target.closest("tr[data-race]");
  if (row && (e.key === "Enter" || e.key === " ")) {
    e.preventDefault();
    pickRace(row);
  }
});

function renderEnergyText(s) {
  const race = energy.race;
  const progress = energy.simT / race.end;
  const lead = `${race.name}, ${longDate(race.start)}.`;
  let line;
  if (!energy.playing && energy.simT >= race.end) line = `Run finished: ${plain(distanceText(s.distance))} in ${duration(race.end)}.`;
  else if (s.silent > SIGNAL_LOST_S) line = `No signal for ${age(s.silent)}${energy.gapSince !== null ? ", skipping ahead" : ""}.`;
  else if (s.speed != null && s.heading != null) line = `Heading ${compassName(s.heading)} at ${kn(s.speed)} kn, ${Math.round(progress * 100)}% of the run done.`;
  else line = `${Math.round(progress * 100)}% of the run done.`;
  $("map-title").textContent = "Monaco harbour";
  $("map-sentence").textContent = `${lead} ${line}`;

  $("r-sog").innerHTML = s.speed != null ? knots(s.speed) : DASH;
  $("r-hdg").innerHTML = s.heading != null ? headingText(s.heading) : DASH;
  $("r-dist").innerHTML = distanceText(s.distance);
  $("r-4").innerHTML = pct(progress);

  const now = clockS(race.start + energy.simT);
  $("strip-now").textContent = `${now} · ${duration(energy.simT)} in`;
  $("strip").setAttribute("aria-valuenow", String(Math.round(energy.simT)));
  $("strip").setAttribute("aria-valuetext", `${now} Monaco time`);
  $("e-play").textContent = energy.playing ? "Pause" : "Play";
  if (s.silent > SIGNAL_LOST_S) setStatus("warn", "No signal", `Replay · ${now} Monaco time`);
  else setStatus("neutral", "Replay", `${now} Monaco time`);

  // performance
  const span = Math.min(WINDOW_S, energy.simT) || 1;
  const heard10 = (race.cumHeard[s.i] - race.cumHeard[s.start]) / span;
  $("perf-sentence").textContent = `${race.name}, ${longDate(race.start)}. Top speed so far ${kn(race.topSoFar[s.i])} kn, of ${kn(race.totals.top)} kn on the whole run.`;
  $("p1").innerHTML = knots(race.topSoFar[s.i]);
  $("p2").innerHTML = race.avgSoFar[s.i] != null ? knots(race.avgSoFar[s.i]) : DASH;
  $("p3").innerHTML = pct(Math.min(1, heard10));
  $("p4").textContent = duration(energy.simT);
}

// Speed over the whole run, binned; bins after the replay head stay empty so the line grows.
function drawEnergyChart() {
  const race = energy.race;
  const binS = Math.max(10, Math.ceil(race.end / CHART_BINS));
  const n = Math.ceil(race.end / binS) || 1;
  const played = Math.floor(energy.simT / binS);
  const key = `${race.id}:${played}`;
  if (key === chartKey) return;
  chartKey = key;
  const sums = new Float64Array(n);
  const counts = new Uint32Array(n);
  for (const f of race.fixes) {
    if (f[0] > energy.simT || f[3] == null) continue;
    const b = Math.min(n - 1, Math.floor(f[0] / binS));
    sums[b] += f[3] * KNOTS_PER_MS;
    counts[b]++;
  }
  const values = Array.from({ length: n }, (_, b) => (counts[b] ? sums[b] / counts[b] : null));
  const labels = Array.from({ length: n }, (_, b) => clock(race.start + b * binS));
  let topAt = -1;
  values.forEach((v, b) => v != null && (topAt < 0 || v > values[topAt]) && (topAt = b));
  $("chart-title").textContent = "Speed over the run, knots";
  $("chart-meta").textContent = `${binS} s averages, Monaco time`;
  $("chart").innerHTML = "";
  Charts.line($("chart"), {
    labels,
    series: [{ name: "Speed", values }],
    format: Charts.fmt.dec(1),
    height: 240,
    annotations: topAt >= 0 ? [{ at: topAt, label: `Fastest ${values[topAt].toFixed(1)} kn at ${labels[topAt]}`, short: `${values[topAt].toFixed(1)} kn` }] : undefined,
  });
}

$("e-race").addEventListener("change", (e) => selectRace(e.target.value));
$("e-play").addEventListener("click", () => {
  if (!energy.race) return;
  if (!energy.playing && energy.simT >= energy.race.end) seek(0);
  energy.playing = !energy.playing;
});
for (const b of document.querySelectorAll("[data-rate]")) {
  b.addEventListener("click", () => {
    energy.rate = Number(b.dataset.rate);
    pressOnly("[data-rate]", b);
  });
}

fetch(ENERGY_DATA)
  .then((r) => r.json())
  .then((data) => {
    for (const race of data.races) {
      prepareRace(race);
      $("e-race").add(new Option(`${race.day} ${race.name}`, race.id));
    }
    energy.races = data.races;
    selectRace(DEFAULT_RACE);
  })
  .catch((err) => ($("map-sentence").textContent = `The race data did not load (${err.message}). Serve the page with make live.`));

// ---------- the signal strip: when the link delivered, and where the replay is ----------

const stripCanvas = $("strip-canvas");

function drawStrip() {
  const race = energy.race;
  const w = stripCanvas.clientWidth;
  const h = stripCanvas.clientHeight;
  if (!race || !w) return;
  const dpr = devicePixelRatio || 1;
  stripCanvas.width = Math.round(w * dpr);
  stripCanvas.height = Math.round(h * dpr);
  const ctx = stripCanvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const played = energy.simT / race.end;
  const bw = w / STRIP_BINS;
  const colours = { played: css("--accent"), ahead: css("--accent-track") };
  for (let b = 0; b < STRIP_BINS; b++) {
    if (!race.heardBins[b]) continue;
    ctx.fillStyle = (b + 1) / STRIP_BINS <= played ? colours.played : colours.ahead;
    ctx.fillRect(b * bw, 8, bw + 0.5, h - 16);
  }
  const x = Math.round(played * w);
  ctx.fillStyle = css("--text");
  ctx.fillRect(Math.min(w - 2, Math.max(0, x - 1)), 0, 2, h);
}

let stripDrag = false;
const stripSeek = (e) => {
  const r = stripCanvas.getBoundingClientRect();
  seek(((e.clientX - r.left) / r.width) * energy.race.end);
};
$("strip").addEventListener("pointerdown", (e) => {
  if (!energy.race) return;
  stripDrag = true;
  $("strip").setPointerCapture(e.pointerId);
  stripSeek(e);
});
$("strip").addEventListener("pointermove", (e) => stripDrag && stripSeek(e));
$("strip").addEventListener("pointerup", () => (stripDrag = false));
$("strip").addEventListener("pointercancel", () => (stripDrag = false));
$("strip").addEventListener("keydown", (e) => {
  if (!energy.race) return;
  const step = e.shiftKey ? 300 : 30;
  const to = { ArrowRight: energy.simT + step, ArrowUp: energy.simT + step, ArrowLeft: energy.simT - step, ArrowDown: energy.simT - step, Home: 0, End: energy.race.end }[e.key];
  if (to === undefined) return;
  e.preventDefault();
  seek(to);
});

// ---------- USV-01: live from foxglove_bridge ----------

const ai = {
  bridge: null,
  status: "disconnected",
  toXY: null,
  pos: null,
  heading: null,
  speed: null,
  lastFix: null, // { lat, lon, t } in simulator seconds, for the speed estimate
  lastFixReal: null,
  simT: 0,
  recent: [], // [x, y, sim seconds] since the page opened
  past: [], // [x, y] from /gps_track, the whole run
  pastDistance: 0,
  speeds: [], // [sim seconds, knots]
  top: 0,
  gpsTimes: [],
  objects: new Map(), // marker id -> { x, y, color, label, seen }
  passed: new Map(), // object name -> { color, closest }
  detections: null,
  inferenceMs: null,
  camera: "front_left",
  image: null,
};

function ensureProjection(lat, lon) {
  if (!ai.toXY) ai.toXY = projection(lat, lon);
}

function onGps(fix) {
  if (!Number.isFinite(fix.latitude) || !Number.isFinite(fix.longitude)) return;
  ensureProjection(fix.latitude, fix.longitude);
  const [x, y] = ai.toXY(fix.latitude, fix.longitude);
  const t = fix.header.stamp.sec + fix.header.stamp.nanosec * 1e-9;
  ai.pos = { x, y };
  ai.simT = t;
  ai.lastFixReal = Date.now();
  ai.gpsTimes.push(ai.lastFixReal);

  const last = ai.recent.at(-1);
  if (!last || Math.hypot(x - last[0], y - last[1]) >= TRACK_STEP_M) ai.recent.push([x, y, t]);
  while (ai.recent.length && ai.recent[0][2] < t - WINDOW_S) ai.recent.shift();

  // speed from successive fixes on the simulator's clock (it can run slower than real time)
  const prev = ai.lastFix;
  if (prev && t - prev.t < 1) return;
  if (prev) {
    const [px, py] = ai.toXY(prev.lat, prev.lon);
    ai.speed = Math.hypot(x - px, y - py) / (t - prev.t);
    ai.top = Math.max(ai.top, ai.speed);
    ai.speeds.push([t, ai.speed * KNOTS_PER_MS]);
    while (ai.speeds.length && ai.speeds[0][0] < t - WINDOW_S) ai.speeds.shift();
  }
  ai.lastFix = { lat: fix.latitude, lon: fix.longitude, t };
}

function onTrack(msg) {
  const coords = JSON.parse(msg.geojson).geometry.coordinates;
  if (!coords.length) return;
  ensureProjection(coords[0][1], coords[0][0]);
  ai.past = coords.map(([lon, lat]) => ai.toXY(lat, lon));
  ai.pastDistance = 0;
  for (let i = 1; i < ai.past.length; i++) ai.pastDistance += Math.hypot(ai.past[i][0] - ai.past[i - 1][0], ai.past[i][1] - ai.past[i - 1][1]);
}

function onImu(imu) {
  const q = imu.orientation;
  const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  ai.heading = (450 - (yaw * 180) / Math.PI) % 360; // ENU yaw -> compass heading
}

function onObstacles(msg) {
  const now = Date.now();
  for (const m of msg.markers) {
    const entry = ai.objects.get(m.id) ?? {};
    if (m.ns === "label") entry.label = m.text;
    else {
      entry.x = m.pose.position.x;
      entry.y = m.pose.position.y;
      entry.color = `rgb(${m.color.r * 255},${m.color.g * 255},${m.color.b * 255})`;
    }
    entry.seen = now;
    ai.objects.set(m.id, entry);
  }
}

const onDetections = (msg) => (ai.detections = msg);
const onInference = (m) => (ai.inferenceMs = m.data);

function onImage(img) {
  if (!ai.image || ai.image.width !== img.width || ai.image.height !== img.height) ai.image = new ImageData(img.width, img.height);
  const src = img.data;
  const dst = ai.image.data;
  const n = img.width * img.height;
  if (img.encoding === "rgb8" || img.encoding === "bgr8") {
    const [r, b] = img.encoding === "rgb8" ? [0, 2] : [2, 0];
    for (let i = 0, j = 0; i < n; i++, j += 3) {
      dst[i * 4] = src[j + r];
      dst[i * 4 + 1] = src[j + 1];
      dst[i * 4 + 2] = src[j + b];
      dst[i * 4 + 3] = 255;
    }
  } else if (img.encoding === "rgba8") {
    dst.set(src.subarray(0, n * 4));
  } else if (img.encoding === "mono8") {
    for (let i = 0; i < n; i++) {
      dst[i * 4] = dst[i * 4 + 1] = dst[i * 4 + 2] = src[i];
      dst[i * 4 + 3] = 255;
    }
  } else {
    return;
  }
  ai.image.dirty = true;
}

function drawCamera() {
  if (!ai.image?.dirty) return;
  ai.image.dirty = false;
  const cam = $("cam");
  const overlay = $("cam-overlay");
  if (cam.width !== ai.image.width) {
    cam.width = overlay.width = ai.image.width;
    cam.height = overlay.height = ai.image.height;
  }
  cam.getContext("2d").putImageData(ai.image, 0, 0);
  $("cam-empty").hidden = true;

  // boxes from the detection node (foxglove_msgs/ImageAnnotations), in image pixels
  const ctx = overlay.getContext("2d");
  ctx.clearRect(0, 0, overlay.width, overlay.height);
  const det = ai.detections;
  if (!det) return;
  ctx.lineWidth = 3;
  for (const shape of det.points) {
    const c = shape.outline_color;
    ctx.strokeStyle = `rgba(${c.r * 255},${c.g * 255},${c.b * 255},${c.a || 1})`;
    ctx.beginPath();
    shape.points.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
    ctx.closePath();
    ctx.stroke();
  }
  ctx.font = `600 22px ${css("--font")}`;
  ctx.fillStyle = "#fff";
  for (const text of det.texts) ctx.fillText(text.text, text.position.x, text.position.y);
}

// Objects in range, nearest first, with world positions for the map.
function currentObjects() {
  const now = Date.now();
  const h = ((ai.heading ?? 0) * Math.PI) / 180;
  const fwd = [Math.sin(h), Math.cos(h)];
  const port = [-Math.cos(h), Math.sin(h)];
  return [...ai.objects.values()]
    .filter((o) => now - o.seen < OBJECT_TIMEOUT_MS && o.x !== undefined)
    .map((o) => ({
      name: objectName(o.label),
      color: o.color,
      dist: Math.hypot(o.x, o.y),
      where: bearingWords(o.x, o.y),
      x: (ai.pos?.x ?? 0) + o.x * fwd[0] + o.y * port[0],
      y: (ai.pos?.y ?? 0) + o.x * fwd[1] + o.y * port[1],
    }))
    .sort((a, b) => a.dist - b.dist);
}

function aiState() {
  const objects = ai.pos ? currentObjects() : [];
  for (const o of objects) {
    const p = ai.passed.get(o.name);
    if (!p || o.dist < p.closest) ai.passed.set(o.name, { color: o.color, closest: o.dist });
  }
  return {
    pos: ai.pos,
    heading: ai.heading,
    speed: ai.speed,
    past: { pts: ai.past, start: 0, end: ai.past.length - 1 },
    recent: { pts: ai.recent, start: 0, end: ai.recent.length - 1 },
    objects,
    fov: true,
    distance: ai.past.length ? ai.pastDistance : 0,
    silent: ai.lastFixReal ? (Date.now() - ai.lastFixReal) / 1000 : Infinity,
  };
}

const objectRows = (objects, emptyText) =>
  objects.length
    ? objects.map((o) => `<li class="row"><span class="swatch" style="background:${o.color}"></span><span>${o.name}</span><span class="where">${o.where}</span><span class="val num">${o.dist.toFixed(1)} m</span></li>`).join("")
    : `<li class="empty-line">${emptyText}</li>`;

function renderAiText(s) {
  const nearest = s.objects[0];
  const connected = ai.status === "connected";
  $("map-title").textContent = "Regatta course, simulator";
  let sentence;
  if (!connected) sentence = ai.status === "connecting" ? "Connecting to USV-01…" : "USV-01 is not streaming right now. This page reconnects by itself.";
  else if (!s.pos) sentence = "Connected. Waiting for the boat's position…";
  else if (s.silent > AI_LIVE_S) sentence = `No signal from USV-01 for ${age(s.silent)}.`;
  else {
    const move = s.speed != null && s.heading != null ? `Heading ${compassName(s.heading)} at ${kn(s.speed)} kn.` : "On the water.";
    const near = nearest ? `Nearest object: ${nearest.name.toLowerCase()}, ${nearest.dist.toFixed(1)} m ${nearest.where}.` : "No objects within 100 m.";
    sentence = `${move} ${near}`;
  }
  $("map-sentence").textContent = sentence;

  $("r-sog").innerHTML = s.speed != null ? knots(s.speed) : DASH;
  $("r-hdg").innerHTML = s.heading != null ? headingText(s.heading) : DASH;
  $("r-dist").innerHTML = s.pos ? distanceText(s.distance) : DASH;
  $("r-4").innerHTML = nearest ? `${nearest.dist.toFixed(1)}<small>m</small>` : DASH;

  if (connected && s.silent <= AI_LIVE_S) setStatus("live", "Online", `updated ${age(s.silent)} ago`);
  else if (connected) setStatus("warn", s.pos ? "No signal" : "Connected", s.pos ? `last fix ${age(s.silent)} ago` : "");
  else if (ai.status === "connecting") setStatus("warn", "Connecting", "");
  else setStatus("neg", "Offline", "");

  const waiting = connected ? "Nothing within 100 m" : "Waiting for USV-01…";
  $("around").innerHTML = objectRows(s.objects.slice(0, 5), waiting);
  $("around-cam").innerHTML = objectRows(s.objects.slice(0, 5), waiting);
  const n = ai.detections?.points.length;
  $("c-det").innerHTML = n == null ? quiet("not running yet") : `${n}<small>in view</small>`;
  $("c-near").innerHTML = nearest ? `${nearest.dist.toFixed(1)}<small>m ${nearest.where}</small>` : DASH;

  // performance
  const now = Date.now();
  ai.gpsTimes = ai.gpsTimes.filter((t) => now - t < 5000);
  $("perf-sentence").textContent = ai.top ? `USV-01 in the simulator. Top speed ${kn(ai.top)} kn since this page opened.` : "USV-01 in the simulator. Figures start once the boat moves.";
  $("p1").innerHTML = ai.top ? knots(ai.top) : DASH;
  $("p2").innerHTML = ai.inferenceMs != null ? `${ai.inferenceMs.toFixed(1)}<small>ms</small>` : quiet("not running yet");
  $("p3").innerHTML = n != null ? String(n) : DASH;
  $("p4").innerHTML = `${(ai.gpsTimes.length / 5).toFixed(1)}<small>Hz</small>`;
  const passed = [...ai.passed.entries()].sort((a, b) => a[1].closest - b[1].closest).slice(0, 6);
  $("passed").innerHTML = passed.length
    ? passed.map(([name, p]) => `<li class="row"><span class="swatch" style="background:${p.color}"></span><span>${name}</span><span class="val num">${p.closest.toFixed(1)} m</span></li>`).join("")
    : `<li class="empty-line">${connected ? "No objects passed yet" : "Waiting for USV-01…"}</li>`;
}

// Speed over the last 10 minutes of simulator time, in 5 s bins.
function drawAiChart() {
  const end = Math.floor(ai.simT / AI_CHART_BIN_S);
  const n = WINDOW_S / AI_CHART_BIN_S;
  const sums = new Float64Array(n);
  const counts = new Uint32Array(n);
  for (const [t, k] of ai.speeds) {
    const b = n - 1 - (end - Math.floor(t / AI_CHART_BIN_S));
    if (b < 0 || b >= n) continue;
    sums[b] += k;
    counts[b]++;
  }
  const values = Array.from({ length: n }, (_, b) => (counts[b] ? sums[b] / counts[b] : null));
  const labels = Array.from({ length: n }, (_, b) => duration(Math.max(0, end - (n - 1 - b)) * AI_CHART_BIN_S));
  $("chart-title").textContent = "Speed, last 10 minutes, knots";
  $("chart-meta").textContent = "simulator time";
  $("chart").innerHTML = "";
  Charts.line($("chart"), { labels, series: [{ name: "Speed", values }], format: Charts.fmt.dec(1), height: 240 });
}

function aiHandlers() {
  const handlers = {
    [AI_TOPICS.gps]: onGps,
    [AI_TOPICS.track]: onTrack,
    [AI_TOPICS.imu]: onImu,
    [AI_TOPICS.obstacles]: onObstacles,
    [AI_TOPICS.detections]: onDetections,
    [AI_TOPICS.inference]: onInference,
  };
  // a raw frame is about 2.7 MB: stream it only while someone is looking at it
  if (app.boat === "ai" && app.view === "cameras") handlers[cameraTopic(ai.camera)] = onImage;
  return handlers;
}

for (const button of document.querySelectorAll("[data-camera]")) {
  button.addEventListener("click", () => {
    ai.camera = button.dataset.camera;
    ai.image = null;
    $("cam-empty").hidden = false;
    for (const id of ["cam", "cam-overlay"]) $(id).getContext("2d").clearRect(0, 0, $(id).width, $(id).height);
    pressOnly("[data-camera]", button);
    ai.bridge?.setHandlers(aiHandlers());
  });
}

// ---------- shared: status, switching ----------

function setStatus(kind, text, detail) {
  const pill = $("status");
  pill.className = `pill ${{ live: "pos live", neutral: "", warn: "warn", neg: "neg" }[kind]}`;
  pill.textContent = text;
  $("stamp").textContent = detail;
}

function pressOnly(selector, active) {
  for (const b of document.querySelectorAll(selector)) b.setAttribute("aria-pressed", String(b === active));
}

function setBoat(boat) {
  app.boat = boat;
  const isAi = boat === "ai";
  pressOnly("[data-boat]", $(`boat-${boat}`));
  $("boat-name").textContent = isAi ? "USV-01" : "Energy boat";
  $("chip-objects").hidden = $("chip-fov").hidden = !isAi;
  $("chip-chart").hidden = isAi; // the simulator's world is not on the real map
  $("energy-actions").hidden = isAi;
  $("ai-actions").hidden = !isAi;
  $("run-panel").hidden = isAi;
  $("around-panel").hidden = !isAi;
  $("signal-panel").hidden = isAi;
  $("cam-ai").hidden = !isAi;
  $("cam-actions").hidden = !isAi;
  $("cam-energy").hidden = isAi;
  $("cam-sentence").textContent = isAi ? "What USV-01 sees, with the AI's detections drawn on top." : "Live camera images come from USV-01, our autonomous boat.";
  $("objects-panel").hidden = !isAi;
  $("races-panel").hidden = isAi;
  document.querySelector(".perf-row").classList.toggle("solo", !isAi);
  $("r-4-label").textContent = isAi ? "Nearest object" : "Run progress";
  $("p2-label").textContent = isAi ? "AI inference time" : "Average while moving";
  $("p3-label").textContent = isAi ? "Detections in view" : "Signal heard, last 10 min";
  $("p4-label").textContent = isAi ? "GPS rate" : "Time on the water";
  map.scale = isAi ? AI_SCALE : ENERGY_SCALE;
  setFollow(true);
  if (isAi && !ai.bridge) ai.bridge = connectBridge(BRIDGE_URL, aiHandlers(), { onStatus: (status) => (ai.status = status) });
  ai.bridge?.setHandlers(aiHandlers());
  lastText = lastChart = lastAiChart = 0;
  chartKey = "";
}

function setView(view) {
  app.view = view;
  for (const b of document.querySelectorAll("[data-view]")) {
    if (b.dataset.view === view) b.setAttribute("aria-current", "page");
    else b.removeAttribute("aria-current");
  }
  for (const v of ["map", "cameras", "performance"]) $(`view-${v}`).hidden = v !== view;
  ai.bridge?.setHandlers(aiHandlers());
  lastText = lastChart = lastAiChart = 0;
  chartKey = "";
}

for (const b of document.querySelectorAll("[data-boat]")) b.addEventListener("click", () => setBoat(b.dataset.boat));
for (const b of document.querySelectorAll("[data-view]")) b.addEventListener("click", () => setView(b.dataset.view));
$("go-ai").addEventListener("click", () => setBoat("ai"));
$("go-cameras").addEventListener("click", () => setView("cameras"));
for (const chip of document.querySelectorAll("[data-layer]")) {
  chip.addEventListener("click", () => {
    const layer = chip.dataset.layer;
    app.layers[layer] = !app.layers[layer];
    chip.setAttribute("aria-pressed", String(app.layers[layer]));
  });
}

// ---------- map: a schematic chart in metres, north up ----------

const mapCanvas = $("map-canvas");

function setFollow(on) {
  map.follow = on;
  $("m-follow").setAttribute("aria-pressed", String(on));
}
const zoom = (f) => (map.scale = Math.min(60, Math.max(0.02, map.scale * f)));
$("m-in").addEventListener("click", () => zoom(1.6));
$("m-out").addEventListener("click", () => zoom(1 / 1.6));
$("m-follow").addEventListener("click", () => setFollow(true));
$("m-full").addEventListener("click", () => {
  const request = document.fullscreenElement ? document.exitFullscreen() : $("map").requestFullscreen?.();
  request?.catch?.(() => {});
});
mapCanvas.addEventListener("dblclick", () => zoom(1.6));

let drag = null;
mapCanvas.addEventListener("pointerdown", (e) => {
  drag = { x: e.clientX, y: e.clientY };
  mapCanvas.setPointerCapture(e.pointerId);
});
mapCanvas.addEventListener("pointermove", (e) => {
  if (!drag) return;
  const dx = e.clientX - drag.x;
  const dy = e.clientY - drag.y;
  if (map.follow && Math.hypot(dx, dy) < 4) return;
  setFollow(false);
  map.cx -= dx / map.scale;
  map.cy += dy / map.scale;
  drag = { x: e.clientX, y: e.clientY };
});
mapCanvas.addEventListener("pointerup", () => (drag = null));
mapCanvas.addEventListener("pointercancel", () => (drag = null));

// ---------- the harbour map: OpenStreetMap tiles placed by their corners' latitude and longitude ----------

const tiles = new Map();
const lonToTile = (lon, z) => ((lon + 180) / 360) * 2 ** z;
const latToTile = (lat, z) => ((1 - Math.log(Math.tan((lat * Math.PI) / 180) + 1 / Math.cos((lat * Math.PI) / 180)) / Math.PI) / 2) * 2 ** z;
const tileToLon = (x, z) => (x / 2 ** z) * 360 - 180;
const tileToLat = (y, z) => (Math.atan(Math.sinh(Math.PI - (2 * Math.PI * y) / 2 ** z)) * 180) / Math.PI;

function tileImage(z, x, y) {
  const key = `${z}/${x}/${y}`;
  let img = tiles.get(key);
  if (!img) {
    img = new Image();
    img.src = TILE_URL(z, x, y);
    tiles.set(key, img);
    if (tiles.size > TILE_CACHE) tiles.delete(tiles.keys().next().value);
  }
  return img.complete && img.naturalWidth ? img : null;
}

// Draws the tiles under the view; returns false when none could be drawn, so the grid takes over.
function drawTiles(ctx, geo, w, h, px, py) {
  const S = map.scale;
  const kx = 111320 * Math.cos((geo.lat0 * Math.PI) / 180);
  const toLL = (x, y) => [geo.lat0 + y / 110540, geo.lon0 + x / kx];
  const fromLL = (lat, lon) => [(lon - geo.lon0) * kx, (lat - geo.lat0) * 110540];
  // the tile zoom whose pixels are closest to the screen's
  const z = Math.min(19, Math.max(1, Math.round(Math.log2(156543.03 * Math.cos((geo.lat0 * Math.PI) / 180) * S))));
  const [latTop, lonLeft] = toLL(map.cx - w / 2 / S, map.cy + h / 2 / S);
  const [latBottom, lonRight] = toLL(map.cx + w / 2 / S, map.cy - h / 2 / S);
  const x0 = Math.floor(lonToTile(lonLeft, z));
  const x1 = Math.floor(lonToTile(lonRight, z));
  const y0 = Math.floor(latToTile(latTop, z));
  const y1 = Math.floor(latToTile(latBottom, z));
  if ((x1 - x0 + 1) * (y1 - y0 + 1) > MAX_TILES) return false;
  let drawn = 0;
  ctx.save();
  ctx.filter = "saturate(0.4)"; // quieter than the track; browsers without canvas filters show full colour
  for (let tx = x0; tx <= x1; tx++) {
    for (let ty = y0; ty <= y1; ty++) {
      const img = tileImage(z, tx, ty);
      if (!img) continue;
      const [ax, ay] = fromLL(tileToLat(ty, z), tileToLon(tx, z));
      const [bx, by] = fromLL(tileToLat(ty + 1, z), tileToLon(tx + 1, z));
      const sx = Math.floor(px(ax));
      const sy = Math.floor(py(ay));
      ctx.drawImage(img, sx, sy, Math.ceil(px(bx)) - sx + 1, Math.ceil(py(by)) - sy + 1);
      drawn++;
    }
  }
  ctx.restore();
  return drawn > 0;
}

function niceStep(minMetres) {
  const p = 10 ** Math.floor(Math.log10(minMetres));
  return [1, 2, 5, 10].map((k) => k * p).find((s) => s >= minMetres);
}

function drawMap(s) {
  const w = mapCanvas.clientWidth;
  const h = mapCanvas.clientHeight;
  if (!w || !h) return;
  const dpr = devicePixelRatio || 1;
  if (mapCanvas.width !== Math.round(w * dpr) || mapCanvas.height !== Math.round(h * dpr)) {
    mapCanvas.width = Math.round(w * dpr);
    mapCanvas.height = Math.round(h * dpr);
  }
  if (map.follow && s?.pos) {
    map.cx = s.pos.x;
    map.cy = s.pos.y;
  }
  const ctx = mapCanvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const S = map.scale;
  const px = (x) => w / 2 + (x - map.cx) * S;
  const py = (y) => h / 2 - (y - map.cy) * S;
  const color = {
    bg: css("--map-bg"), grid: css("--map-grid"), past: css("--track-past"), accent: css("--accent"), track: css("--accent-track"),
    brand: css("--brand"), surface: css("--surface"), ink: css("--text"), font: css("--font"),
  };

  ctx.fillStyle = color.bg;
  ctx.fillRect(0, 0, w, h);

  const chart = Boolean(s?.geo && app.layers.chart) && drawTiles(ctx, s.geo, w, h, px, py);
  $("map-credit").hidden = !(s?.geo && app.layers.chart);

  // grid in round metres, where there is no harbour map under the track
  const step = niceStep(72 / S);
  ctx.strokeStyle = chart ? "transparent" : color.grid;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let gx = Math.floor((map.cx - w / 2 / S) / step) * step; px(gx) <= w; gx += step) {
    const x = Math.round(px(gx)) + 0.5;
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
  }
  for (let gy = Math.floor((map.cy - h / 2 / S) / step) * step; py(gy) >= 0; gy += step) {
    const y = Math.round(py(gy)) + 0.5;
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
  }
  ctx.stroke();

  $("map-empty").hidden = Boolean(s?.pos);
  if (s?.pos) {
    const line = ({ pts, start, end }, width, stroke) => {
      if (end - start < 1) return;
      ctx.strokeStyle = stroke;
      ctx.lineWidth = width;
      ctx.lineCap = ctx.lineJoin = "round";
      ctx.beginPath();
      for (let i = start; i <= end; i++) {
        const [x, y] = pts[i];
        if (i === start) ctx.moveTo(px(x), py(y));
        else ctx.lineTo(px(x), py(y));
      }
      ctx.stroke();
    };
    if (app.layers.past) line(s.past, 7, color.past);
    if (app.layers.recent) line(s.recent, 3, color.accent);

    const bx = px(s.pos.x);
    const by = py(s.pos.y);
    const rot = ((s.heading ?? 0) * Math.PI) / 180;

    if (s.fov && app.layers.fov && s.heading != null) {
      const r = Math.max(CAMERA_RANGE_M * S, 60);
      const half = ((CAMERA_FOV_DEG / 2) * Math.PI) / 180;
      ctx.save();
      ctx.translate(bx, by);
      ctx.rotate(rot);
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.arc(0, 0, r, -Math.PI / 2 - half, -Math.PI / 2 + half);
      ctx.closePath();
      ctx.globalAlpha = 0.7;
      ctx.fillStyle = color.track;
      ctx.fill();
      ctx.restore();
    }

    if (s.fov && app.layers.objects) {
      ctx.font = `500 12px ${color.font}`;
      for (const o of s.objects) {
        const ox = px(o.x);
        const oy = py(o.y);
        ctx.beginPath();
        ctx.arc(ox, oy, 6, 0, 2 * Math.PI);
        ctx.fillStyle = o.color;
        ctx.fill();
        ctx.lineWidth = 2;
        ctx.strokeStyle = color.surface;
        ctx.stroke();
        ctx.fillStyle = color.ink;
        ctx.fillText(`${o.name} ${o.dist.toFixed(0)} m`, ox + 10, oy + 4);
      }
    }

    // the boat: twin red hulls under a white deck, bow along the heading
    const L = Math.max(BOAT_LENGTH_M * S, 26);
    ctx.save();
    ctx.translate(bx, by);
    ctx.rotate(rot);
    ctx.fillStyle = color.brand;
    for (const side of [-1, 1]) {
      ctx.beginPath();
      ctx.ellipse(side * L * 0.22, 0, L * 0.09, L * 0.5, 0, 0, 2 * Math.PI);
      ctx.fill();
    }
    ctx.fillStyle = color.surface;
    ctx.strokeStyle = color.brand;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.rect(-L * 0.22, -L * 0.16, L * 0.44, L * 0.3);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  // scale bar, bottom right; north arrow, bottom left
  const len = step * S;
  const x1 = w - 16;
  const x0 = x1 - len;
  const y = h - 18;
  ctx.strokeStyle = color.ink;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(x0, y - 5);
  ctx.lineTo(x0, y);
  ctx.lineTo(x1, y);
  ctx.lineTo(x1, y - 5);
  ctx.stroke();
  ctx.fillStyle = color.ink;
  ctx.font = `500 12px ${color.font}`;
  ctx.textAlign = "right";
  ctx.fillText(step >= 1000 ? `${step / 1000} km` : `${step} m`, x1, y - 8);
  ctx.textAlign = "center";
  ctx.font = `700 12px ${color.font}`;
  ctx.fillText("N", 24, h - 14);
  ctx.beginPath();
  ctx.moveTo(24, h - 46);
  ctx.lineTo(29, h - 32);
  ctx.lineTo(24, h - 35);
  ctx.lineTo(19, h - 32);
  ctx.closePath();
  ctx.fill();
  ctx.textAlign = "left";
}

// ---------- main loop ----------

function frame(now) {
  tickEnergy(now);
  const s = app.boat === "energy" ? energyState() : aiState();
  if (app.view === "map" && now - map.lastDraw > MAP_FRAME_MS) {
    map.lastDraw = now;
    drawMap(s);
    if (app.boat === "energy") drawStrip();
  }
  if (app.boat === "ai" && app.view === "cameras") drawCamera();
  if (now - lastText > TEXT_FRAME_MS) {
    lastText = now;
    if (app.boat === "energy" && s) renderEnergyText(s);
    if (app.boat === "ai") renderAiText(s);
  }
  if (app.view === "performance" && now - lastChart > CHART_FRAME_MS / 4) {
    lastChart = now;
    if (app.boat === "energy" && energy.race) drawEnergyChart();
    else if (app.boat === "ai" && now - lastAiChart > CHART_FRAME_MS) {
      lastAiChart = now;
      drawAiChart();
    }
  }
  requestAnimationFrame(frame);
}

setBoat(location.hash === "#ai" ? "ai" : "energy");
requestAnimationFrame(frame);
