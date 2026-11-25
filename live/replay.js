// Plays a short USV-01 recording on a loop, through the same interface as connectBridge,
// so the public page has something to show while the boat is not streaming.
// The recording is made with `make record` and `make replay-data` (live/tools/build_usv_replay.py).

const LOOP_GAP_S = 2; // pause between the end of the recording and the next loop
const PREFETCH = 6; // camera frames fetched ahead of time

// handlers: { "<topic>": (message, receiveTimeMs) => void }, as for connectBridge.
// onStatus(state): "connecting" | "replay" | "disconnected"
// onLoop(): called each time the recording starts again, to clear trails that would jump.
// Returns { setHandlers(handlers), close() }.
export function connectReplay(baseUrl, handlers, { onStatus = () => {}, onLoop = () => {} } = {}) {
  let stopped = false;
  let replay = null;
  let timer = null;
  const frames = new Map(); // url -> Promise<Uint8Array>

  const fetchFrame = (url) => {
    if (!frames.has(url)) {
      frames.set(
        url,
        fetch(url).then((res) => (res.ok ? res.arrayBuffer() : Promise.reject(res.status))).then((buf) => new Uint8Array(buf)),
      );
      frames.get(url).catch(() => frames.delete(url));
    }
    return frames.get(url);
  };

  // header stamps move on by one recording length per loop, so speed and charts keep going forward
  function shifted(msg, offsetS) {
    if (!msg.header || !offsetS) return msg;
    const ns = msg.header.stamp.nanosec + Math.round((offsetS % 1) * 1e9);
    const sec = msg.header.stamp.sec + Math.floor(offsetS) + Math.floor(ns / 1e9);
    return { ...msg, header: { ...msg.header, stamp: { sec, nanosec: ns % 1e9 } } };
  }

  function run() {
    const period = replay.duration + LOOP_GAP_S;
    let loop = -1;
    let msgIndex = 0;
    const frameIndex = {};
    const t0 = performance.now();

    function tick() {
      if (stopped) return;
      const elapsed = (performance.now() - t0) / 1000;
      const thisLoop = Math.floor(elapsed / period);
      if (thisLoop !== loop) {
        loop = thisLoop;
        msgIndex = 0;
        for (const name in replay.frames) frameIndex[name] = 0;
        onLoop();
      }
      const t = elapsed - loop * period;
      const offset = loop * period;
      const now = Date.now();

      while (msgIndex < replay.messages.length && replay.messages[msgIndex][0] <= t) {
        const [, topic, msg] = replay.messages[msgIndex++];
        handlers[topic]?.(shifted(msg, offset), now);
      }

      for (const [name, list] of Object.entries(replay.frames)) {
        const topic = `/public/cameras/${name}/compressed`;
        let i = frameIndex[name];
        let due = null;
        while (i < list.length && list[i][0] <= t) due = list[i++];
        frameIndex[name] = i;
        if (!handlers[topic]) continue;
        for (let k = i; k < Math.min(i + PREFETCH, list.length); k++) fetchFrame(new URL(list[k][1], baseUrl).href);
        if (due) {
          fetchFrame(new URL(due[1], baseUrl).href)
            .then((data) => handlers[topic]?.({ format: "jpeg", data }, Date.now()))
            .catch(() => {});
        }
      }
      timer = setTimeout(tick, 50);
    }
    tick();
  }

  onStatus("connecting");
  fetch(new URL("replay.json", baseUrl))
    .then((res) => (res.ok ? res.json() : Promise.reject(new Error(`replay.json: ${res.status}`))))
    .then((data) => {
      if (stopped) return;
      replay = data;
      onStatus("replay");
      run();
    })
    .catch(() => onStatus("disconnected"));

  return {
    setHandlers(next) {
      handlers = next;
    },
    close() {
      stopped = true;
      clearTimeout(timer);
    },
  };
}
