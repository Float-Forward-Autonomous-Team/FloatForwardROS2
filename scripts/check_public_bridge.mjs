// Connects to the public foxglove_bridge the way an outsider would and checks that it is
// read-only: no publish/service/parameter capabilities, only the public page's topics, and a
// client trying to publish to the thrusters is refused. Run before sharing a tunnel link:
//   node scripts/check_public_bridge.mjs [ws://localhost:8766]
// Exits 1 if anything is wrong. Needs Node 22+ (built-in WebSocket).

const url = process.argv[2] ?? "ws://localhost:8766";
const FORBIDDEN_CAPS = ["clientPublish", "services", "parameters", "parametersSubscribe", "assets"];
const ALLOWED = [
  /^\/wamv\/sensors\/gps\/gps\/fix$/,
  /^\/gps_track$/,
  /^\/wamv\/sensors\/imu\/imu\/data$/,
  /^\/obstacles\/markers$/,
  /^\/detections$/,
  /^\/detections\/inference_ms$/,
  /^\/public\/cameras\/[a-z_]+\/compressed$/,
];

const problems = [];
const topics = new Set();
let caps = null;

const ws = new WebSocket(url, ["foxglove.sdk.v1", "foxglove.websocket.v1"]);
const done = (code) => {
  ws.close();
  process.exit(code);
};
setTimeout(finish, 4000);

ws.onerror = () => {
  console.error(`Could not connect to ${url}. Is \`make race\` running?`);
  done(1);
};

ws.onopen = () => {
  // try to become a publisher on the thruster topic, as an attacker would
  ws.send(
    JSON.stringify({
      op: "advertise",
      channels: [{ id: 1, topic: "/wamv/thrusters/main/thrust", encoding: "cdr", schemaName: "std_msgs/msg/Float64" }],
    }),
  );
};

ws.onmessage = (event) => {
  if (typeof event.data !== "string") return;
  const msg = JSON.parse(event.data);
  if (msg.op === "serverInfo") caps = msg.capabilities ?? [];
  if (msg.op === "advertise") for (const c of msg.channels) topics.add(c.topic);
  if (msg.op === "status") console.log(`bridge says (level ${msg.level}): ${msg.message}`);
};

function finish() {
  if (caps === null) problems.push("no serverInfo received");
  else for (const cap of FORBIDDEN_CAPS) if (caps.includes(cap)) problems.push(`capability "${cap}" is enabled`);
  for (const topic of topics) if (!ALLOWED.some((re) => re.test(topic))) problems.push(`topic not on the public list: ${topic}`);

  console.log(`capabilities: ${JSON.stringify(caps)}`);
  console.log(`topics (${topics.size}):\n  ${[...topics].sort().join("\n  ")}`);
  if (problems.length) {
    console.error(`\nNOT SAFE to share:\n  ${problems.join("\n  ")}`);
    done(1);
  }
  console.log("\nOK: read-only, public topics only.");
  done(0);
}
