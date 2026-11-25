// Connection to foxglove_bridge: subscribes to the topics the page asks for and hands
// back decoded ROS 2 messages. The same WebSocket the Foxglove app uses, so the
// page needs no backend of its own.

import { FoxgloveClient } from "@foxglove/ws-protocol";
// These two are CommonJS packages: the CDN only exposes them as a default export.
import rosmsg from "@foxglove/rosmsg";
import rosmsg2 from "@foxglove/rosmsg2-serialization";

const { parse } = rosmsg;
const { MessageReader } = rosmsg2;

const RECONNECT_MS = 2000;
// foxglove_bridge 3.x only accepts the newer "foxglove.sdk.v1" name for the same
// protocol; offer it alongside the library's own name so older bridges work too.
const SUBPROTOCOLS = ["foxglove.sdk.v1", FoxgloveClient.SUPPORTED_SUBPROTOCOL];

// handlers: { "<topic>": (message, receiveTimeMs) => void }; only these topics are streamed.
// onStatus(state, detail): state is "connecting" | "connected" | "disconnected"
// onTopics(channels): every topic the bridge currently advertises
// Returns { setHandlers(handlers), close() }; setHandlers changes the subscriptions live.
export function connectBridge(url, handlers, { onStatus = () => {}, onTopics = () => {}, WebSocketImpl = WebSocket } = {}) {
  let client = null;
  let stopped = false;
  let channels = new Map(); // channel id -> channel
  let subscriptions = new Map(); // channel id -> { subId, reader }
  let bySubId = new Map(); // subscription id -> { topic, reader }

  // Subscribe to wanted topics that are advertised, drop the ones no longer wanted.
  function sync() {
    if (!client) return;
    for (const [channelId, sub] of subscriptions) {
      const channel = channels.get(channelId);
      if (!channel || !handlers[channel.topic]) {
        if (channel) client.unsubscribe(sub.subId);
        subscriptions.delete(channelId);
        bySubId.delete(sub.subId);
      }
    }
    for (const channel of channels.values()) {
      if (subscriptions.has(channel.id) || !handlers[channel.topic]) continue;
      if (channel.encoding !== "cdr" || channel.schemaEncoding === "ros2idl") continue;
      const reader = new MessageReader(parse(channel.schema, { ros2: true }));
      const subId = client.subscribe(channel.id);
      subscriptions.set(channel.id, { subId, reader });
      bySubId.set(subId, { topic: channel.topic, reader });
    }
  }

  function open() {
    onStatus("connecting", url);
    channels = new Map();
    subscriptions = new Map();
    bySubId = new Map();
    client = new FoxgloveClient({ ws: new WebSocketImpl(url, SUBPROTOCOLS) });

    client.on("open", () => onStatus("connected", url));
    client.on("error", (err) => onStatus("disconnected", String(err?.message ?? err)));
    client.on("close", () => {
      onStatus("disconnected", url);
      if (!stopped) setTimeout(open, RECONNECT_MS);
    });
    client.on("advertise", (added) => {
      for (const channel of added) channels.set(channel.id, channel);
      sync();
      onTopics([...channels.values()]);
    });
    client.on("unadvertise", (removed) => {
      for (const id of removed) channels.delete(id);
      sync();
      onTopics([...channels.values()]);
    });
    client.on("message", ({ subscriptionId, data }) => {
      const sub = bySubId.get(subscriptionId);
      const handler = sub && handlers[sub.topic];
      if (!handler) return;
      const bytes = new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
      handler(sub.reader.readMessage(bytes), Date.now());
    });
  }

  open();
  return {
    setHandlers(next) {
      handlers = next;
      sync();
    },
    close() {
      stopped = true;
      client?.close();
    },
  };
}
