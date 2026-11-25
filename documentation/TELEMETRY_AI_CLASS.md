# Telemetry for the AI Class

How data gets from the boat's sensors to a screen on shore during a race, and how we test it.

**Last updated:** 25 November 2026. **Keep this file current:** whenever the sensors, topics, bridge,
tunnel, radio or tests change, update the matching section, the status table and the change log.

This repository is public: never put passwords, tokens, the real tunnel address, email addresses or
internal race data in this file.

## Status

| Part | Status |
|---|---|
| ROS 2 topics in simulation | Works |
| `foxglove_bridge` for the team (port 8765) | Works on one computer |
| Read-only bridge for the public (port 8766) | Works with `make race`, not with `make slalom` |
| Public live page through a Cloudflare quick tunnel | `make tunnel`, run from a Mac |
| Viewing from a second computer | To test, see [Testing with two computers](#testing-with-two-computers) |
| Team tunnel with login (8765) | Planned |
| Steering from detections | Planned; the boat steers on GPS and IMU |
| Camera compression for the front and rear cameras | Planned; `public_camera` knows only the old three |
| Jetson, sensor drivers, LTE modem | To be decided |
| Safety radio | To be decided |

## What happens to the data

Target for race day. ROS 2, the bridge and `cloudflared` all run on the Jetson.

```
 ON THE BOAT
┌──────────────────────────────────────────────────────────────┐
│ 1. SENSORS       measure the world                           │
│    camera = pixels, LiDAR = distances, GPS = position,       │
│    IMU = heading and turn rate                               │
└──────────────────────────────┬───────────────────────────────┘
                               │ cables: USB, Ethernet, CAN
                               ▼
┌─ JETSON ─────────────────────────────────────────────────────┐
│ 2. DRIVERS       read each sensor, add a timestamp,          │
│                  publish it as a ROS topic                   │
│                              ▼                               │
│ 3. ROS NODES     perception: pixels ─> "buoy, 13 m ahead"    │
│                  planning:   where to go                     │
│                  control:    thrust and steering angle ──────┼─> MOTOR + STEERING
│                              │                               │
│                              ├──────────────> DISK           │
│                              │   everything recorded         │
│                              ▼                               │
│ 4. COMPRESS      camera image ─> small JPEG                  │
│                              ▼                               │
│ 5. BRIDGE        takes only the requested topics,            │
│                  ROS messages ─> WebSocket                   │
│                              ▼                               │
│ 6. CLOUDFLARED   encrypts it and puts it in the tunnel       │
└──────────────────────────────┬───────────────────────────────┘
                               │ Ethernet or USB
                               ▼
┌──────────────────────────────────────────────────────────────┐
│ 7. LTE MODEM     turns it into a mobile radio signal         │
└──────────────────────────────┬───────────────────────────────┘
                               ▼  mobile network
┌──────────────────────────────────────────────────────────────┐
│ 8. CELL TOWER    passes it to the internet                   │
└──────────────────────────────┬───────────────────────────────┘
                               ▼
┌──────────────────────────────────────────────────────────────┐
│ 9. CLOUDFLARE    checks the login, joins boat and laptop     │
└──────────────────────────────┬───────────────────────────────┘
                               │ encrypted internet
 ON SHORE                      ▼
┌─ LAPTOP ─────────────────────────────────────────────────────┐
│ 10. CLOUDFLARED  decrypts it ─> localhost:9000               │
│                              ▼                               │
│ 11. FOXGLOVE     decodes the messages, draws the map,        │
│                  cameras, 3D view and scoreboard             │
└──────────────────────────────────────────────────────────────┘
```

Steps 1-3 drive the boat and never wait for the internet. Steps 4-11 are only for watching: if they
fail, the boat keeps driving and recording.

### What the bridge does

`foxglove_bridge` is a ROS 2 node that translates between ROS (DDS) and a browser (WebSocket):

1. It tells Foxglove which topics exist and their format.
2. Foxglove asks for the topics its open panels need.
3. The bridge forwards only those, as they arrive, and stops when the panel closes.

It does not read sensors, detect, store, or create the link to shore. By default it also lets
Foxglove publish messages, change parameters and call services, so **port 8765 must never be reachable
without a login**.

### How much data

Estimates at the stock simulator's settings; the real sensors are not chosen.

| Data | Approximate size | Send to shore? |
|---|---|---|
| Raw camera, 1280x720 at 30 fps | 660 Mbit/s each | No, record on board |
| Full LiDAR cloud, 10 Hz | 77 Mbit/s | No, record on board |
| JPEG video, 640 px at 5 fps | 1-2 Mbit/s each | Yes |
| GPS, speed, state, detected positions | Well under 1 Mbit/s | Yes |

## Rules that matter

AI Class Technical Rules 2027; check the official document for the exact wording.

| Rule | Meaning for telemetry |
|---|---|
| REQ_5 | Safety data to shore from at least 2 NM, should not rely on Wi-Fi or direct grid connection |
| REQ_45, REQ_61 | Remote access; the operator can always take control |
| REQ_60 | A dashboard showing the AI: boxes, labels, distances, point cloud, localisation |
| REQ_64 | Link lost for 5 s: stop. Back within 15 s: may continue. After that: shut down |

**Open question for the organisers:** does "direct grid connection" exclude LTE for safety data? Until
answered, safety data and remote takeover go over a separate radio. Foxglove and Cloudflare are for
watching only.

## Which Cloudflare setup

**Recommended: a named Cloudflare Tunnel, with Cloudflare Access as the login, on the free plan.**

| Option | Use it for | Why |
|---|---|---|
| **Named tunnel + Access** | Team dashboard, 8765 | Fixed address, team login, the boat needs no public IP. Needs a domain on Cloudflare |
| Quick tunnel (`make tunnel`) | Public page 8766, short tests | No account or domain, but a new address each time, no login, no uptime guarantee |
| Cloudflare One Client (WARP) | Not now | Acts as a VPN; more to set up on each laptop |

The free Zero Trust plan is meant for teams under 50 users. Check the current terms when signing up.

Also worth comparing before we decide: **Foxglove remote access** (`remote_access:=true` on the
bridge), which connects through Foxglove itself without Cloudflare. Check that our bridge version is
built with it, and what it costs.

Notes:

- Cloudflare can technically read the traffic passing through it.
- `cloudflared` needs outbound port 7844. Some campus networks may block it; a phone hotspot usually works.

### Setup

1. Add a domain to a team Cloudflare account.
2. In Zero Trust, create a self-hosted Access application for `boat.OURDOMAIN` that allows only the
   team's emails. **Do this before step 4**, so the boat is never reachable without a login.
3. Install `cloudflared` on both computers: `brew install cloudflared` (Mac),
   `winget install Cloudflare.cloudflared` (Windows), the `linux-arm64` package from Cloudflare's GitHub releases
   (Jetson).
4. On the boat computer:

   ```bash
   cloudflared tunnel login
   cloudflared tunnel create boat-test              # use "boat" on the Jetson later
   cloudflared tunnel route dns boat-test boat.OURDOMAIN
   ```

   `create` prints the tunnel's ID. Put it in `~/.cloudflared/config.yml`:

   ```yaml
   tunnel: TUNNEL-ID
   credentials-file: /full/path/to/.cloudflared/TUNNEL-ID.json
   ingress:
     - hostname: boat.OURDOMAIN
       service: tcp://127.0.0.1:8765
     - service: http_status:404
   ```

   Check it with `cloudflared tunnel ingress validate`. Never commit anything from `~/.cloudflared/`.

## Testing with two computers

**Computer A is the boat**: it runs the simulator, ROS 2 and the bridge, as the Jetson will.
**Computer B is the shore**: it needs only a browser with Foxglove and, from test 3, `cloudflared`.

```
 COMPUTER A (boat)                                    COMPUTER B (shore)
 simulator ─> ROS 2 ─> foxglove_bridge 8765 ═══════> localhost:9000 ─> Foxglove
                                          test 1: SSH on the same network
                                          test 3: Cloudflare, with login
                                          test 4: same, with A on mobile data
```

On A, in two terminals, for tests 1, 3 and 4:

```bash
make vrx-race     # terminal 1
make slalom       # terminal 2, once the boat has spawned
```

On B, import `foxglove/slalom_layout.json` into Foxglove. The boat model in 3D may not appear on the
second computer; that does not matter for these tests.

### Test 1: same network, no internet

The container's ports are open only to A itself (`127.0.0.1` in `compose.yml`), so B goes through SSH.

1. A (Mac): System Settings > General > Sharing > turn on **Remote Login**. Find A's address with
   `ipconfig getifaddr en0`.
2. B: `ssh -N -L 9000:127.0.0.1:8765 USER@ADDRESS-OF-A`
3. B: open `ws://localhost:9000` in Foxglove.
4. Afterwards, turn Remote Login off again.

If B cannot reach A, the network may separate its clients; connect both to a phone hotspot.
**Pass:** the scoreboard, map and cameras update on B.

### Test 2: public page through a quick tunnel

1. A, three terminals: `make vrx WORLD=sydney_regatta`, `make race`, `make tunnel`.
2. B, on another network: open the link that `make tunnel` prints.

This shows only the read-only public topics, not the slalom dashboard. **Pass:** the boat moves on the page on B.

### Test 3: team dashboard through Cloudflare

Needs the [setup](#setup) above.

1. A: `cloudflared tunnel run boat-test`
2. B: `cloudflared access tcp --hostname boat.OURDOMAIN --url localhost:9000`, then log in when the
   browser opens.
3. B: open `ws://localhost:9000` in Foxglove.

**Pass:** Foxglove works on B, and someone outside the team is refused at the login.

### Test 4: like on the water

Repeat test 3 with **A on a phone hotspot** (the boat is the side on LTE) and B on a different network.

| Check | How | Pass |
|---|---|---|
| Delay | Show `/race/elapsed` on both screens, film both with one phone, compare. Simulation time runs slower than real time, so divide by the simulator's speed (about 0.4 in one recording) | Write down the result |
| Bandwidth | Activity Monitor > Network on A, with only scoreboard and map open, then with cameras | Write down the result |
| Link lost | Hotspot off for 10 s, then on | The simulated boat keeps driving; record whether Foxglove and the tunnel reconnect by themselves, and how fast |
| Long run | Leave it on for 30 minutes | No disconnects |

Start with the scoreboard and map only. The slalom layout opens raw images, which can be too much for a
mobile connection.

This does not test REQ_64: stopping when the link is lost needs a watchdog node, which does not exist yet.

### Results

| Date | Test | Networks of A and B | Delay | Bandwidth | Result |
|---|---|---|---|---|---|
| | | | | | not tested yet |

## Next steps

- [ ] Test 1, then test 2
- [ ] Team Cloudflare account and a domain
- [ ] Test 3, then test 4
- [ ] Compare with Foxglove remote access
- [ ] Compress the front and rear cameras; make a small layout for mobile data
- [ ] Read-only public bridge in the slalom launch
- [ ] Recording on the boat computer
- [ ] Choose Jetson, LTE modem and safety radio; ask the organisers about REQ_5
- [ ] Watchdog node for REQ_64

## Change log

| Date | Change |
|---|---|
| 25 Nov 2026 | First version: data path, Cloudflare recommendation, two-computer tests |

## Sources

- [Cloudflare: arbitrary TCP with `cloudflared access`](https://developers.cloudflare.com/cloudflare-one/access-controls/applications/non-http/cloudflared-authentication/arbitrary-tcp/)
- [Cloudflare: quick tunnels](https://developers.cloudflare.com/tunnel/get-started/quick-tunnels/)
- [Cloudflare: tunnel with firewall](https://developers.cloudflare.com/cloudflare-one/networks/connectors/cloudflare-tunnel/configure-tunnels/tunnel-with-firewall/)
- [Cloudflare: Zero Trust plans](https://www.cloudflare.com/plans/sase-zero-trust.md)
- [Foxglove bridge](https://docs.foxglove.dev/docs/fleet/bridge) and
  [remote access](https://docs.foxglove.dev/docs/fleet/remote-access)
