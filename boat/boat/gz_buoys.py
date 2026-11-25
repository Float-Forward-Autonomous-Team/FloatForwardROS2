"""Simulation only: put race buoys into Gazebo and take them out again."""

import subprocess
import time

from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

RELEASE_TOPIC = "/vrx/release"
PREFIX = "race_buoy_"
TIMEOUT_MS = 5000

# Single quotes only: the text goes inside a double-quoted message field.
SDF = """<?xml version='1.0'?>
<sdf version='1.9'><model name='{name}'><static>true</static><link name='link'>
  <visual name='v'><pose>0 0 0.45 0 0 0</pose>
    <geometry><cylinder><radius>{radius}</radius><length>1.3</length></cylinder></geometry>
    <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>
  <collision name='c'><pose>0 0 0.45 0 0 0</pose>
    <geometry><cylinder><radius>{radius}</radius><length>1.3</length></cylinder></geometry>
  </collision>
</link></model></sdf>"""


class GazeboCourse:
    """Talks to the running Gazebo world."""

    def __init__(self, boat_model="wamv"):
        """Find the world and start listening for the boat's position."""
        self.gz = GzNode()
        self.boat_model = boat_model
        self.boat = None  # latest (x, y) of the boat in the world
        self.existing = set()
        # Gazebo removes models late, so every race needs fresh names.
        self.run = int(time.time()) % 100000
        self.world = next(
            (
                t.split("/")[2]
                for t in self.gz.topic_list()
                if t.startswith("/world/") and t.endswith("/pose/info")
            ),
            None,
        )
        if self.world is None:
            raise RuntimeError("no Gazebo world found; start `make vrx-race` first")
        self.release_pub = self.gz.advertise(RELEASE_TOPIC, Empty)
        if not self.gz.subscribe(Pose_V, f"/world/{self.world}/pose/info", self.on_poses):
            raise RuntimeError("could not read model positions from Gazebo")

    def on_poses(self, msg):
        """Remember where the boat is and which race buoys exist (Gazebo thread)."""
        self.existing = {p.name for p in msg.pose if p.name.startswith(PREFIX)}
        for p in msg.pose:
            if p.name == self.boat_model:
                self.boat = (p.position.x, p.position.y)

    def call(self, service, kind, request):
        """Call a world service with the `gz` tool; True if it answered yes.

        The tool runs outside this process: a request from here would wait for a reply
        that arrives on the same thread as the pose messages.
        """
        command = ["gz", "service", "-s", f"/world/{self.world}/{service}"]
        command += ["--reqtype", f"gz.msgs.{kind}", "--reptype", "gz.msgs.Boolean"]
        command += ["--timeout", str(TIMEOUT_MS), "--req", request]
        reply = subprocess.run(command, capture_output=True, text=True, timeout=60)
        return "data: true" in reply.stdout

    def remove_old(self):
        """Remove the buoys of an earlier race."""
        for name in sorted(self.existing):
            self.call("remove", "Entity", f'name: "{name}", type: MODEL')

    def spawn(self, name, x, y, radius, rgb):
        """Put a static buoy at world position (x, y); True if Gazebo accepted it."""
        name = f"{PREFIX}{self.run}_{name}"
        sdf = SDF.format(name=name, radius=radius, rgb=" ".join(map(str, rgb)))
        sdf = " ".join(sdf.split())
        pose = f"position {{x: {x} y: {y}}} orientation {{w: 1}}"
        return self.call("create", "EntityFactory", f'sdf: "{sdf}" name: "{name}" pose {{{pose}}}')

    def release(self):
        """Free the boat from its start platform; does nothing if it is already free."""
        self.release_pub.publish(Empty())
