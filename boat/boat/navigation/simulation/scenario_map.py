"""Rasterize a Gazebo world's collision geometry into a surface occupancy grid.

This reads the same SDF and cached model assets that Gazebo loads. It does not
infer navigability from visual colours. Plane collisions (the water / seabed)
are ignored; shore and object collision footprints are occupied.
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import unquote, urlparse

from PIL import Image, ImageDraw


def _identity():
    return ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def _multiply(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)) for i in range(4)
    )


def _point(matrix, xyz):
    x, y, z = xyz
    return tuple(sum(matrix[i][k] * (x, y, z, 1.0)[k] for k in range(4)) for i in range(3))


def _pose(text):
    values = [float(v) for v in text.split()] if text else []
    values += [0.0] * (6 - len(values))
    x, y, z, roll, pitch, yaw = values[:6]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y),
        (-sp, cp * sr, cp * cr, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def _float_list(text):
    return [float(item) for item in text.split()]


class ScenarioMap:
    """Build a bounded 2D grid from world and model collision geometry."""

    def __init__(self, origin_x, origin_y, width_m, height_m, resolution_m, model_roots):
        """Configure the grid extent and Gazebo model lookup paths."""
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.resolution = float(resolution_m)
        self.width = math.ceil(float(width_m) / self.resolution)
        self.height = math.ceil(float(height_m) / self.resolution)
        if self.resolution <= 0 or self.width <= 0 or self.height <= 0:
            raise ValueError("Map bounds and resolution must be positive")
        self.model_roots = [Path(path) for path in model_roots]
        self.image = Image.new("L", (self.width, self.height), 0)
        self.draw = ImageDraw.Draw(self.image)
        self.collision_count = 0

    def _pixel(self, x, y):
        return (
            (x - self.origin_x) / self.resolution,
            self.height - 1 - (y - self.origin_y) / self.resolution,
        )

    def _polygon(self, points):
        self.draw.polygon([self._pixel(x, y) for x, y, _ in points], fill=100)

    def draw_circle(self, image, x, y, radius):
        """Draw a moving object's footprint on a copy of the static map."""
        draw = ImageDraw.Draw(image)
        px, py = self._pixel(x, y)
        radius_px = max(0.5, radius / self.resolution)
        draw.ellipse((px - radius_px, py - radius_px, px + radius_px, py + radius_px), fill=100)

    def values(self, image=None):
        """Return occupancy values in ROS bottom-left-first row order."""
        image = image if image is not None else self.image
        return list(image.transpose(Image.Transpose.FLIP_TOP_BOTTOM).tobytes())

    def _resolve_model(self, uri, base_dir):
        uri = uri.strip()
        if uri.startswith("https://fuel.gazebosim.org/"):
            parts = [unquote(p) for p in urlparse(uri).path.strip("/").split("/")]
            if len(parts) < 4 or parts[2] != "models":
                raise ValueError(f"Unsupported Fuel model URI: {uri}")
            directory = (
                Path.home()
                / ".gz/fuel/fuel.gazebosim.org"
                / parts[1].lower()
                / "models"
                / parts[3].lower()
            )
            versions = (
                sorted(
                    (p for p in directory.iterdir() if p.name.isdigit()),
                    key=lambda p: int(p.name),
                    reverse=True,
                )
                if directory.exists()
                else []
            )
            if versions:
                return versions[0] / "model.sdf"
        else:
            path = Path(uri.removeprefix("model://"))
            candidates = [base_dir / path, *(root / path for root in self.model_roots)]
            for candidate in candidates:
                if candidate.is_file():
                    return candidate
                if (candidate / "model.sdf").is_file():
                    return candidate / "model.sdf"
        raise FileNotFoundError(f"Gazebo model is not cached or installed: {uri}")

    def load_world(self, world_sdf):
        """Load every collision footprint included by the selected world."""
        world_path = Path(world_sdf)
        world = ET.parse(world_path).getroot().find("world")
        if world is None:
            raise ValueError(f"No <world> in {world_path}")
        for include in world.findall("include"):
            self._include(include, _identity(), world_path.parent)
        for model in world.findall("model"):
            self._model(model, _identity(), world_path.parent)
        if not self.collision_count:
            raise ValueError(f"No supported collision geometry found in {world_path}")

    def _include(self, include, parent, base_dir):
        uri = include.findtext("uri")
        model_path = self._resolve_model(uri, base_dir)
        model = ET.parse(model_path).getroot().find("model")
        if model is None:
            raise ValueError(f"No <model> in {model_path}")
        self._model(model, parent, model_path.parent, include.findtext("pose"))

    def _model(self, model, parent, base_dir, override_pose=None):
        matrix = _multiply(
            parent, _pose(override_pose if override_pose is not None else model.findtext("pose"))
        )
        for link in model.findall("link"):
            link_matrix = _multiply(matrix, _pose(link.findtext("pose")))
            for collision in link.findall("collision"):
                collision_matrix = _multiply(link_matrix, _pose(collision.findtext("pose")))
                geometry = collision.find("geometry")
                if geometry is not None:
                    self._geometry(geometry, collision_matrix, base_dir)
        for child in model.findall("model"):
            self._model(child, matrix, base_dir)
        for include in model.findall("include"):
            self._include(include, matrix, base_dir)

    def _geometry(self, geometry, matrix, base_dir):
        if geometry.find("plane") is not None:
            return
        cylinder = geometry.find("cylinder")
        sphere = geometry.find("sphere")
        box = geometry.find("box")
        mesh = geometry.find("mesh")
        if cylinder is not None:
            radius = float(cylinder.findtext("radius"))
            half_length = float(cylinder.findtext("length")) / 2
            points = [
                _point(
                    matrix,
                    (
                        radius * math.cos(2 * math.pi * i / 24),
                        radius * math.sin(2 * math.pi * i / 24),
                        z,
                    ),
                )
                for z in (-half_length, half_length)
                for i in range(24)
            ]
            self._convex_hull(points)
        elif sphere is not None:
            radius = float(sphere.findtext("radius"))
            points = [
                _point(
                    matrix,
                    (
                        radius * math.cos(2 * math.pi * i / 32),
                        radius * math.sin(2 * math.pi * i / 32),
                        0.0,
                    ),
                )
                for i in range(32)
            ]
            self._polygon(points)
        elif box is not None:
            hx, hy, hz = (v / 2 for v in _float_list(box.findtext("size")))
            points = [
                _point(matrix, (x, y, z)) for x in (-hx, hx) for y in (-hy, hy) for z in (-hz, hz)
            ]
            self._convex_hull(points)
        elif mesh is not None:
            uri = mesh.findtext("uri")
            scale = _float_list(mesh.findtext("scale") or "1 1 1")
            mesh_path = self._resolve_mesh(uri, base_dir)
            self._mesh(mesh_path, matrix, scale)
        else:
            raise ValueError(
                f"Unsupported collision geometry: {ET.tostring(geometry, encoding='unicode')}"
            )
        self.collision_count += 1

    def _resolve_mesh(self, uri, base_dir):
        path = Path(uri.removeprefix("model://"))
        for candidate in (base_dir / path, *(root / path for root in self.model_roots)):
            if candidate.is_file():
                return candidate
        raise FileNotFoundError(f"Collision mesh is not cached or installed: {uri}")

    def _convex_hull(self, points):
        planar = sorted(set((x, y) for x, y, _ in points))
        if len(planar) < 3:
            return

        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        lower, upper = [], []
        for p in planar:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)
        for p in reversed(planar):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)
        self._polygon([(x, y, 0.0) for x, y in lower[:-1] + upper[:-1]])

    def _mesh(self, path, matrix, scale):
        if path.suffix.lower() != ".dae":
            raise ValueError(f"Only COLLADA collision meshes are supported: {path}")
        root = ET.parse(path).getroot()
        ns = {"c": root.tag.split("}")[0].removeprefix("{")}
        for geometry in root.findall(".//c:library_geometries/c:geometry", ns):
            mesh = geometry.find("c:mesh", ns)
            vertices = mesh.find("c:vertices", ns)
            source_ref = next(
                inp.get("source")
                for inp in vertices.findall("c:input", ns)
                if inp.get("semantic") == "POSITION"
            ).removeprefix("#")
            source = next(
                item for item in mesh.findall("c:source", ns) if item.get("id") == source_ref
            )
            numbers = _float_list(source.find("c:float_array", ns).text)
            raw_points = [tuple(numbers[i : i + 3]) for i in range(0, len(numbers), 3)]
            mesh_matrix = matrix
            for node in root.findall(".//c:library_visual_scenes//c:node", ns):
                instance = node.find("c:instance_geometry", ns)
                if instance is not None and instance.get("url") == "#" + geometry.get("id"):
                    transform = node.find("c:matrix", ns)
                    if transform is not None:
                        vals = _float_list(transform.text)
                        collada_matrix = tuple(
                            tuple(vals[j * 4 + i] for j in range(4)) for i in range(4)
                        )
                        mesh_matrix = _multiply(matrix, collada_matrix)
                    break
            points = [
                _point(mesh_matrix, (p[0] * scale[0], p[1] * scale[1], p[2] * scale[2]))
                for p in raw_points
            ]
            for triangles in mesh.findall("c:triangles", ns):
                inputs = triangles.findall("c:input", ns)
                stride = max(int(inp.get("offset", 0)) for inp in inputs) + 1
                vertex_offset = next(
                    int(inp.get("offset", 0)) for inp in inputs if inp.get("semantic") == "VERTEX"
                )
                indices = [int(v) for v in triangles.find("c:p", ns).text.split()]
                for start in range(0, len(indices), stride * 3):
                    triangle = [
                        points[indices[start + vertex_offset + stride * i]] for i in range(3)
                    ]
                    self._polygon(triangle)
