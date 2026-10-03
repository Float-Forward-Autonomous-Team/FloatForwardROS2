"""Write the values from boat_params.yaml into a generated WAM-V URDF, in place."""

import math
import sys
import xml.etree.ElementTree as ET

import yaml


def plugins(root, name):
    """Return the Gazebo plugins called `name`, failing if the URDF has none."""
    found = [p for p in root.iter("plugin") if p.get("name") == name]
    if not found:
        sys.exit(f"apply_boat_params: no <plugin name='{name}'> in the URDF")
    return found


def set_child(parent, tag, value):
    """Set the text of `parent`'s `tag` child, failing if it does not exist."""
    child = parent.find(tag)
    if child is None:
        sys.exit(f"apply_boat_params: no <{tag}> in <plugin name='{parent.get('name')}'>")
    child.text = str(value)


def apply_hull(root, hull):
    """Set hull mass and inertia, and the size and position of the buoyancy hulls."""
    links = [link for link in root.iter("link") if link.get("name", "").endswith("/base_link")]
    if len(links) != 1:
        sys.exit(f"apply_boat_params: expected one base_link, found {len(links)}")
    inertial = links[0].find("inertial")
    inertial.find("mass").set("value", str(hull["mass"]))
    for axis, value in hull["inertia"].items():
        inertial.find("inertia").set(axis, str(value))

    for surface in plugins(root, "vrx::Surface"):
        set_child(surface, "hull_length", hull["length"])
        set_child(surface, "hull_radius", hull["radius"])
        for point in surface.find("points"):
            x, y, z = point.text.split()
            side = 1 if float(y) > 0 else -1
            point.text = f"{x} {side * hull['spacing']} {z}"


def apply_drag(root, drag):
    """Set the drag coefficients of the hydrodynamics plugin."""
    for hydro in plugins(root, "vrx::SimpleHydrodynamics"):
        for tag, value in drag.items():
            set_child(hydro, tag, value)


def apply_thruster(root, thruster):
    """Set the forward thrust limit of every engine."""
    for engine in plugins(root, "gz::sim::systems::Thruster"):
        set_child(engine, "max_thrust_cmd", thruster["max_thrust"])


def sensors(root, sensor_type):
    """Return the Gazebo sensors of `sensor_type`, failing if the URDF has none."""
    found = [s for s in root.iter("sensor") if s.get("type") == sensor_type]
    if not found:
        sys.exit(f"apply_boat_params: no <sensor type='{sensor_type}'> in the URDF")
    return found


def set_path(sensor, path, value):
    """Set the text of the element at `path` under `sensor`, creating it if missing."""
    node = sensor
    for tag in path.split("/"):
        child = node.find(tag)
        node = child if child is not None else ET.SubElement(node, tag)
    node.text = str(value)


def apply_camera(root, camera):
    """Set resolution, field of view, rate and noise of every camera."""
    for sensor in sensors(root, "camera"):
        set_path(sensor, "update_rate", camera["rate"])
        set_path(sensor, "camera/horizontal_fov", math.radians(camera["horizontal_fov"]))
        set_path(sensor, "camera/image/width", camera["width"])
        set_path(sensor, "camera/image/height", camera["height"])
        set_path(sensor, "camera/noise/stddev", camera["noise_stddev"])


def apply_lidar(root, lidar):
    """Set beam layout, range, rate and noise of every lidar."""
    for sensor in sensors(root, "gpu_ray"):
        set_path(sensor, "update_rate", lidar["rate"])
        set_path(sensor, "ray/scan/horizontal/samples", lidar["samples"])
        set_path(sensor, "ray/scan/vertical/samples", lidar["beams"])
        set_path(sensor, "ray/scan/vertical/min_angle", math.radians(lidar["vertical_fov"][0]))
        set_path(sensor, "ray/scan/vertical/max_angle", math.radians(lidar["vertical_fov"][1]))
        set_path(sensor, "ray/range/min", lidar["range"][0])
        set_path(sensor, "ray/range/max", lidar["range"][1])
        set_path(sensor, "ray/noise/stddev", lidar["noise_stddev"])


def apply_gps(root, gps):
    """Set the rate of every GPS, and add position noise if any is requested."""
    for sensor in sensors(root, "navsat"):
        set_path(sensor, "update_rate", gps["rate"])
        for axis in ("horizontal", "vertical"):
            stddev = gps[f"{axis}_noise_stddev"]
            if stddev > 0:
                noise = f"navsat/position_sensing/{axis}/noise"
                set_path(sensor, f"{noise}/stddev", stddev)
                sensor.find(noise).set("type", "gaussian")


def apply_imu(root, imu):
    """Set the rate of every IMU."""
    for sensor in sensors(root, "imu"):
        set_path(sensor, "update_rate", imu["rate"])


def main():
    """Apply the parameter file to the URDF given on the command line."""
    if len(sys.argv) != 3:
        sys.exit("usage: apply_boat_params.py <boat_params.yaml> <wamv.urdf>")
    params_path, urdf_path = sys.argv[1:]

    with open(params_path) as f:
        params = yaml.safe_load(f)

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    apply_hull(root, params["hull"])
    apply_drag(root, params["drag"])
    apply_thruster(root, params["thruster"])
    apply_camera(root, params["camera"])
    apply_lidar(root, params["lidar"])
    apply_gps(root, params["gps"])
    apply_imu(root, params["imu"])
    tree.write(urdf_path, xml_declaration=True, encoding="unicode")
    print(f"apply_boat_params: wrote {params_path} into {urdf_path}")


if __name__ == "__main__":
    main()
