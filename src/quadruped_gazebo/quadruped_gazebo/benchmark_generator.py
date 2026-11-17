"""Deterministic Gazebo world generation for simulation benchmarks."""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import random
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml


GENERATOR_VERSION = 2
SUPPORTED_SCHEMA_VERSION = 1
HEIGHTFIELD_TYPES = {"rough_patch", "slope", "ditch"}
MODEL_TYPES = {"rock_field", "rubble_field", "bridge", "passage"}
SUPPORTED_TYPES = HEIGHTFIELD_TYPES | MODEL_TYPES


class ScenarioError(ValueError):
    """Raised when a benchmark scenario is invalid."""


def load_scenario(path: str | Path) -> dict[str, Any]:
    """Load and validate a benchmark scenario YAML file."""
    scenario_path = Path(path).resolve()
    with scenario_path.open("r", encoding="utf-8") as stream:
        scenario = yaml.safe_load(stream)
    if not isinstance(scenario, dict):
        raise ScenarioError("scenario root must be a mapping")
    validate_scenario(scenario)
    return scenario


def validate_scenario(scenario: dict[str, Any]) -> None:
    """Validate fields that affect deterministic geometry generation."""
    if scenario.get("schema_version") != SUPPORTED_SCHEMA_VERSION:
        raise ScenarioError(
            f"schema_version must be {SUPPORTED_SCHEMA_VERSION}"
        )
    if not isinstance(scenario.get("name"), str) or not scenario["name"]:
        raise ScenarioError("name must be a non-empty string")
    seed = scenario.get("random_seed")
    if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
        raise ScenarioError("random_seed must be a non-negative integer")

    world = _required_mapping(scenario, "world")
    world_size = _positive_pair(world.get("size"), "world.size")
    samples = world.get("height_samples")
    if (
        not isinstance(samples, list)
        or len(samples) != 2
        or any(not isinstance(value, int) for value in samples)
        or samples[0] != samples[1]
        or not _is_power_of_two(samples[0] - 1)
    ):
        raise ScenarioError(
            "world.height_samples must be equal 2^n+1 integer dimensions"
        )
    _positive_number(world.get("friction"), "world.friction")
    _non_negative_number(
        world.get("static_model_clearance"), "world.static_model_clearance"
    )
    _rgba_color(world.get("sky_color"), "world.sky_color")

    terrain = _required_mapping(scenario, "terrain")
    _non_negative_number(
        terrain.get("base_roughness_amplitude"),
        "terrain.base_roughness_amplitude",
    )
    _positive_number(
        terrain.get("correlation_length"), "terrain.correlation_length"
    )

    spawn = _required_mapping(scenario, "spawn")
    position = spawn.get("position")
    if not isinstance(position, list) or len(position) != 3:
        raise ScenarioError("spawn.position must contain [x, y, z]")
    for index, value in enumerate(position):
        _number(value, f"spawn.position[{index}]")
    _number(spawn.get("yaw"), "spawn.yaw")

    clear_pads = scenario.get("clear_pads")
    if not isinstance(clear_pads, list) or not clear_pads:
        raise ScenarioError("clear_pads must be a non-empty list")
    pad_ids: set[str] = set()
    for pad in clear_pads:
        if not isinstance(pad, dict):
            raise ScenarioError("each clear pad must be a mapping")
        pad_id = _unique_id(pad, pad_ids, "clear pad")
        _validate_bounds(
            pad.get("center"), pad.get("size"), world_size, f"clear pad {pad_id}"
        )

    lanes = scenario.get("lanes")
    if not isinstance(lanes, list) or not lanes:
        raise ScenarioError("lanes must be a non-empty list")

    lane_ids: set[str] = set()
    feature_ids: set[str] = set()
    features: list[dict[str, Any]] = []
    for lane in lanes:
        if not isinstance(lane, dict):
            raise ScenarioError("each lane must be a mapping")
        lane_id = _unique_id(lane, lane_ids, "lane")
        _number(lane.get("center_y"), f"lane {lane_id}.center_y")
        if lane.get("difficulty") not in {"easy", "moderate", "hard"}:
            raise ScenarioError(
                f"lane {lane_id}.difficulty must be easy, moderate, or hard"
            )
        finish = _required_mapping(lane, "finish_region")
        _validate_bounds(
            finish.get("center"),
            finish.get("size"),
            world_size,
            f"lane {lane_id}.finish_region",
        )
        lane_features = lane.get("features")
        if not isinstance(lane_features, list) or not lane_features:
            raise ScenarioError(f"lane {lane_id}.features must be non-empty")
        for feature in lane_features:
            if not isinstance(feature, dict):
                raise ScenarioError(f"lane {lane_id} feature must be a mapping")
            feature_id = _unique_id(feature, feature_ids, "feature")
            feature_type = feature.get("type")
            if feature_type not in SUPPORTED_TYPES:
                raise ScenarioError(
                    f"feature {feature_id} has unsupported type {feature_type!r}"
                )
            _validate_bounds(
                feature.get("center"),
                feature.get("size"),
                world_size,
                f"feature {feature_id}",
            )
            _validate_feature_parameters(feature)
            feature["_lane_id"] = lane_id
            features.append(feature)

    for feature in features:
        allowed = feature.get("allow_overlap_with", [])
        if not isinstance(allowed, list) or any(
            not isinstance(value, str) for value in allowed
        ):
            raise ScenarioError(
                f"feature {feature['id']}.allow_overlap_with must be a list of IDs"
            )
        unknown = set(allowed) - feature_ids
        if unknown:
            raise ScenarioError(
                f"feature {feature['id']} allows unknown overlap IDs: {sorted(unknown)}"
            )

    for index, first in enumerate(features):
        for second in features[index + 1 :]:
            if not _rectangles_overlap(first, second):
                continue
            first_allowed = second["id"] in first.get("allow_overlap_with", [])
            second_allowed = first["id"] in second.get("allow_overlap_with", [])
            if not (first_allowed and second_allowed):
                raise ScenarioError(
                    f"features {first['id']} and {second['id']} overlap without "
                    "mutual allow_overlap_with entries"
                )

    for pad in clear_pads:
        for feature in features:
            if _rectangles_overlap(pad, feature):
                raise ScenarioError(
                    f"feature {feature['id']} overlaps reserved clear pad {pad['id']}"
                )

    # Validation adds temporary lane IDs for clearer internal processing. Do not
    # let these become part of the caller's source-of-truth data.
    for feature in features:
        feature.pop("_lane_id", None)


def generate_world(
    scenario_path: str | Path,
    output_dir: str | Path,
    seed_override: int | None = None,
) -> dict[str, Any]:
    """Generate deterministic benchmark artifacts and return their paths."""
    scenario_path = Path(scenario_path).resolve()
    scenario = load_scenario(scenario_path)
    seed = scenario["random_seed"] if seed_override is None else seed_override
    if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
        raise ScenarioError("seed override must be a non-negative integer")

    output_path = Path(output_dir).resolve()
    output_path.mkdir(parents=True, exist_ok=True)
    elevation, x_coords, y_coords = _generate_elevation(scenario, seed)

    elevation_path = output_path / "elevation.npy"
    np.save(elevation_path, elevation.astype("<f4"), allow_pickle=False)

    minimum = float(elevation.min())
    maximum = float(elevation.max())
    vertical_range = max(maximum - minimum, 0.001)
    normalized = np.rint((elevation - minimum) / vertical_range * 65535.0)
    heightmap = np.clip(normalized, 0, 65535).astype(np.uint16)
    terrain_path = output_path / "terrain.png"
    Image.fromarray(heightmap, mode="I;16").save(terrain_path, compress_level=9)

    # Gazebo has had backend-specific mismatches between rendered heightmaps
    # and heightfield collisions. Use one indexed mesh for both so the depth
    # camera and physics engine necessarily consume the same surface. Keep the
    # PNG as a portable inspection artifact and elevation.npy as numeric truth.
    mesh_path = output_path / "terrain.obj"
    _write_terrain_mesh(mesh_path, elevation, x_coords, y_coords)

    model_records: list[dict[str, Any]] = []
    world_root = _build_world_sdf(
        scenario,
        seed,
        elevation,
        x_coords,
        y_coords,
        mesh_path,
        model_records,
    )
    ET.indent(world_root, space="  ")
    world_path = output_path / "world.sdf"
    ET.ElementTree(world_root).write(
        world_path, encoding="utf-8", xml_declaration=True
    )

    manifest = _build_manifest(
        scenario,
        scenario_path,
        seed,
        elevation,
        x_coords,
        y_coords,
        world_path,
        terrain_path,
        mesh_path,
        elevation_path,
        model_records,
    )
    manifest_path = output_path / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    return {
        "seed": seed,
        "world": world_path,
        "terrain": terrain_path,
        "terrain_mesh": mesh_path,
        "elevation": elevation_path,
        "manifest": manifest_path,
        "spawn": copy.deepcopy(scenario["spawn"]),
    }


def _generate_elevation(
    scenario: dict[str, Any], seed: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    world = scenario["world"]
    size_x, size_y = (float(value) for value in world["size"])
    sample_x, sample_y = world["height_samples"]
    x_coords = np.linspace(-size_x / 2.0, size_x / 2.0, sample_x)
    # Image row zero is the top of the heightmap, which Gazebo maps to +Y.
    # Keep the numeric elevation array in that same order so generated static
    # models sample the surface that is actually rendered beneath them.
    y_coords = np.linspace(size_y / 2.0, -size_y / 2.0, sample_y)
    grid_x, grid_y = np.meshgrid(x_coords, y_coords)
    base = float(world.get("base_elevation", 0.0))
    terrain = scenario["terrain"]
    elevation = np.full((sample_y, sample_x), base, dtype=np.float64)
    elevation += _smooth_noise(
        grid_x,
        grid_y,
        float(terrain["base_roughness_amplitude"]),
        float(terrain["correlation_length"]),
        _feature_rng(seed, "base_terrain"),
    )

    for lane in scenario["lanes"]:
        for feature in lane["features"]:
            feature_type = feature["type"]
            if feature_type == "rough_patch":
                mask = _rectangular_taper(grid_x, grid_y, feature)
                local_noise = _smooth_noise(
                    grid_x,
                    grid_y,
                    float(feature["amplitude"]),
                    float(feature["correlation_length"]),
                    _feature_rng(seed, feature["id"]),
                )
                elevation += mask * local_noise
            elif feature_type == "slope":
                center_x, center_y = feature["center"]
                length, width = feature["size"]
                longitudinal = np.clip(
                    1.0 - np.abs(grid_x - center_x) / (length / 2.0), 0.0, 1.0
                )
                lateral = _cosine_window(grid_y - center_y, width / 2.0)
                peak = math.tan(float(feature["slope_rad"])) * length / 2.0
                elevation += peak * longitudinal * lateral
            elif feature_type == "ditch":
                elevation -= float(feature["depth"]) * _rectangular_taper(
                    grid_x, grid_y, feature
                )

    for pad in scenario["clear_pads"]:
        mask = _rectangular_taper(grid_x, grid_y, pad, edge_fraction=0.20)
        elevation = elevation * (1.0 - mask) + base * mask

    return elevation.astype(np.float32), x_coords, y_coords


def _write_terrain_mesh(
    path: Path,
    elevation: np.ndarray,
    x_coords: np.ndarray,
    y_coords: np.ndarray,
) -> None:
    """Write the elevation grid as a deterministic, upward-facing OBJ mesh."""
    rows, columns = elevation.shape
    lines = ["# Visual Paws deterministic benchmark terrain\n"]
    for row, y in enumerate(y_coords):
        for column, x in enumerate(x_coords):
            lines.append(
                f"v {float(x):.9f} {float(y):.9f} "
                f"{float(elevation[row, column]):.9f}\n"
            )

    dz_dy, dz_dx = np.gradient(elevation, y_coords, x_coords)
    normal_x = -dz_dx
    normal_y = -dz_dy
    normal_z = np.ones_like(elevation)
    magnitudes = np.sqrt(normal_x**2 + normal_y**2 + normal_z**2)
    for row in range(rows):
        for column in range(columns):
            lines.append(
                f"vn {float(normal_x[row, column] / magnitudes[row, column]):.9f} "
                f"{float(normal_y[row, column] / magnitudes[row, column]):.9f} "
                f"{float(normal_z[row, column] / magnitudes[row, column]):.9f}\n"
            )

    # OBJ indices are one-based. Row zero maps to +Y, so this winding gives
    # both triangles an upward-facing normal.
    for row in range(rows - 1):
        for column in range(columns - 1):
            top_left = row * columns + column + 1
            top_right = top_left + 1
            bottom_left = top_left + columns
            bottom_right = bottom_left + 1
            lines.append(
                f"f {top_left}//{top_left} {bottom_left}//{bottom_left} "
                f"{top_right}//{top_right}\n"
            )
            lines.append(
                f"f {top_right}//{top_right} {bottom_left}//{bottom_left} "
                f"{bottom_right}//{bottom_right}\n"
            )

    path.write_text("".join(lines), encoding="ascii")


def _build_world_sdf(
    scenario: dict[str, Any],
    seed: int,
    elevation: np.ndarray,
    x_coords: np.ndarray,
    y_coords: np.ndarray,
    mesh_path: Path,
    model_records: list[dict[str, Any]],
) -> ET.Element:
    sdf = ET.Element("sdf", {"version": "1.10"})
    world = ET.SubElement(sdf, "world", {"name": scenario["name"]})

    physics = ET.SubElement(world, "physics", {"name": "1ms", "type": "ignored"})
    _text(physics, "max_step_size", "0.001")
    _text(physics, "real_time_factor", "1.0")
    _text(physics, "real_time_update_rate", "1000")
    # Bullet provides robust static triangle-mesh contacts for the terrain.
    dart = ET.SubElement(physics, "dart")
    _text(dart, "collision_detector", "bullet")
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
        ("gz-sim-contact-system", "gz::sim::systems::Contact"),
        ("gz-sim-imu-system", "gz::sim::systems::Imu"),
        ("gz-sim-forcetorque-system", "gz::sim::systems::ForceTorque"),
    ):
        ET.SubElement(world, "plugin", {"filename": filename, "name": name})
    sensors = ET.SubElement(
        world,
        "plugin",
        {
            "filename": "gz-sim-sensors-system",
            "name": "gz::sim::systems::Sensors",
        },
    )
    _text(sensors, "render_engine", "ogre2")

    scene = ET.SubElement(world, "scene")
    _text(
        scene,
        "background",
        " ".join(str(value) for value in scenario["world"]["sky_color"]),
    )
    _text(scene, "ambient", "0.4 0.4 0.4 1")
    _text(scene, "shadows", "true")

    light = ET.SubElement(world, "light", {"type": "directional", "name": "sun"})
    _text(light, "cast_shadows", "true")
    _text(light, "pose", "0 0 10 0 0 0")
    _text(light, "diffuse", "0.8 0.8 0.8 1")
    _text(light, "specular", "0.2 0.2 0.2 1")
    _text(light, "direction", "-0.5 0.1 -0.9")

    terrain_model = ET.SubElement(world, "model", {"name": "benchmark_terrain"})
    _text(terrain_model, "static", "true")
    _text(terrain_model, "pose", "0 0 0 0 0 0")
    terrain_link = ET.SubElement(terrain_model, "link", {"name": "terrain_link"})
    for element_name, element_id in (
        ("collision", "terrain_collision"),
        ("visual", "terrain_visual"),
    ):
        element = ET.SubElement(terrain_link, element_name, {"name": element_id})
        geometry = ET.SubElement(element, "geometry")
        mesh = ET.SubElement(geometry, "mesh")
        _text(mesh, "uri", mesh_path.as_uri())
        _text(mesh, "scale", "1 1 1")
        if element_name == "visual":
            material = ET.SubElement(element, "material")
            _text(material, "ambient", "0.44 0.40 0.33 1")
            _text(material, "diffuse", "0.44 0.40 0.33 1")
        if element_name == "collision":
            surface = ET.SubElement(element, "surface")
            friction = ET.SubElement(surface, "friction")
            ode = ET.SubElement(friction, "ode")
            coefficient = scenario["world"]["friction"]
            _text(ode, "mu", coefficient)
            _text(ode, "mu2", coefficient)

    for lane in scenario["lanes"]:
        for feature in lane["features"]:
            feature_type = feature["type"]
            static_clearance = float(scenario["world"]["static_model_clearance"])
            if feature_type == "rock_field":
                _add_rocks(
                    world,
                    feature,
                    lane,
                    seed,
                    elevation,
                    x_coords,
                    y_coords,
                    static_clearance,
                    model_records,
                )
            elif feature_type == "rubble_field":
                _add_rubble(
                    world,
                    feature,
                    lane,
                    seed,
                    elevation,
                    x_coords,
                    y_coords,
                    static_clearance,
                    model_records,
                )
            elif feature_type == "bridge":
                center_x, center_y = feature["center"]
                length, width = feature["size"]
                ground = _sample_elevation(
                    elevation, x_coords, y_coords, center_x - length / 2.0, center_y
                )
                thickness = float(feature["thickness"])
                z = ground + float(feature["clearance"]) + thickness / 2.0
                _add_box_model(
                    world,
                    feature["id"],
                    [center_x, center_y, z, 0.0, 0.0, 0.0],
                    [length, width, thickness],
                    scenario["world"]["friction"],
                    "0.45 0.32 0.20 1",
                )
                model_records.append(
                    _model_record(feature, lane, feature["id"], [center_x, center_y, z])
                )
            elif feature_type == "passage":
                _add_passage(
                    world,
                    feature,
                    lane,
                    scenario["world"]["friction"],
                    static_clearance,
                    elevation,
                    x_coords,
                    y_coords,
                    model_records,
                )

    return sdf


def _add_rocks(
    world: ET.Element,
    feature: dict[str, Any],
    lane: dict[str, Any],
    seed: int,
    elevation: np.ndarray,
    x_coords: np.ndarray,
    y_coords: np.ndarray,
    clearance: float,
    model_records: list[dict[str, Any]],
) -> None:
    rng = _feature_rng(seed, feature["id"])
    center_x, center_y = feature["center"]
    length, width = feature["size"]
    low, high = feature["height_range"]
    for index in range(feature["count"]):
        x = rng.uniform(center_x - length * 0.42, center_x + length * 0.42)
        y = rng.uniform(center_y - width * 0.42, center_y + width * 0.42)
        height = rng.uniform(low, high)
        radius = height / 2.0
        ground = _sample_elevation(elevation, x_coords, y_coords, x, y)
        ground += clearance
        name = f"{feature['id']}_{index:02d}"
        model = ET.SubElement(world, "model", {"name": name})
        _text(model, "static", "true")
        _text(model, "pose", f"{x:.9f} {y:.9f} {ground + radius:.9f} 0 0 0")
        link = ET.SubElement(model, "link", {"name": "link"})
        _add_sphere_geometry(link, radius, "0.32 0.30 0.27 1")
        model_records.append(_model_record(feature, lane, name, [x, y, ground + radius]))


def _add_rubble(
    world: ET.Element,
    feature: dict[str, Any],
    lane: dict[str, Any],
    seed: int,
    elevation: np.ndarray,
    x_coords: np.ndarray,
    y_coords: np.ndarray,
    clearance: float,
    model_records: list[dict[str, Any]],
) -> None:
    rng = _feature_rng(seed, feature["id"])
    center_x, center_y = feature["center"]
    length, width = feature["size"]
    low, high = feature["block_size_range"]
    for index in range(feature["count"]):
        x = rng.uniform(center_x - length * 0.42, center_x + length * 0.42)
        y = rng.uniform(center_y - width * 0.42, center_y + width * 0.42)
        size_x = rng.uniform(low, high) * rng.uniform(1.2, 2.0)
        size_y = rng.uniform(low, high) * rng.uniform(0.8, 1.4)
        size_z = rng.uniform(low, high)
        yaw = rng.uniform(-math.pi, math.pi)
        ground = _sample_elevation(elevation, x_coords, y_coords, x, y)
        ground += clearance
        name = f"{feature['id']}_{index:02d}"
        _add_box_model(
            world,
            name,
            [x, y, ground + size_z / 2.0, 0.0, 0.0, yaw],
            [size_x, size_y, size_z],
            0.9,
            "0.48 0.48 0.46 1",
        )
        model_records.append(
            _model_record(feature, lane, name, [x, y, ground + size_z / 2.0])
        )


def _add_passage(
    world: ET.Element,
    feature: dict[str, Any],
    lane: dict[str, Any],
    friction: float,
    clearance: float,
    elevation: np.ndarray,
    x_coords: np.ndarray,
    y_coords: np.ndarray,
    model_records: list[dict[str, Any]],
) -> None:
    center_x, center_y = feature["center"]
    length, _ = feature["size"]
    opening = float(feature["passage_width"])
    thickness = float(feature["wall_thickness"])
    height = float(feature["wall_height"])
    for side, sign in (("left", 1.0), ("right", -1.0)):
        y = center_y + sign * (opening / 2.0 + thickness / 2.0)
        ground = _sample_elevation(elevation, x_coords, y_coords, center_x, y)
        ground += clearance
        name = f"{feature['id']}_{side}"
        z = ground + height / 2.0
        _add_box_model(
            world,
            name,
            [center_x, y, z, 0.0, 0.0, 0.0],
            [length, thickness, height],
            friction,
            "0.55 0.50 0.43 1",
        )
        model_records.append(_model_record(feature, lane, name, [center_x, y, z]))


def _add_box_model(
    world: ET.Element,
    name: str,
    pose: list[float],
    size: list[float],
    friction: float,
    color: str,
) -> None:
    model = ET.SubElement(world, "model", {"name": name})
    _text(model, "static", "true")
    _text(model, "pose", " ".join(f"{value:.9f}" for value in pose))
    link = ET.SubElement(model, "link", {"name": "link"})
    for element_name in ("collision", "visual"):
        element = ET.SubElement(link, element_name, {"name": element_name})
        geometry = ET.SubElement(element, "geometry")
        box = ET.SubElement(geometry, "box")
        _text(box, "size", " ".join(f"{value:.9f}" for value in size))
        if element_name == "collision":
            surface = ET.SubElement(element, "surface")
            friction_element = ET.SubElement(surface, "friction")
            ode = ET.SubElement(friction_element, "ode")
            _text(ode, "mu", friction)
            _text(ode, "mu2", friction)
        else:
            material = ET.SubElement(element, "material")
            _text(material, "ambient", color)
            _text(material, "diffuse", color)


def _add_sphere_geometry(link: ET.Element, radius: float, color: str) -> None:
    for element_name in ("collision", "visual"):
        element = ET.SubElement(link, element_name, {"name": element_name})
        geometry = ET.SubElement(element, "geometry")
        sphere = ET.SubElement(geometry, "sphere")
        _text(sphere, "radius", f"{radius:.9f}")
        if element_name == "visual":
            material = ET.SubElement(element, "material")
            _text(material, "ambient", color)
            _text(material, "diffuse", color)


def _build_manifest(
    scenario: dict[str, Any],
    scenario_path: Path,
    seed: int,
    elevation: np.ndarray,
    x_coords: np.ndarray,
    y_coords: np.ndarray,
    world_path: Path,
    terrain_path: Path,
    mesh_path: Path,
    elevation_path: Path,
    model_records: list[dict[str, Any]],
) -> dict[str, Any]:
    features = []
    for lane in scenario["lanes"]:
        for feature in lane["features"]:
            features.append(
                {
                    "id": feature["id"],
                    "type": feature["type"],
                    "lane": lane["id"],
                    "difficulty": lane["difficulty"],
                    "center": feature["center"],
                    "size": feature["size"],
                    "bounds": _bounds(feature),
                    "parameters": {
                        key: value
                        for key, value in feature.items()
                        if key
                        not in {
                            "id",
                            "type",
                            "center",
                            "size",
                            "allow_overlap_with",
                        }
                    },
                }
            )
    return {
        "schema_version": scenario["schema_version"],
        "generator_version": GENERATOR_VERSION,
        "scenario": scenario["name"],
        "seed": seed,
        "scenario_sha256": _sha256(scenario_path),
        "artifacts": {
            "world.sdf": _sha256(world_path),
            "terrain.png": _sha256(terrain_path),
            "terrain.obj": _sha256(mesh_path),
            "elevation.npy": _sha256(elevation_path),
        },
        "terrain": {
            "shape": list(elevation.shape),
            "origin": [float(x_coords[0]), float(y_coords[-1])],
            "image_row_order": "positive_y_to_negative_y",
            "size": scenario["world"]["size"],
            "minimum": float(elevation.min()),
            "maximum": float(elevation.max()),
        },
        "spawn": scenario["spawn"],
        "finish_regions": {
            lane["id"]: lane["finish_region"] for lane in scenario["lanes"]
        },
        "features": features,
        "generated_models": model_records,
    }


def _validate_feature_parameters(feature: dict[str, Any]) -> None:
    feature_id = feature["id"]
    feature_type = feature["type"]
    if feature_type == "rough_patch":
        _positive_number(feature.get("amplitude"), f"feature {feature_id}.amplitude")
        _positive_number(
            feature.get("correlation_length"),
            f"feature {feature_id}.correlation_length",
        )
    elif feature_type == "slope":
        slope = _positive_number(
            feature.get("slope_rad"), f"feature {feature_id}.slope_rad"
        )
        if slope >= math.pi / 2.0:
            raise ScenarioError(f"feature {feature_id}.slope_rad must be below pi/2")
    elif feature_type == "ditch":
        _positive_number(feature.get("depth"), f"feature {feature_id}.depth")
    elif feature_type == "rock_field":
        _positive_integer(feature.get("count"), f"feature {feature_id}.count")
        _positive_range(
            feature.get("height_range"), f"feature {feature_id}.height_range"
        )
    elif feature_type == "rubble_field":
        _positive_integer(feature.get("count"), f"feature {feature_id}.count")
        _positive_range(
            feature.get("block_size_range"),
            f"feature {feature_id}.block_size_range",
        )
    elif feature_type == "bridge":
        _positive_number(
            feature.get("thickness"), f"feature {feature_id}.thickness"
        )
        _non_negative_number(
            feature.get("clearance"), f"feature {feature_id}.clearance"
        )
    elif feature_type == "passage":
        opening = _positive_number(
            feature.get("passage_width"),
            f"feature {feature_id}.passage_width",
        )
        _positive_number(
            feature.get("wall_thickness"),
            f"feature {feature_id}.wall_thickness",
        )
        _positive_number(
            feature.get("wall_height"), f"feature {feature_id}.wall_height"
        )
        if opening >= feature["size"][1]:
            raise ScenarioError(
                f"feature {feature_id}.passage_width must fit inside feature size"
            )


def _smooth_noise(
    grid_x: np.ndarray,
    grid_y: np.ndarray,
    amplitude: float,
    correlation_length: float,
    rng: random.Random,
) -> np.ndarray:
    if amplitude == 0.0:
        return np.zeros_like(grid_x)
    phase_x = rng.uniform(-math.pi, math.pi)
    phase_y = rng.uniform(-math.pi, math.pi)
    phase_mix = rng.uniform(-math.pi, math.pi)
    wavelength = max(correlation_length, 1e-6)
    noise = (
        np.sin(2.0 * math.pi * grid_x / wavelength + phase_x)
        + np.sin(2.0 * math.pi * grid_y / (wavelength * 1.37) + phase_y)
        + 0.5
        * np.sin(
            2.0 * math.pi * (grid_x + grid_y) / (wavelength * 2.11) + phase_mix
        )
    )
    return amplitude * noise / 2.5


def _rectangular_taper(
    grid_x: np.ndarray,
    grid_y: np.ndarray,
    rectangle: dict[str, Any],
    edge_fraction: float = 0.15,
) -> np.ndarray:
    center_x, center_y = rectangle["center"]
    length, width = rectangle["size"]
    x_ratio = np.abs(grid_x - center_x) / (length / 2.0)
    y_ratio = np.abs(grid_y - center_y) / (width / 2.0)
    inside = np.maximum(x_ratio, y_ratio)
    edge_start = 1.0 - edge_fraction
    scaled = np.clip((1.0 - inside) / max(edge_fraction, 1e-6), 0.0, 1.0)
    return np.where(inside <= edge_start, 1.0, 0.5 - 0.5 * np.cos(math.pi * scaled))


def _cosine_window(offset: np.ndarray, half_width: float) -> np.ndarray:
    normalized = np.clip(np.abs(offset) / half_width, 0.0, 1.0)
    return 0.5 + 0.5 * np.cos(math.pi * normalized)


def _feature_rng(seed: int, feature_id: str) -> random.Random:
    digest = hashlib.sha256(f"v{GENERATOR_VERSION}:{seed}:{feature_id}".encode()).digest()
    return random.Random(int.from_bytes(digest[:8], "big"))


def _sample_elevation(
    elevation: np.ndarray,
    x_coords: np.ndarray,
    y_coords: np.ndarray,
    x: float,
    y: float,
) -> float:
    x_index = int(np.argmin(np.abs(x_coords - x)))
    y_index = int(np.argmin(np.abs(y_coords - y)))
    return float(elevation[y_index, x_index])


def _model_record(
    feature: dict[str, Any],
    lane: dict[str, Any],
    name: str,
    position: list[float],
) -> dict[str, Any]:
    return {
        "name": name,
        "feature_id": feature["id"],
        "lane": lane["id"],
        "difficulty": lane["difficulty"],
        "position": [float(value) for value in position],
    }


def _required_mapping(mapping: dict[str, Any], key: str) -> dict[str, Any]:
    value = mapping.get(key)
    if not isinstance(value, dict):
        raise ScenarioError(f"{key} must be a mapping")
    return value


def _unique_id(item: dict[str, Any], known: set[str], kind: str) -> str:
    item_id = item.get("id")
    if not isinstance(item_id, str) or not item_id:
        raise ScenarioError(f"each {kind} must have a non-empty string id")
    if item_id in known:
        raise ScenarioError(f"duplicate {kind} id: {item_id}")
    known.add(item_id)
    return item_id


def _validate_bounds(
    center: Any, size: Any, world_size: list[float], label: str
) -> None:
    center_pair = _number_pair(center, f"{label}.center")
    size_pair = _positive_pair(size, f"{label}.size")
    if abs(center_pair[0]) + size_pair[0] / 2.0 > world_size[0] / 2.0:
        raise ScenarioError(f"{label} exceeds world x bounds")
    if abs(center_pair[1]) + size_pair[1] / 2.0 > world_size[1] / 2.0:
        raise ScenarioError(f"{label} exceeds world y bounds")


def _rectangles_overlap(first: dict[str, Any], second: dict[str, Any]) -> bool:
    first_bounds = _bounds(first)
    second_bounds = _bounds(second)
    return not (
        first_bounds[1] <= second_bounds[0]
        or second_bounds[1] <= first_bounds[0]
        or first_bounds[3] <= second_bounds[2]
        or second_bounds[3] <= first_bounds[2]
    )


def _bounds(rectangle: dict[str, Any]) -> list[float]:
    center_x, center_y = rectangle["center"]
    length, width = rectangle["size"]
    return [
        center_x - length / 2.0,
        center_x + length / 2.0,
        center_y - width / 2.0,
        center_y + width / 2.0,
    ]


def _positive_range(value: Any, label: str) -> list[float]:
    pair = _positive_pair(value, label)
    if pair[0] > pair[1]:
        raise ScenarioError(f"{label} minimum must not exceed maximum")
    return pair


def _rgba_color(value: Any, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 4:
        raise ScenarioError(f"{label} must contain [red, green, blue, alpha]")
    color = [
        _number(component, f"{label}[{index}]")
        for index, component in enumerate(value)
    ]
    if any(component < 0.0 or component > 1.0 for component in color):
        raise ScenarioError(f"{label} components must be between 0 and 1")
    return color


def _number_pair(value: Any, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ScenarioError(f"{label} must contain two numbers")
    return [_number(item, f"{label}[{index}]") for index, item in enumerate(value)]


def _positive_pair(value: Any, label: str) -> list[float]:
    pair = _number_pair(value, label)
    if any(item <= 0.0 for item in pair):
        raise ScenarioError(f"{label} values must be positive")
    return pair


def _number(value: Any, label: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ScenarioError(f"{label} must be a number")
    if not math.isfinite(float(value)):
        raise ScenarioError(f"{label} must be finite")
    return float(value)


def _positive_number(value: Any, label: str) -> float:
    number = _number(value, label)
    if number <= 0.0:
        raise ScenarioError(f"{label} must be positive")
    return number


def _non_negative_number(value: Any, label: str) -> float:
    number = _number(value, label)
    if number < 0.0:
        raise ScenarioError(f"{label} must be non-negative")
    return number


def _positive_integer(value: Any, label: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ScenarioError(f"{label} must be a positive integer")
    return value


def _is_power_of_two(value: int) -> bool:
    return value > 0 and value & (value - 1) == 0


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _text(parent: ET.Element, tag: str, value: Any) -> ET.Element:
    element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element
