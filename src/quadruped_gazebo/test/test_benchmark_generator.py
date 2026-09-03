"""Tests for deterministic benchmark world generation."""

import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from quadruped_gazebo.benchmark_generator import (
    ScenarioError,
    generate_world,
    load_scenario,
    validate_scenario,
)


SCENARIO = (
    Path(__file__).parents[1] / "config" / "benchmarks" / "v0_geometry.yaml"
)


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_v0_scenario_is_valid() -> None:
    scenario = load_scenario(SCENARIO)
    assert scenario["random_seed"] == 42


def test_generation_is_identical_for_default_seed(tmp_path: Path) -> None:
    first = generate_world(SCENARIO, tmp_path / "same-output")
    first_hashes = {
        name: _digest(first[name])
        for name in ("world", "terrain", "terrain_mesh", "elevation", "manifest")
    }
    second = generate_world(SCENARIO, tmp_path / "same-output")
    second_hashes = {
        name: _digest(second[name])
        for name in ("world", "terrain", "terrain_mesh", "elevation", "manifest")
    }
    assert first["seed"] == second["seed"] == 42
    assert first_hashes == second_hashes


def test_seed_override_changes_stochastic_artifacts(tmp_path: Path) -> None:
    default = generate_world(SCENARIO, tmp_path / "default")
    changed = generate_world(SCENARIO, tmp_path / "changed", seed_override=43)
    assert _digest(default["elevation"]) != _digest(changed["elevation"])
    default_manifest = json.loads(default["manifest"].read_text())
    changed_manifest = json.loads(changed["manifest"].read_text())
    assert default_manifest["seed"] == 42
    assert changed_manifest["seed"] == 43
    assert [item["bounds"] for item in default_manifest["features"]] == [
        item["bounds"] for item in changed_manifest["features"]
    ]


def test_generated_world_is_well_formed_xml(tmp_path: Path) -> None:
    artifacts = generate_world(SCENARIO, tmp_path / "world")
    root = ET.parse(artifacts["world"]).getroot()
    assert root.tag == "sdf"
    assert root.findtext("./world/physics/dart/collision_detector") == "bullet"
    assert root.findtext("./world/scene/background") == "0.45 0.68 0.92 1.0"
    terrain = root.find("./world/model[@name='benchmark_terrain']")
    assert terrain is not None
    manifest = json.loads(artifacts["manifest"].read_text())
    terrain_z = float(terrain.findtext("pose").split()[2])
    assert terrain_z == 0.0
    collision_uri = terrain.findtext("./link/collision/geometry/mesh/uri")
    visual_uri = terrain.findtext("./link/visual/geometry/mesh/uri")
    assert collision_uri == visual_uri == artifacts["terrain_mesh"].as_uri()
    assert manifest["artifacts"]["terrain.obj"] == _digest(artifacts["terrain_mesh"])
    assert root.find("./world/model[@name='easy_bridge']") is not None

    elevation = np.load(artifacts["elevation"])
    world_size_x, world_size_y = manifest["terrain"]["size"]
    x_coords = np.linspace(-world_size_x / 2.0, world_size_x / 2.0, elevation.shape[1])
    y_coords = np.linspace(world_size_y / 2.0, -world_size_y / 2.0, elevation.shape[0])

    # Gazebo maps image row zero to positive world Y. The hard slope at y=-3
    # must therefore remain in the lower lane instead of being mirrored onto
    # the easy lane at y=+3.
    slope_x = int(np.argmin(np.abs(x_coords - (-4.7))))
    easy_y = int(np.argmin(np.abs(y_coords - 3.0)))
    hard_y = int(np.argmin(np.abs(y_coords - (-3.0))))
    assert elevation[hard_y, slope_x] > elevation[easy_y, slope_x] + 0.1

    rock = root.find("./world/model[@name='easy_rocks_00']")
    rock_pose = [float(value) for value in rock.findtext("pose").split()]
    rock_radius = float(rock.findtext("./link/collision/geometry/sphere/radius"))
    rock_x = int(np.argmin(np.abs(x_coords - rock_pose[0])))
    rock_y = int(np.argmin(np.abs(y_coords - rock_pose[1])))
    seating = rock_pose[2] - rock_radius - float(elevation[rock_y, rock_x])
    assert seating == pytest.approx(0.003, abs=1e-8)


def test_duplicate_feature_id_is_rejected() -> None:
    scenario = load_scenario(SCENARIO)
    broken = copy.deepcopy(scenario)
    broken["lanes"][1]["features"][0]["id"] = "easy_rough"
    with pytest.raises(ScenarioError, match="duplicate feature id"):
        validate_scenario(broken)


def test_feature_on_clear_pad_is_rejected() -> None:
    scenario = load_scenario(SCENARIO)
    broken = copy.deepcopy(scenario)
    broken["lanes"][0]["features"][0]["center"] = [-8.0, 3.0]
    with pytest.raises(ScenarioError, match="reserved clear pad"):
        validate_scenario(broken)
