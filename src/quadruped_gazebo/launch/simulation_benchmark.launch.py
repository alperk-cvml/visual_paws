"""Generate and launch a deterministic Visual Paws simulation benchmark."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from quadruped_gazebo.benchmark_generator import generate_world, load_scenario


def _resolve_scenario(value: str, package_share: Path) -> Path:
    candidate = Path(value)
    if candidate.is_absolute():
        return candidate
    if candidate.suffix not in {".yaml", ".yml"}:
        candidate = candidate.with_suffix(".yaml")
    return package_share / "config" / "benchmarks" / candidate


def _generate_and_include(context, package_share: Path):
    scenario_path = _resolve_scenario(
        LaunchConfiguration("scenario").perform(context), package_share
    )
    scenario = load_scenario(scenario_path)

    seed_value = LaunchConfiguration("seed").perform(context).strip()
    seed_override = None if seed_value in {"", "config"} else int(seed_value)
    selected_seed = scenario["random_seed"] if seed_override is None else seed_override
    output_root = Path(LaunchConfiguration("output_dir").perform(context)).resolve()
    run_dir = output_root / scenario["name"] / str(selected_seed)
    artifacts = generate_world(scenario_path, run_dir, seed_override)

    spawn = artifacts["spawn"]
    x, y, z = spawn["position"]
    base_launch = package_share / "launch" / "gazebo.launch.py"
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(base_launch)),
        launch_arguments={
            "world": str(artifacts["world"]),
            "x_pose": str(x),
            "y_pose": str(y),
            "z_pose": str(z),
            "yaw_pose": str(spawn["yaw"]),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "localization_mode": LaunchConfiguration("localization_mode"),
        }.items(),
    )
    return [
        LogInfo(
            msg=(
                f"Generated benchmark '{scenario['name']}' with seed "
                f"{artifacts['seed']} at {run_dir}"
            )
        ),
        include,
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("quadruped_gazebo"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="v0_geometry",
                description="Scenario name or absolute YAML path",
            ),
            DeclareLaunchArgument(
                "seed",
                default_value="config",
                description=(
                    "Integer random seed override; 'config' uses the scenario YAML"
                ),
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="/tmp/visual_paws/simulation_benchmark",
                description="Root directory for generated benchmark artifacts",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "localization_mode",
                default_value="ground_truth",
                choices=["ground_truth", "slam"],
            ),
            OpaqueFunction(function=_generate_and_include, args=[package_share]),
        ]
    )
