#!/usr/bin/env python3
"""Generate deterministic Gazebo benchmark artifacts from a scenario YAML."""

import argparse
import json

from quadruped_gazebo.benchmark_generator import generate_world


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Override the scenario random_seed; omitted uses the YAML value",
    )
    args = parser.parse_args()
    artifacts = generate_world(args.scenario, args.output_dir, args.seed)
    print(
        json.dumps(
            {key: str(value) for key, value in artifacts.items() if key != "spawn"},
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
