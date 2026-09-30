#!/usr/bin/env python3
"""
Reads model.sdf.template, substitutes ROBOT_NAME, and writes out a
concrete SDF file for one robot instance. Called once per robot by the
launch file before spawning.
"""
import sys
from pathlib import Path


def generate(template_path: str, robot_name: str, output_path: str):
    template = Path(template_path).read_text()
    concrete = template.replace('ROBOT_NAME', robot_name)
    Path(output_path).write_text(concrete)


if __name__ == '__main__':
    if len(sys.argv) != 4:
        print('Usage: generate_robot_sdf.py <template_path> <robot_name> <output_path>')
        sys.exit(1)
    generate(sys.argv[1], sys.argv[2], sys.argv[3])
