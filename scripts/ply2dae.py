import argparse
import subprocess
import sys

def install_and_import(package):
    try:
        __import__(package)
    except ImportError:
        print(f"Installing missing package: {package}")
        subprocess.check_call([sys.executable, "-m", "pip", "install", package])

install_and_import("pycollada")
install_and_import("trimesh")

import trimesh

def convert_ply_to_dae(input_file, output_file):
    try:
        mesh = trimesh.load(input_file)
        mesh.export(output_file)
        print(f"Converted '{input_file}' to '{output_file}' successfully.")
    except Exception as e:
        print(f"Failed to convert '{input_file}' to '{output_file}': {e}")
        sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert .ply to .dae")
    parser.add_argument("input", help="Input .ply file path")
    parser.add_argument("output", help="Output .dae file path")

    args = parser.parse_args()
    convert_ply_to_dae(args.input, args.output)
