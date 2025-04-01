#!/bin/bash

TARGET_MAPS_DIR="/path/to/mesh_navigation_tutorials/mesh_navigation_tutorials/maps"
TARGET_SIM_DIR="mesh_navigation_tutorials/mesh_navigation_tutorials_sim"
MODEL_DIR="$TARGET_SIM_DIR/models/hsfd"
MESHES_DIR="$MODEL_DIR/meshes"

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 /full/path/to/mesh.ply"
  exit 1
fi

SOURCE_PLY="$1"

if [ ! -f "$SOURCE_PLY" ]; then
  echo "Error: .ply file not found: $SOURCE_PLY"
  exit 1
fi

SOURCE_DIR=$(dirname "$SOURCE_PLY")
BASE_NAME=$(basename "$SOURCE_PLY" .ply)
SOURCE_DAE="$SOURCE_DIR/$BASE_NAME.dae"

if [ ! -f "$SOURCE_DAE" ]; then
  echo "No .dae file found, converting using ply2dae.py..."
  
  if ! python3 ply2dae.py "$SOURCE_PLY" "$SOURCE_DAE"; then
    echo "Error: ply2dae.py failed to convert the file."
    exit 1
  fi

  if [ ! -f "$SOURCE_DAE" ]; then
    echo "Error: .dae file was not created after conversion."
    exit 1
  fi
fi

cp "$SOURCE_PLY" "$TARGET_MAPS_DIR"
cp "$SOURCE_DAE" "$TARGET_MAPS_DIR"
cd "$TARGET_SIM_DIR" || exit
mkdir -p "$MESHES_DIR"

cp ../mesh_navigation_tutorials/maps/"$BASE_NAME".ply "$MESHES_DIR"
cp ../mesh_navigation_tutorials/maps/"$BASE_NAME".dae "$MESHES_DIR"

# model.config
cat <<EOF > "$MODEL_DIR/model.config"
<?xml version="1.0"?>
<model>
  <name>HS Fulda Campus</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Nico Schroeder</name>
  </author>
  <description>
    hsfd
  </description>
</model>
EOF

# model.sdf
cat <<EOF > "$MODEL_DIR/model.sdf"
<?xml version="1.0" ?>
<sdf version="1.6">
<model name="hsfd">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <mesh>
          <scale>1 1 1</scale>
          <uri>meshes/hsfd.dae</uri>
        </mesh>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <mesh>
          <scale>1 1 1</scale>
          <uri>meshes/hsfd.dae</uri>
        </mesh>
      </geometry>
    </visual>
  </link>
</model>
</sdf>
EOF

echo "Setup complete for model: $BASE_NAME"
