# Neural Implicit LiDAR-Based SLAM - Research Project

## HS Fulda Campus PIN-SLAM (8x Speed Up)
<video src='https://github.com/user-attachments/assets/68c96a92-838b-49f9-b878-6d1f69a23948'></video>


## Table of Contents

- [PIN-SLAM](#pin-slam)
  - [Setup](#setup-pin-slam)
  - [Data Preparation](#get-data-for-hsfd-campus)
  - [Configuration](#prepare-config)
  - [Run Algorithm](#run-algorithm)
  - [View Results](#view-results)
- [Mesh Navigation](#mesh-navigation-on-pin-slam-mesh)
  - [Setup](#setup-mesh-navigation)
  - [Data Preparation](#prepare-data)
  - [Run Navigation](#run-mesh-navigation)
- [Links](#links)

---

## PIN-SLAM

### Setup PIN-SLAM

```bash
git clone git@github.com:PRBonn/PIN_SLAM.git
cd PIN_SLAM
conda create --name pin python=3.10
conda activate pin
conda install pytorch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1 pytorch-cuda=11.8 -c pytorch -c nvidia
pip3 install -r requirements.txt
```

### Get Data for HSFD Campus

```bash
python3 rosbag2ply.py -i [path to input rosbag] -o [path to output folder] -t [topic name]
```

### Prepare Config

Edit the configuration file at `config/lidar_slam/run_hsfd.yaml`. Below is a breakdown of key parameters:

- `min_range_m: 3.0`, `max_range_m: 80.0`: Filter out LiDAR points too close (<3m) or too far (>80m), helping reduce noise and irrelevant data.

* `surface_sample_range_m: 0.3`: Radius around surface points to sample for SDF learning.
* `surface_sample_n: 4`: Number of surface samples per measurement.

- `voxel_size_m: 0.4`: Size of voxel grid for neural point map – smaller = higher resolution.
- `search_alpha: 0.8`: Controls neighborhood size during neural point querying (larger = more robust, slower).

* `loss_weight_on: True`: Enables distance-based weighting of loss.
* `dist_weight_scale: 0.8`: Scales loss weight based on distance to reflect noise.

- `batch_size_new_sample: 3000`: New samples added to training buffer per frame.
- `pool_capacity: 1e7`: Max number of points in training data buffer.

* `source_vox_down_m: 0.6`: Downsample resolution for source point cloud used in tracking.
* `iter_n: 100`: Max iterations for pose registration.

- `map_context: True`: Use local map descriptors for loop closure.
 `pgo_freq_frame: 30`: Perform pose graph optimization every 30 frames.

* `iters: 15`: Mapping optimization iterations per frame.
* `batch_size: 4096`: raining batch size per frame (big value = faster convergence but more VRAM needed).

- `o3d_vis_on: False`, `mesh_freq_frame: 50`, `mesh_min_nn: 18`
- `save_map: True`, `save_mesh: True`

### Run Algorithm

```bash
python3 pin_slam.py config/lidar_slam/run_hsfd.yaml -vsm
```

### View Results

```bash
python3 vis_pin_map.py ./experiments/hsfd_* -m 0.2 -c neural_points.ply -o mesh_20cm.ply -n 8
```

---

## Mesh Navigation on PIN-SLAM Mesh

### Setup Mesh Navigation

```bash
cd $YOUR_ROS_WS/src
git clone git@github.com:naturerobots/mesh_navigation_tutorials.git
vcs import --input mesh_navigation_tutorials/source_dependencies.yaml
rosdep install --from-paths . --ignore-src -r -y
cd $YOUR_ROS_WS
colcon build --packages-up-to mesh_navigation_tutorials
source install/setup.bash
```

### Prepare Data

```bash
chmod +x scripts/prepare_mesh_nav.sh
./scripts/prepare_mesh_nav.sh
```

### Run Mesh Navigation

```bash
cd $YOUR_ROS_WS
colcon build --packages-up-to mesh_navigation_tutorials
source install/setup.bash
ros2 launch mesh_navigation_tutorials mesh_navigation_tutorial_launch.py world_name:=hsfd
```

![mesh_nav_overview](assets/mesh_nav_overview.png)

---

## Links

- [PIN-SLAM GitHub](https://github.com/PRBonn/PIN_SLAM)
- [Mesh Navigation GitHub](https://github.com/naturerobots/mesh_navigation)
- [Mesh Navigation Tutorials](https://github.com/naturerobots/mesh_navigation_tutorials)

