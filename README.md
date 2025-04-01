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

- `min_range_m: 3.0`, `max_range_m: 80.0`: Filter out LiDAR points too close or too far.

* `surface_sample_range_m: 0.3`: Radius for sampling around surfaces.
* `surface_sample_n: 4`: Samples per surface point.
* `free_sample_begin_ratio: 0.5`, `free_sample_end_dist_m: 1.0`: Free space sampling settings.
* `free_front_sample_n: 2`: Helps with dynamic object filtering.

- `voxel_size_m: 0.4`: Resolution of neural point map.
- `search_alpha: 0.8`: Neighborhood size for querying.

* `loss_weight_on: True`, `dist_weight_scale: 0.8`
* `ekional_loss_on: True`, `weight_e: 0.5`

- `batch_size_new_sample: 3000`
- `pool_capacity: 1e7`

* `source_vox_down_m: 0.6`, `iter_n: 100`
* `eigenvalue_check: False`, `GM_dist: 0.2`, `valid_nn_k: 5`

- `map_context: True`, `pgo_freq_frame: 30`, `context_cosdist: 0.25`

* `iters: 15`, `batch_size: 4096`

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

