# LiDAR NeRF Based SLAM - Literature Research

## Why?
- LiDAR provide sparse yet accurate depth information over long distances
- LiDAR can have critial gaps in the 3D-Representation because of its sparseness

## Main hope
- Create dense, smooth maps of the environemt using sparse LiDAR-Data

## Paper 1: NeRF-LOAM
- has basically 3 interconnected modules

#### 1. Neural odomoetry
- neural odometry module estimates  6-DoF pose for each incoming LiDAR Scan by minimizing SDF errors through a fixed implicit network
- for each new LiDAR Scan the module compares the incoming points to the previously learned SDF and minimizes the error between them
#### 2. Neural mapping
- in parallel to the neural odometry module the neural mapping module employs dynamic voxel embeddings within an octree-based architecture capturing local geometry
- dynamic voxel embedding:
  - 3D Cubes that divide space into smaller regions
  - instead of pre-allocating these voxels across the entire space the system dynamically creates and updates voxels where LiDAR-Data is collected
  - each voxel stores a neural embedding
- octree-based architecture
  - octree is a hierarchical data structure that efficiently organizes 3D-Space
  - this structure ensures that computational resources are focused on areas with rich geometry which reduces memory usage in empty or less relevant regions
- key-scans-refinement-strategy: during mapping, certain scans are refined to improve the maps accuracy
- this should prevent catastrophic forgetting

#### 3. mesh reconstruction

#### Link:
https://openaccess.thecvf.com/content/ICCV2023/papers/Deng_NeRF-LOAM_Neural_Implicit_Representation_for_Large-Scale_Incremental_LiDAR_Odometry_and_ICCV_2023_paper.pdf

## Paper 2: LONER
- Key Highlights:
  - Efficient Odometry with ICP
  - Neural Mapping
  - Dynamic Margin Loss
  - Hierarchical Encoding
  - Mesh from offline-visualization with selected keyframes
- parallel mapping and tracking threads
- tracking thread processes incoming scans using ICP for odometry estimation
- mapping thread uses selected keyframes to update the neural scene represenation
- LiDAR scans are transformed using point-to-plane ICP
- scene is represented by an MLP with a hierarchical feature encoding
- dynamic margin loss function (Jensen-Shannon + Depth Loss + Sky Loss):
  - basically using 3 different loss functions for 3 different purposes
  - apply the loss-function based on the type of region is being processed (flat, detailed, etc.)

#### Link:
https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=10284988

## Paper 3: PIN-SLAM
#### 1. Point-based implicit neural map representation
- environment is moduled using neural points which are:
  - sparse: only key-points are used
  - optimizable: properties can be updated dynamically to improve the map over time
  - elastic: points can deform continously during adjustments
#### 2. incremental learning of SDFs
- SDFs represent the environment implicitly
- PIN-SLAM learns local SDFs incrementally, refining a map as new LiDAR Data is collected
#### 3. Pose-Estimation via Point-to-Implicit-Registration
- instead of finding direct correspondences between individual points, PIN-SLAM aligns LiDAR-Scans with the SDF-Based Map
- correspondence-free-point-to-implicit model registration
- reduces computational complexity
#### 4. Efficient Odometry Estimation
- real time
#### 5. Dynamic Point Filterin
- filters out redundant/irrelevant points from the LiDAR Data
#### 6. Loop Closure Detection
- uses local polar context descriptors (features that describe the local shape of the environment in polar coords)
- PIN-LO: Variant of PIN-SLAM where loop closure detection module is disabled
#### 7. Drift correction through optimized neural points
#### 8. Voxel hashing and neural point indexing for real time performance
#### 9. Compact Map Representation


## Overview of the 3 Papers
1. Scene Encoding
  - NeRF-Loam: Octree Grid + MLP
  - LONER: Hierarchical Grid + MLP
  - PIN-SLAM: Neural Points + MLP
2. Geometry Representation
  - NeRF-Loam: SDF
  - LONER: Density
  - PIN-SLAM: SDF
3. Input Modalities algos can process
  - NeRF-LOAM: Depth
  - LONER: Depth
  - PIN-SLAM: RGB-D, Depth
5. Object/Semantic Segmentation: Pin-SLAM
6. Loop-Closure: Pin-SLAM
7. Frame-to-Model: NeRF-Loam, Pin-SLAM
8. Frame-to-Frame: LONER


#### Link:
https://arxiv.org/pdf/2401.09101


## Datasets
- KITTI: dataset serves as a popular benchmark for evaluating stereo, optical flow, visual odometry/SLAM algorithms, among others. Acquired from a vehicle equipped with stereo cameras, Velodyne LiDAR, GPS and inertial sensors, the dataset contains 42,000 stereo pairs and LiDAR pointclouds from 61 scenes representing autonomous driving scenarios. The KITTI odometry dataset, with 22 LiDAR scan sequences, contributes to the evaluation of odometry methods using LiDAR data. (https://www.cvlibs.net/datasets/kitti/)
- Newer College: 2.2 km walk around New College, Oxford (https://arxiv.org/pdf/ori.ox.ac.uk/datasets/newer-college-dataset)


## Other interesting papers implementing 3DGS-style LiDAR-based SLAM
- LIV-Gauss-Map (https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=10529285)
- MM-Gaussian (https://arxiv.org/pdf/2404.04026)
