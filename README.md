<div align="center">
  <h1>BEV-LIO(LC)</h1>
  <h2>BEV Image Assisted LiDAR-Inertial Odometry with Loop Closure</h2>
  <p><strong><i>IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2025)</i>.</strong></p>
  <br>

  [![Code](https://img.shields.io/badge/Code-GitHub-black?logo=github)](https://github.com/HxCa1/BEV-LIO-LC)
  [![arXiv](https://img.shields.io/badge/arXiv-2502.19242-b31b1b.svg)](https://arxiv.org/abs/2502.19242)
  [![YouTube](https://img.shields.io/badge/YouTube-FF0000?logo=youtube&logoColor=white)](https://www.youtube.com/watch?v=JGUbQDItF8g)
  [![Bilibili](https://img.shields.io/badge/Bilibili-00A1D6?logo=bilibili&logoColor=white)](https://www.bilibili.com/video/BV1XD9HYdE1D/?share_source=copy_web&vd_source=c68fb3638c81dd65be55db8f6d22cb6b)

</div>

<p align="center">
  <img width='100%' src="doc/mapoverlay_kth_seq.gif">
</p>

## Pipeline
<div align="center">
<img src="doc/overview.svg" width=99% />
</div>

## 1. Prerequisites

### 1.1 **Ubuntu** and **ROS**

**Ubuntu == 20.04**

ROS == Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

(Other versions haven't been tested)

### 1.2. **PCL && Eigen**

PCL    >= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Eigen  >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

GTSAM >= 4.0.0(tested on 4.0.0-alpha2)

### 1.3. **livox_ros_driver**

Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

*Remarks:*

- Since the FAST-LIO must support Livox serials LiDAR firstly, so the **livox_ros_driver** must be installed and **sourced** before run any FAST-LIO luanch file.
- How to source? The easiest way is add the line ``` source $Livox_ros_driver_dir$/devel/setup.bash ``` to the end of file ``` ~/.bashrc ```, where ``` $Livox_ros_driver_dir$ ``` is the directory of the livox ros driver workspace (should be the ``` ws_livox ``` directory if you completely followed the livox official document).

### 1.4 CUDA && LibTorch

Assume you have installed CUDA, check [this link](https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html) for the CUDA Toolkit version and install [here](https://developer.nvidia.com/cuda-toolkit-archive).

For LibTorch, follow [libtorch installation](https://pytorch.org/get-started/locally/), here i choose to use [Stable-Linux-LibTorch-C++/Java-CUDA11.8](https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.7.1%2Bcu118.zip).

For the question: 
```
BEV-LIO-LC/include/REM.hpp:122:107: error: no matching function for call to ‘torch::nn::functional::GridSampleFuncOptions::mode(const torch::enumtype::kBicubic&)’
122 | auto options = torch::nn::functional::GridSampleFuncOptions().align_corners(true).mode(torch::kBicubic);
```
Change the code at Line 19 in libtorch/include/torch/csrc/api/include/torch/nn/options/vision.h

```c++
typedef std::variant<enumtype::kBilinear, enumtype::kNearest, enumtype::kBicubic> mode_t;
```

Remember to change the path of LibTorch in the CMakeLists.txt to yours.

### 1.5 Model

We directly use the model from [BEVPlace++](https://github.com/zjuluolun/BEVPlace2),  check /src/models/tool, you can use turn.py to transform your trained model to .pt format to be used in our codes at src/BEV_LIO/models/tool.

And change the path of resnet_weights.pth at 86 line in REM.hpp.

## 2.Build

### 2.1 Build from source

Clone the repository and catkin_make:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/HxCa1/BEV-LIO-LC.git
    cd ..
    catkin_make
    source devel/setup.bash
```

- Remember to source the livox_ros_driver before build (follow 1.3 **livox_ros_driver**)
- If you want to use a custom build of PCL, add the following line to ~/.bashrc
  ```export PCL_ROOT={CUSTOM_PCL_PATH}```

## 3. Running

### 3.1 For MCD

The MCD dataset can be downloaded [here](https://mcdviral.github.io/Download.html). 

To run a sequence of ntu:

```
roslaunch bev_lio_lc mcd_ntu.launch
```

To run a sequence of kth or tuhh:

```
roslaunch bev_lio_lc mcd_kth_tuhh.launch
```

### 3.2 For NCD

The NCD dataset can be downloaded [here](https://ori-drs.github.io/newer-college-dataset/download/). 

To run a sequence:

```
roslaunch bev_lio_lc NCD.launch
```

### 3.3 For M2DGR

The M2DGR dataset can be downloaded [here](https://github.com/SJTU-ViSYS/M2DGR). 

To run a sequence:

```
roslaunch bev_lio_lc m2dgr.launch
```

## 4.Citation

Please consider citing our work if you find our code or paper useful:
  ```bibtex
@inproceedings{cai2025bev,
  title={BEV-LIO(LC): BEV Image Assisted LiDAR-Inertial Odometry with Loop Closure},
  author={Haoxin Cai and Shenghai Yuan and Xinyi Li and Junfeng Guo and Jianqi Liu},
  booktitle={Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2025},
  address={Hangzhou, China}
}
  ```

## 5.Acknowledgments

Thank the authors of [BEVPlace++](https://github.com/zjuluolun/BEVPlace2), [FAST-LIO2](https://github.com/hku-mars/BEV_LIO), [COIN-LIO](https://github.com/ethz-asl/COIN-LIO), [MapClosures](https://github.com/PRBonn/MapClosures) and [FAST-LIO-SAM](https://github.com/kahowang/FAST_LIO_SAM) for open-sourcing their outstanding works.

For the problem of FAST-LIO-SAM that crashes during long-term runs, we refer to [FAST-LIO-SAM-LOOP](https://github.com/Hero941215/fast_lio-sam_loop).
- L. Luo, S.-Y. Cao, X. Li, J. Xu, R. Ai, Z. Yu, and X. Chen,
  “Bevplace++: Fast, robust, and lightweight lidar global localizationfor unmanned ground vehicles,” IEEE Transactions on Robotics (T-RO), 2025.
- W. Xu, Y. Cai, D. He, J. Lin, and F. Zhang, “Fast-lio2: Fast direct lidar-inertial odometry,” IEEE Transactions on Robotics, vol. 38, no. 4, pp.2053–2073, 2022.
- P. Pfreundschuh, H. Oleynikova, C. Cadena, R. Siegwart, and O. An-dersson, “Coin-lio: Complementary intensity-augmented lidar inertial odometry,” in 2024 IEEE International Conference on Robotics and Automation (ICRA), 2024, pp. 1730–1737.
- S. Gupta, T. Guadagnino, B. Mersch, I. Vizzo, and C. Stachniss, “Effectively detecting loop closures using point cloud density maps,” in Proc. of the IEEE Intl. Conf. on Robotics & Automation (ICRA),2024.
- J. Wang, “Fast-lio-sam: Fast-lio with smoothing and mapping.” https://github.com/kahowang/FAST_LIO_SAM, 2022.
