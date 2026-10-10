# stereo_vslam

Stereo visual SLAM for the [KITTI odometry](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) sequences. Each stereo pair is tracked, selected frames become keyframes, a short window of those keyframes is refined by local bundle adjustment, and a background thread closes loops with a pose graph.

## Dependencies

C++23 and CMake 4.2 or newer.

1. Eigen3
2. OpenCV
3. Sophus
4. g2o + CSparse (SuiteSparse)
5. glog, gflags
6. yaml-cpp
7. Pangolin
8. FBoW
9. nanoflann
10. Boost

On Ubuntu, the packaged pieces are typically:

```bash
sudo apt install build-essential cmake pkg-config \
  libeigen3-dev libopencv-dev libgoogle-glog-dev libgflags-dev \
  libyaml-cpp-dev libsuitesparse-dev libboost-dev \
  libgl1-mesa-dev libglu1-mesa-dev libglew-dev freeglut3-dev
```

Sophus, g2o (built with the CSparse solver), Pangolin, FBoW, and nanoflann are usually installed from source into `/usr/local`. CMake looks for them with `find_package` (`Sophus`, `G2O`, `Pangolin`, `Glog`, `GFlags`, `CSparse`, `fbow`, `yaml-cpp`). Eigen is included from `/usr/include/eigen3`. g2o is also searched under `G2O_ROOT` and `/opt/ros`.

FBoW needs an ORB vocabulary, for example `orb_mur.fbow` from the [FBoW vocabularies](https://github.com/rmsalinas/fbow/tree/master/vocabularies). Point `loop_closure.vocabulary_path` at that file.

## Build

From the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The executable is `bin/vslam`.

## Run

Set these in `config/config.yaml`:

- `dataset.dataset_path` — a KITTI sequence folder (`calib.txt`, `image_0/`, `image_1/`).
- `loop_closure.vocabulary_path` — the FBoW vocabulary file.
- `viewer.enabled` — set `false` to run without the Pangolin window.

Add the libcudnn path if needed to the `LD_LIBRARY_PATH` .

- `export LD_LIBRARY_PATH=<LIBCUDNN_PATH>:${LD_LIBRARY_PATH}`

From the repository root:

```bash
./bin/vslam --config_file=./config/config.yaml
```

Frames are read until the sequence ends. With the viewer on, the window shows the current pose and the map and closes when the sequence finishes.

## Pipeline

```
stereo frame
    -> Tracker          pose, and a keyframe when the policy asks
    -> LocalMapper      insert keyframe, then local BA
    -> LoopClosure      background loop detect + pose graph
    -> Viewer           current frame and map
```

Tracking and local BA run on the main thread; loop closure runs in the background.

1. The tracker estimates the current camera pose and inserts a keyframe when tracking weakens or the camera turns.
  1. There is an option to use ORB with knn/BF Matcher or Superpoint with LightGlue matcher.
2. Local bundle adjustment refines a short window of recent keyframes after each new keyframe is added.
3. Loop closure looks for a return to a known place and corrects the trajectory when it finds one.



## TODO

- [ ] Add multiple camera mode. 

- [ ] Add test cases.



## Reference

Xiang Gao, Tao Zhang, Yi Liu, and Qinrui Yan. *Introduction to Visual SLAM: From Theory to Practice*. English edition and example code: [gaoxiang12/slambook-en](https://github.com/gaoxiang12/slambook-en), chapter project `slambook2/ch13`.