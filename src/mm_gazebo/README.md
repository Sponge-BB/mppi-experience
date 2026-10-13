mm_gazebo
========

This package contains a minimal launch file to display the `mm.urdf` robot in Gazebo.

How to use
----------

1. Make sure your workspace is built and sourced:

```bash
cd /home/hai/remani_ws
catkin_make  # or your preferred build
source devel/setup.bash
```

2. Launch Gazebo and spawn `mm.urdf` from `my_description`:

```bash
roslaunch mm_gazebo display_mm_gazebo.launch
```

Notes
-----
- This launch file references `mm.urdf` in the existing `my_description` package. It does not copy or modify the URDF.
- If you later want controller integration, move controller-related yaml/launch into this package.
