in ubuntu 18.04

```
mkdir -p xx_ws/src && cd ~/mkdir/src
git clone https://github.com/st88018/realsense_ws.git
cd realsense_ws
git checkout e096a95
change CMakelists.txt opencv 4 -> opencv 3
sudo apt-get install ros-<ur-version>-sophus
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

roslaunch real vrpn.launch #vrpn
roslaunch real px4.launch
roslaunch real Jeremy.launch
```

There you go!
Got any queation, find JBL@GH034
