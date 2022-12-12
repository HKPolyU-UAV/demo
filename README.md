CAUTION! Only tested in ubuntu 18.04
Stable Version 12/12/2022 @pattylo


```
mkdir -p xx_ws/src && cd ~/mkdir/src
https://github.com/HKPolyU-UAV/demo.git
cd demo
sudo apt-get install ros-melodic-sophus #if not yet installed
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
cd ../..
catkin_make
```
To launch VICON pose feedback
```
roslaunch demo vrpn.launch #vrpn
```
To launch flight controller
```
roslaunch demo px4.launch #change ip address if needed
```
Launch FSM to demo this very nice system
```
roslaunch demo Jeremy.launch
```

There you go!</br>
Kudos to Jeremy Chang</br>
Got any queation, please do find JBL@GH034</br>
