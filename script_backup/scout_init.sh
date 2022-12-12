source ~/scout_ws/devel/setup.sh
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
roslaunch scout_base scout_mini_base.launch