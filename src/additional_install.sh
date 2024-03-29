cd ~
if [ -d "/home/risc/src" ] 
then
    echo "~/src directory already exists"
else
    echo "Creating ~/src directory"
    mkdir src
fi

cd ~/src
if [ -d "/home/risc/src/Firmware" ] 
then
    echo "PX4 Firmware already cloned"
else
    echo "Cloning into ~/src/Firmware"
    git clone https://github.com/PX4/Firmware.git
fi

cd ~/src/Firmware

git submodule update --init --recursive

sudo apt install python3-pip -y
pip3 install empy
pip3 install packaging
pip3 install toml
pip3 install numpy
pip3 install jinja2
pip3 install pyros-genmsg

sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio -y

## Build the firmware without running
DONT_RUN=1 make px4_sitl_default gazebo


## Setup environment variables

echo "source /home/risc/src/Firmware/Tools/setup_gazebo.bash /home/risc/src/Firmware /home/risc/src/Firmware/build/px4_sitl_default" >> ~/.bashrc

echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/risc/src/Firmware:/home/risc/src/Firmware/Tools/sitl_gazebo" >> ~/.bashrc