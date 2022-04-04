============
Installation
============

To install voxfield, please install `ROS Indigo <http://wiki.ros.org/indigo/Installation/Ubuntu/>`_, `ROS Kinetic <http://wiki.ros.org/kinetic/Installation/Ubuntu/>`_ or `ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu/>`_.
These instructions are for Ubuntu, Voxfield will also run on OS X, but you're more or less on your own there.

First install additional system dependencies (swap kinetic for indigo or melodic as necessary)::

	sudo apt-get install python-wstool python-catkin-tools ros-kinetic-cmake-modules protobuf-compiler autoconf libprotobuf-dev protobuf-c-compiler

Next, add a few other dependencies.
If you don't have a catkin workspace yet, set it up as follows::

	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws
	catkin init
	catkin config --extend /opt/ros/kinetic
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
	catkin config --merge-devel

If using `SSH keys for github <https://help.github.com/articles/connecting-to-github-with-ssh/>`_ (recommended)::

	cd ~/catkin_ws/src/
	git clone git@github.com:VIS4ROB-lab/voxfield.git 
	wstool init . ./voxfield/voxfield_ssh.rosinstall
	wstool update


If **not using SSH** keys but using https instead::

	cd ~/catkin_ws/src/
	git clone https://github.com/VIS4ROB-lab/voxfield.git
	wstool init . ./voxfield/voxfield_https.rosinstall
	wstool update

If you have already initalized wstool replace the above ``wstool init`` with ``wstool merge -t``

Compile::

	cd ~/catkin_ws/src/
	catkin build voxblox_ros
