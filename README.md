<a name="top"></a>
# Vending Machine
<p align="center">
  <img src="Imagenes/Maquina_Botanas_Frente.png" alt="Image Open" style="width:30%;"> 
</p>


<p align="center">This is a simulation of the behavior for the hardware of a vending machine in VHDL.

## Project Overview
The Basys 3 is an FPGA (Field Programmable Gate Array) board and stands as one of the most recommended and utilized tools for student training in system development through hardware description. In this project, the internal simulation of a candy vending machine will be carried out, aiming to receive money, dispense the product, and, if necessary, provide change. Prior to this stage, a thorough analysis of the development of a sequential design in VHDL will be conducted, specifically tailored for the Basys 3 board. This analysis will be performed using the EDA Playground development environment in collaboration with the Vivado 2018.2 plugin developed by Xilinx, Inc. To implement the design code for the Basys 3 board, we will use the Vivado 2018.2 software.
  
  
  
## Content List
- [Vending Machine](#vending-machine)  
- [Project Overview](#project-overview)  
- [Requirements](#requirements)
- [VHDL Code Development](#vhdl-code-development)
  - [Frequency Divider](#Frequency-Divider-(div_freq_1_hz).)
  - [Validation of the EDA Playground Testbench](#validation-of-the-eda-playground-testbench)
- [Initial Setup](#initial-setup)
  - [Installation](#installation)
  - [QSPI](#qspi)
  - [Hardware Configuration](#hardware-configuration)
  - [Basys 3](#basys-3)
  - [Simulation](#simulation)
  - [Changes in the original project](#changes-in-the-original-project)
- [Future Enhancements](#future-enhancements)
- [Important Links](#important-links)
- [Contact](#contact)
## Requirements

To run this project you need the following components:

- Vivado (Software).
- EDA Playground (Software).
- Basys 3.

# VHDL Code Development

In this section, the codes that compose this project will be described.

## Frequency Divider (div_freq_1_hz).

The code is divided into two processes, gen_clock and persecond. The gen_clock process is responsible for counting and updating the clock state. When the counter reaches its maximum value (max_count), the clock is inverted (from 0 to 1 or vice versa). The counter is then reset to 0.

The persecond process simply takes the split clock value (clk_state) and assigns it to the clkSplit output signal.

This design has the advantage that the frequency of clkDivided can be easily changed by simply changing the value of max_count. Additionally, using an event-based account ensures that the split clock has a consistent, well-defined period, which is useful for controlling devices.

A disadvantage of this design is that it consumes more resources than other frequency division methods, such as D flip-flops, but it has the advantage of being more flexible and not relying on external components such as counters.

The code also includes a reset to reset the counter and split clock signal at any time.

  
  <p align="center">
  <img src="Images/procesomatriz.PNG" alt="Imagen Open">
  </p>
[Back to Top](#top)
  
### Obtaining and validation of the Forward Kinematics and Inverse Kinematics using Matlab.

Made the DH parameters, it is possible to obtain the direct and inverse kinematics of the robot, this will be done using the Matlab software, in conjunction with Peter Corke's "Robotics Toolbox" plugin, by obtaining and validating both kinematics we can obtain the kinematic analysis of the robot.

In general, the procedure consists of generating the robot in Matlab, giving the specifications of the measurements and types of the joints, as well as the rotation that they have, that is, substituting the values in the DH matrix, which remains in the following way:

<p align="center">
  <img src="Images/DHvalores.PNG" alt="Imagen Open">
</p>

Using the ".teach" command, the graphical interface of the robot is printed, which is as follows:


<p align="center">
  <img src="Images/robotmatlab.PNG" alt="Imagen Open">
</p>


The direct kinematics is obtained by calculating through the analysis of each of the joints, to later obtain the analysis of the entire robot by multiplying the results of all the joints, the procedure for each joint is as follows:

<p align="center">
  <img src="Images/proceDC.PNG" alt="Imagen Open">
</p>

Therefore, it is enough to substitute the values ​​in each of the operations, said values ​​are the same as those of the DH table, this applies to all the joints of our robot, with which it only remains to multiply the analyzes of the different joints. To know that the calculation is correct, the values of the matrix obtained are compared with the values ​​of the simulation, as observed below:

<p align="center">
  <img src="Images/CD.PNG" alt="Imagen Open">
</p>


Since the values are the same, the calculation is correct.

To obtain the inverse kinematics, a complex analysis must be carried out for each joint, in which the procedure differs depending on the characteristics of each one, so using the Robotics Toolbox plugin, the ".ikine" command can be used, which performs the calculation in the position that we assign, the result is shown below:

<p align="center">
  <img src="Images/CI.PNG" alt="Imagen Open">
</p>


With both kinematics validated, we proceed to use Ros and Gazebo.


[Back to Top](#top)
## Initial Setup

### Installation
Is important to say that is recomended to install Ubuntu 20.04 in the computer not in a Virtual Box.
The OpenManipulator is configurated to work in ROS Noetic in the mentioned Ubuntu version, once the Ubuntu is installed is recomended to install ROS from the Wiki:
http://wiki.ros.org/noetic/Installation/Ubuntu


However the installation also can be donde with this comand that is the fast installation:
```ROS
$ sudo apt update
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
$ chmod 755 ./install_ros_noetic.sh
$ bash ./install_ros_noetic.sh
```

Install dependent pacakges
```ROS
$ source ~/.bashrc
$ sudo apt-get install ros-noetic-ros-controllers ros-noetic-gazebo* ros-noetic-moveit* ros-noetic-industrial-core
$ sudo apt install ros-noetic-dynamixel-sdk ros-noetic-dynamixel-workbench*
$ sudo apt install ros-noetic-robotis-manipulator
```
Download and build OpenMANIPULATOR-X packages
```ROS
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
$ cd ~/catkin_ws && catkin_make
```
This project is an addition for the OpenManipulator_teleop. A CPP program was created in the 'src' directory, and all the dependencies such as the launch files and headers were created. Additionally, the CMake file was modified to add the pick and place function. To use this project, ensure that all the OpenMANIPULATOR-X packages are installed. Then, create a new code file with the desired name and location, either through VS Code or directly in a text editor. Remember to save the file with the corresponding name. You can check deeper this part in Changes in the original project
### Open CR
Once all the OpenMANIPULATOR-X packages are installed and ROS (Robot Operating System), the hardware provided by the lab for the manipulation of the robot is the OpenCR
<p align="center">
  <img src="Images/opencr.png" alt="Open CR" style="width:20%;"> 
</p>
In order to connect the computer to the Open Manipulator X, it is important to download Arduino. As of the date of this project, the downloaded version is Arduino IDE 1.8.19, specifically for Linux.


https://www.arduino.cc/en/software


Once Arduino is downloaded, is time to configured the Open CR to the port of the computer, the Open has a guide where in case of the Open CR is not flashed the new user can configured as well in the following link:

https://emanual.robotis.com/docs/en/parts/controller/opencr10/#arduino-ide

First the ports are configured:

USB Port Settings
```ROS
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
$ sudo cp ./99-opencr-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```
Compiler Settings
```ROS
$ sudo apt-get install libncurses5-dev:i386
```
Now once Arduino is downloaded extract the file and install it.

```ROS
$ cd ~/downloads/arduino-1.8.19
$ ./install.sh
```
Set the file path of installed Arduino IDE as an absolute path named PATH in the bashrc file.
```ROS
$ gedit ~/.bashrc
$ export PATH=$PATH:$HOME/tools/arduino-1.8.19
$ source ~/.bashrc
```
Porting to Arduino IDE(Linux)
Install the OpenCR package via Boards Manager
Click Tools → Board → Boards Manager.
<p align="center">
  <img src="Images/Openboard.png" alt="Board configuration" style="width:30%;"> 
</p>
<p align="center">
  <img src="Images/Openboard2.png" alt="Port Configuration " style="width:30%;"> 
</p>
However, if is not clear enough there is the link for the Open CR in the links given above

[Back to Top](#top)
### Simulation
Once all the previous steps are done the implementation can be done.

For initialized the Open is command is necessary to give torque to the motors by the following comand:
```ROS
$ roslaunch open_manipulator_controller open_manipulator_controller.launch usb_port:=/dev/ttyACM0 baud_rate:=1000000
```
To run the project you need to write the following command
```ROS
$ roslaunch open_manipulator_teleop open_manipulator_teleop_Pick_and_Place.launch
```
<p align="center">
  <img src="Images/PickandPlace.png" alt="Pick and Place program" style="width:55%;"> 
</p>
Once the user select a option, the interface will be showing each step that the robot needs to complete the task selected.

Finally, depending of the choice the robot wil start moving, in the video is shown the Palletizing and the Depalletizing:


<p align="center">
  <a href="https://www.youtube.com/watch?v=AY5m8ooS1Zg"><img src="https://img.youtube.com/vi/AY5m8ooS1Zg/0.jpg" alt="Video de pick and place"></a>
</p>

## Changes in the original project
All the codes were created in c++ taking as a base the Original file of OpenManipulator_Teleop, if you want to use this example of pick and place you can add all the files to the original documents in the part of OpenManipulator_Teleop, as the Header, the Lunch and the C++ in the src file where is located the program, also is important to modify the CMakeLists.txt, the exact part that needs to be added is the following part:

```ROS
add_executable(open_manipulator_teleop_Pick_and_Place src/open_manipulator_teleop_Pick_and_Place.cpp)
add_dependencies(open_manipulator_teleop_Pick_and_Place ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_teleop_Pick_and_Place ${catkin_LIBRARIES})
```
Where this code is responsible for compiling and building an executable named "open_manipulator_teleop_Pick_and_Place", specifying its dependencies, and linking necessary libraries to it in a ROS project.

You can add this files in VS code or in a text note just save the files with the same name and termination, and for the Cmake just modify it.


## Add New Things

If you want to contribute to this project, please follow these steps:

1. Set the positions of the things that where the Open will do the pick and place.
2. With the matlab codes calculate the inverse kinematics to obtain the positions of the ariticulations.
3. That values can be simulated with the GUI Program. (Do not forget to connect to the Open before, just instead of lunch the new program lunch the following one)
```ROS
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
```
<p align="center">
  <img src="Images/GUI.png" alt="GUI" style="width:45%;"> 
</p>

4. Verify the positions and save it in the new cpp code .
5. Is recomended to simulate first in Gazebo to avoid problems with the real Open.

Launch Gazebo:
```ROS
$ roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
```
Connect the Open to Gazebo
```ROS
$ roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
```
Launch your program
```ROS
$ roslaunch open_manipulator_teleop open_manipulator_teleop_Pick_and_Place.launch
```
<p align="center">
  <img src="Images/Gazebo.png" alt="Gazebon" style="width:45%;"> 
</p>


if the simulation is correct now you can do it with the OpenManipulatorX

## Important Links
For the VHDL compiler you can use this link to check it:

–link eda

For the QSPI you can check this link:

https://youtu.be/5pV0R82A_CE?si=vsrD9TKzp-9_Ht8v

For the Basys 3:

https://digilent.com/reference/programmable-logic/basys-3/reference-manual

Also, other source where more information can be found is the the following book:

https://issuu.com/umbrella12/docs/2019_book_quickstartguidetovhdl


[Back to Top](#top)

## Contact 
Authors:

Ricado Ramos Morales - ricardo.ramosms@udlap.mx - Github: 

Adriel Ivann Ferrer Alejo - adriel.ferrerao@udlap.mx - Github: IvannFerrer 

Irving Alejandro Vásquez Salinas - irving.vasquezss@udlap.mx - IrvingVasquez3

Project Link: 

# Enjoy, entertain yourself, and improve the program!
