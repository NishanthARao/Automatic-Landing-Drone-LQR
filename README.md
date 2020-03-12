# Automatic-Landing-Drone-LQR

This simulation is carried out on ROS-melodic and Gazebo9 platform. The drone starts from arbitrary position, and then lands at the origin automatically. This is achieved using the LQR controller. The drone model is created in blender, and is spawned in gazebo using meshes in a .xacro model file.

![Image](https://github.com/NishanthARao/PlutoX-ROS-Joystick-Control/blob/master/Pluto.png)

You can check out the video demonstration here: https://youtu.be/icPwiH_3MI4

The drone matrices are obtained by linearizating the drone dynamics about the hovering equilibrium position. From the Jacobian matrix, we obtain the so called system matrices A and B. Q and R are choosen according to the landing problem. Once we have the A, B, Q, and R matrices, we use the MATLAB command 'lqr' to generate the K matrix. This K matrix is fixed for an LTI system given A, B, Q, and R matrices. Also in our case, the K matrix is a time-invariant matrix.

Further, we have used the NumCpp library as an alternative to Numpy in Python. The reason is that the matrices are huge i.e., of the order 12. Thus matrix computations like matrix multiplications, finding the inverses must be optimized to a large extent in order to use them for real-time applications.

These are the references we have used for the project:<br>
```[1]Modelling of Quadcopter: https://www.kth.se/polopoly_fs/1.588039.1550155544!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf```<br>
```[2]The linear Tracking problem: Linear Optimal Control Systems - Huibert Kwakernaak and Raphel Sivan``` 

# Installation
1. **Install NumCpp**
```
cd ~/Downloads/
mkdir NumCpp 
cd NumCpp
git clone https://github.com/dpilger26/NumCpp.git
```
Build the library:
```
cd ~/Downloads/Numcpp/install/
mkdir build
cd build
cmake ..
make install
```
Next, go to this link:
https://www.boost.org/doc/libs/1_66_0/boost/math/special_functions/chebyshev.hpp<br>
After you have copied the entire code,
```
cd ~
cd /usr/include/boost/math/special_functions/
sudo gedit chebyshev.hpp
```
Paste the code in the file. Save it and close the file along with the terminal. It is highly recommended that you restart your system. If you face any problems installing NumCpp, follow the official link here:
https://dpilger26.github.io/NumCpp/doxygen/html/md__c_1__users_pilgeda__documents__git_hub__num_cpp_install__r_e_a_d_m_e.html

2) **Get prerequisite packages**
For the program to work properly, we need to get gazebo-ros-control and some additional dependencies.
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 
sudo apt-get update
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-ros-control ros-melodic-ros-controllers
sudo apt-get install ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-position-controllers
```
3) **Download and Run the simulation**
Initialize your ROS workspace
```
mkdir -p ~/catkin_control_ws/src
cd ~/catkin_control_ws/src
catkin_init_workspace
```
Build your workspace
```
cd ~/catkin_control_ws/
catkin_make
```
Once the workspace is built,
```
cd ~
gedit ~/.bashrc
```
Add this line at the end of the file:
```
source ~/catkin_control_ws/devel/setup.bash
```
Save the file. Close the terminal and reopen a new terminal.


Create a ROS package called ```lqr```
```
cd ~/catkin_control_ws/src/
catkin_create_pkg lqr roscpp rospy std_msgs
```
Download all the files into the lqr folder. You have to replace the existing ```src``` folder, CMakeLists.txt and the package.xml files. Donot MERGE them. Thus the folder hierarchy should look as follows:
```
catkin_control_ws/src/lqr
  -config
  -launch
  -meshes
  .
  .
  .
  -worlds
  -CMakeLists.txt
  -package.xml
```
It is necessary that you build the lqr package
```
cd ~/catkin_control_ws/
catkin_make
```
Please note that the catkin_make command also compiles the ROS program ```landing_lqr.cpp``` present in the ```src``` folder. Thus, if you want to make any changes to the ```landing_lqr.cpp``` make sure to compile the code again via the catkin_make command.<br>
The building process should not result in any error. Please check the above steps if you face any error.

Now, it is important that you have all the models required for generating the gazebo environment. For this, copy the folder ```models```. Go to home directory and press ```ctrl+h```. You should see some additional folders pop up, and one of them will be ```.gazebo```. Go inside this folder and paste the ```models``` folder here. The ```ctrl+h``` command brings up the hidden folders. So if they are annoying to see each time you open the home directory, just prese ```ctrl+h``` again, so that the hidden folders are now hidden.

Now, launch the simulation
```
roslaunch lqr pluto_gazebo.launch
```
If you have followed all the instructions properly, the gazebo simulation must pop up, and you should see the PlutoX drone on a grass plane. Note that it may take some time for the gazebo simulation to launch, as we have just added some new models to it. Worst case, just restart your system and check again.
You should see the drone spawned on the grass plane. After this, run the command
```
rosrun lqr landing_lqr
```
You should see the drone starting from some arbitrarily specified position and then land at the origin.
