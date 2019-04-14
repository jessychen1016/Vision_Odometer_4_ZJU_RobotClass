# Vision_Odometer_4_ZJU_RobotClass

## Before you get started

* Download and install Realsense SDK on your PC (Windows/Ubuntu/Mac OS) via the link given below
 	https://github.com/IntelRealSense/librealsense

* Gether up all the pieces into the Robot like this:

* Learn how to use CMake and how to build your own project here:
	https://stackoverflow.com/questions/9878225/using-cmake-on-windows-for-c for windows
	or
	https://askubuntu.com/questions/610291/how-to-install-cmake-3-2-on-ubuntu for ubuntu
	or
	ask in the group chat.

* Run samples provided by the SDK to peek into how Realsense works.

* If you are using Windows, please download "Dynamic Link Lib" from release in this repository into your own "bin" folder after building your project.

* Run the demonstation I provides above with an IDE (for Windows, VS2015 recommanded) to get in touch with realsense APIs.

* Replace actionmodule.cpp, actionmodule.h in the "include" file with your own command sender to send command to your own robot.

* Write your own code to realize VO function.

* Have fun!!!


ps:

In the demonstration I provided, you will learn how to start the camera, how to unable filters to the camera to reduce unexpected noise, how to load your own parameters as camera's preset, and how to get color frame, depth frame and pointcloud.

The demonstration package above also contains a CMakeLists.txt for you to cmake and to build your own project, feel free to modify it as you need to.

Noted that Realsense SDK only supports C++ or C# which means you have to handle these languages yourself.
