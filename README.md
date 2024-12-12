This repo contains sample code and base classes you can use to build and test your robots. All of your robots should inherit from RobotBase. 
*Don't add anything to this repo!  You can clone it and then copy the files into your other project directory.* Don't push anything here either! 

Here's some information about what's here:

**RobotWarzFlow.drawio**
This is a flow diagram that shows what the basic flow of your arena will need to do in order to automate the robot fight. To open this file, download it to your computer and use a browser based tool called "draw.io"  (you just navigate to draw.io in your browser) then it will let you open a local file. 

**Arena-to-robot-sequence.drawio**
This file shows how to design the calls from your Arena to the robot. The Robots you write will receive these calls from MY arena during the in class portion of the Final Exam. You'll need to implement the functions the way they are specified in the requirements and in the RobotBase base class. 

**RobotBase**
The repo contains RobotBase.h and RobotBase.cpp - this is the base class that you need to build your robots from. Read this code carefully and make sure you understand what it does. Do not change RobotBase - you must use it as is, and you must implement the pure virtual functions. 

**test_robot.cpp**
This file contains code to test your robots. It also has base code to LOAD your robots. You can copy this code into your Arena to load the robots in the directory. Remember your robot will be compiled into a shared object or .so 

**Robot_Ratboy.cpp**
A sample robot. It moves immediately to the left side of the arena, then moves up and down, scanning only to the right. It shoots with a railgun

**Robot_HammerTime.cpp**
A sample robot. It wanders around and scans in a circular pattern. It has a hammer. If it gets close to another robot, watch out! The hammer deals a fearsome blow!

**Robot_Flame_e_o.cpp** 
Flame-e-o hotman!  This test robot wanders around and scorches its opponents!


You can use these robots to test your arena. They are not perfect. But you can see how to construct logic that makes the robots move and shoot. You can see how to use the RadarObj list that gets passed to your robot when it does a radar scan. 








