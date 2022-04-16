### Robot22
----------------------------------------------------------------------------
FRC Team 4450 2022 Robot Control program.

This is Mr. Corn's 2022 program. DO NOT merge back into the master branch.

This is the Command based 2022 competition robot control program reference implementation created by the Olympia Robotics Federation (FRC Team 4450). 
Operates the robot "Mugato" for FRC game "Rapid React".

----------------------------------------------------------------------------
## Instructions to setup development environment for VS Code
1) Follow the instructions [here](https://wpilib.screenstepslive.com/s/currentCS/m/java) to setup the JDK, Visual Studio Code, the FRC plugins and tools. Do not install the C++ portion. You do not need the FRC Update Suite to compile code.
2) Clone this repository to local folder.
3) Open that folder in Visual Studio Code.
4) Build the project using the build project command from the WPILib commands list.

### If RobotLib gets an update:
1) download the RobotLib.json file from the RobotLib Github repo and drop it into the vendordeps folder inside the project folder. Build the project.
****************************************************************************************************************
Version 22.4

*   Merge RAC season development branch onto master. Any future work will be done on master.

R. Corn, April 16 2022

Version 22.RAC.3

*   Replace the home brew SRX Magnetic Encoder simulation scheme using dmmy analog gyros with CTRE's built-in 
    sim support. Requires v3.13.0 of RobotLib.
*   Various tweaks and clean up get everything working with the SRX encoder sim changes.

R. Corn, April 8 2022

Version 22.RAC.2

*   Replace the home brew NavX simulation scheme using dmmy analog gyros with NavX's built-in sim support.
    Requires v3.12.0 of RobotLib.
*   Various tweaks and clean up get everything working with the NavX sim changes.

R. Corn, April 2 2022

Version 22.RAC.1

*   Move LCD panel displays to new ShuffleBoard class.

R. Corn, March 29 2022

Version 22.RAC.0

*   Start Mr. Corn's code for 2022 robot.

R. Corn, February 10 2022

Version 22.0

*   Base code for 2022 robot.

R. Corn, January 2022
