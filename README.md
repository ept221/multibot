# Multibot 
Multibot is a ROS Kinetic package that allows for the simulation of multiple turtlebots in Gazebo, each with their own navigation stack, sharing a common map, and all visualized in rviz.

## Running the Package 
To launch the package, run: roslaunch multi multi/launch/start. The main launch file start.launch has two optional commandline arguments: world_file, and map_file. If these are **not** provided, a default world and map will be loaded. To load a cusom world and file run: roslaunch multi multi/launch/start world_file:=customWorld.world map_file:=customMap.yaml, where customWorld.world and customMap.yaml are your custom files.

## Configure the Package
### Add Another Bot:
To add another turtlebot to the system, open the start.launch file and insert the following:
```xml
<include file="single.launch">
	<arg name="name" value="INSERT STRING HERE"/>
	
	<arg name="x_pose" value="INSERT FLOAT HERE"/>
	<arg name="y_pose" value="INSERT FLOAT HERE"/>
	<arg name="z_pose" value="INSERT FLOAT HERE"/>
	<arg name="R_pose" value="INSERT FLOAT HERE"/>
	<arg name="P_pose" value="INSERT FLOAT HERE"/>
	<arg name="Y_pose" value="INSERT FLOAT HERE"/>
	
	<arg name="x_pose_amcl" value="INSERT FLOAT HERE"/>
	<arg name="y_pose_amcl" value="INSERT FLOAT HERE"/>
	<arg name="Y_pose_amcl" value="INSERT FLOAT HERE"/>
</include>
```
where name is the unique string which shall identify the new robot, where *x_pose, y_pose,* and *z_pose* are floats giving the initial 3D cartesian coordinates of the robot in the Gazebo world, where *R_pose, P_pose,* and *Y_pose* are floats giving the initial role, pitch, and yaw of the robot in the Gazebo world, and where *x_pose_amcl, y_pose_amcl,* and *Y_pose_amcl* are floats giving the inital 2D cartesian coordinates and the yaw of the robot in the map used by the navigation stack.

The name argument is the only required argument, and all others default to 0.0. Additionaly, if *x_pose_amcl, y_pose_amcl,* and *Y_pose_amcl* are not provided, they will default to *x_pose, y_pose* and *Y_pose*. The reason they are included is to account for a rotation in the map relative to the Gazebo world. For example, suppose you wanted to spawn a new robot at (1,1) in the Gazebo world, but your map used for navigation was rotated 90 degrees counterclockwise. Then you would want to set *x_pose = 1.0, y_pose = 1.0*, but then include *x_pose_amcl = -1.0*, and *y_pose_amcl = 1.0*. (In this example technically you could leave out the *y_pose_amcl*, because in this case it does not differ from the initial Gazebo *y_pose* value, but it is still good practice to include it for clarity).

### Adjusting Navigation Parameters:
The parameter .yaml files for amcl, movebase, etc. are located in multi/include/param. One important change from the default perameters is that in the move_base.launch file, the parameter move_base/DWAPlannerROS/max_rot_vel has been lowered to 1.0 instead of the default 5.0. This was done to pervent the turtlebot from oscillating and moving in loops while traveling to its destination.
