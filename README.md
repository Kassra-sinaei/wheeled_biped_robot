# wheeled_biped_robot
6-DoF wheeled biped robot

<br>
<p>9888 This Project is in progress...</p><br>
<br>

<b>How To Run</b><br>
1.Choreonoid sim<br>
<dl>
  <dt>Open choreonoid GUI</dt>
  <dt>Add each file in choreonoid Branch to similar folder in choreonoid directory</dt>
  <dt> File->Open Project->WB_minimum.cnoid </dt>
  <dt> Run AISTSimulator </dt>
</dl>
<br>
<img src="WB_test#2.gif" align="center"/> 
<br>
1.Gazebo Simulation sim<br>
<dl>
  <dt>Create Catkin Workspace and clone src folder to your catkin_ws/src directory</dt>
  <dt> Go to catkin_ws and run $catkin_make</dt>
  <dt> To run simulation without contrller run $ roslaunch robot_sim robot_full.launch </dt>
  <dt> To run simulation with contrller run $ roslaunch robot_control const_pid.launch </dt>
</dl>
<br>
<img src="wheeled_biped_gazebo.png" align="center"/>
