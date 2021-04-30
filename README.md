# wheeled_biped_robot
6-DoF wheeled biped robot

<br>
<p>9888 This Project is in progress...</p><br>
<br>

<b>How To Run</b><br>
1.Choreonoid sim<br>
<ol>
  <li>Open choreonoid GUI</li>
  <li>Add each file in choreonoid Branch to similar folder in choreonoid directory</li>
  <li> File->Open Project->WB_minimum.cnoid </li>
  <li> Run AISTSimulator </li>
</ol>
<br>
<img src="WB_test#2.gif" align="center"/> 
<br>
1.Gazebo Simulation sim<br>
<ol>
  <li>Create Catkin Workspace and clone src folder to your catkin_ws/src directory</li>
  <li> Go to catkin_ws and run $catkin_make</li>
  <li> To run simulation without contrller run $ roslaunch robot_sim robot_full.launch </li>
  <li> To run simulation with contrller run $ roslaunch robot_control const_pid.launch </li>
</ol>
<br>
<img src="wheeled_biped_gazebo.png" align="center"/>
