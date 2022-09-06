# probabilistic_robotics_project
2D Range Only SLAM for the course probabilistc robotics held by professor Giorgio Grisetti
## Results:
- without pose-pose constraints
<img src="https://github.com/DennisRotondi/probabilistic_robotics_project/blob/main/figures/initial_guess_odometry/without_pose_pose_contraints/map.png" width="50%">
<img src="https://github.com/DennisRotondi/probabilistic_robotics_project/blob/main/figures/initial_guess_odometry/without_pose_pose_contraints/traj.png" width="50%">

- with pose-pose constraints
<img src="https://github.com/DennisRotondi/probabilistic_robotics_project/blob/main/figures/initial_guess_odometry/map.png" width="50%">
<img src="https://github.com/DennisRotondi/probabilistic_robotics_project/blob/main/figures/initial_guess_odometry/traj.png" width="50%">

- with pose-pose constraints + GT on the first 3 odometry poses <br>
(in general it's common that the first measurements are more reliable than the following and therefore are correct)
<img src="https://github.com/DennisRotondi/probabilistic_robotics_project/blob/main/figures/partial_knowledge_odometry/map.png" width="50%">
<img src="https://github.com/DennisRotondi/probabilistic_robotics_project/blob/main/figures/partial_knowledge_odometry/traj.png" width="50%">
