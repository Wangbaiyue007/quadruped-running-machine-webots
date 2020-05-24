# Quadraped-running-machine
A quadruped running machine in webots, planned to simulate three different gaits: trotting, pacing, and bounding.<br>

## Instructions
Open "Quadruped-running.wbt" in Webots<br>
Control the robot with the four arrow keys, where "↑" or "↓" means forward and backward, "←" or "→" means turning left or right.<br>
## In the process
We've just done the "troting" gait. Now working on "pacing" and "bounding".<br>
Some error will show up in the consle during running but it usually dose not affect the robot.<br>
When the yaw angle is less than 45° w.r.t z-axis, the robot will become unstable due to the inaccurate IMU measurment.<br>
## Reference
We started our project referencing an existed one-legged-hopping-machine project: https://github.com/YuXianYuan/Hopping-in-Three-Dimensions.git<br>
We're also motivatied by Raiber's work back to the 1980s. Articles can be found in our reference dirctory.<br>
The one-legged .wbt file and the licence of the previous author is kept.<br>
