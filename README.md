crazygames
==========

The aim of this project is to implement min-max time consensus tracking on a multi-quadrotor testbed. The trajectory of a leader quadrotor is generated manually. The remaining quadrotors converge onto this reference trajectory in min-max time using a local feedback control strategy which is known to be globally optimal. Further, in order to analyze the effect of finite communication/measurement rate on consensus tracking, communication delay frame-work has been implemented.  


The Crazyflie 2.0 platorm is chosen to build the multi-quadrotor testbed, with the Optitrack system providing localization information. The implementation of the  controllers is based on ROS, building on the work done [here](https://github.com/whoenig/crazyflie_ros)

Features:
* Leader-Follower 
* Formation flight with multiple agents
* Time-optimal control of quadrotor
* Two-player pursuit-evasion games

For details regarding theory regarding consensus tracking, please refer to [this](https://ieeexplore.ieee.org/document/7944697). A paper based on this project is submitted to ECC 2019.  
