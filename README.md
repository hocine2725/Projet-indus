# Programmation de la trajectoire de l'UR

Simple overview of use/purpose.

## Description

Notre but est de détecter le plan et de plannifier une trajectoire afin de faire une dépose

## Getting Started

### Dependencies

* ubuntu 16
* installer le dernier driver de l'UR "https://github.com/UniversalRobots/Universal_Robots_ROS_Driver"
* installer trac_ik_kinematics_plugin (pas encore disponible sur ubuntu 20)

### Installing

* recopier et écraser le dossier fmauch_universal_robot dans le dossier du driver installé précedemment 
* recopier les dossier trajectory / detection / simulation 
* pour la simulation recopier le dossier Qr dans ./gazebo/models/
### Executing program

* manipuler le robot réel

Veuillez suivre le manuel fourni dans le dossier afin d'établir et de configuration de l'adresse IP 
faire la calibration du robot au préalable  
```
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.56.2 kinematics_config:="${HOME}/my_robot_calibration.yaml"

```
lancer moveit 
```
roslaunch ur3_e_moveit_config ur3_e_moveit_planning_execution.launch
```
lancer rviz 
```
roslaunch ur3_e_moveit_config moveit_rviz.launch config:=true
```
lancer le service 
```
rosrun trajectory server.py
```
lancer le programme qui permet de se mettre à la position Initial
```
rosrun trajectory move.py
```
bien placer le aruco, lancer la detection (il faut faire la calibation au préalable)
```
rosrun detection detection.py
```

afficher les points 
```
rosrun trajectory points.py
```

lancer la trajectoire 
```
rosrun trajectory client.py
```


*pour la simulation gazebo 
changer éventullement l'URDF selon le type de robot afin d'ajouter la camera "http://gazebosim.org/tutorials?tut=ros_gzplugins"

lancer gazebo 
```
roslaunch ur_e_gazebo ur3e.launch limited:=true
```
lancer moveit (changer eventuellement le fichier controllers dans fmauch_universal_robot/ur3_e_moveit_config/config )
pour la simulation mettre : /follow_joint_trajectory
pour le robot réel :scaled_pos_joint_traj_controller//follow_joint_trajectory

```
roslaunch ur3_e_moveit_config ur3_e_moveit_planning_execution.launch sim:=true limited:=true
```
lancer rviz comme précedemment 

ca permet de publier un repere aruco fixe
```
rosrun simulation fixe.py
```
lancer la detection avec la camera de la simulation, il faut insérer le model QR dans le monde gazebo 
```
rosrun simulation detect.py
```



## Help
Contant the Authors

## Authors
Polytech Sorbonne 
Hocine BEN HAMOU
hocine2725@gmail.com
