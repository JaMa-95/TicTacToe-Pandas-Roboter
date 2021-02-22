# TicTacToe-Pandas-Roboter
You can play tic tac toe with a pandas Roboter in ROS.
Wolrd generated in Gazebo

First terminal
•	cd panda_tictactoe_ws
•	source devel/setup.bash
•	roslaunch panda_moveit_config demo.launch

Second terminal (after sourcing):
•	roslaunch panda_control pick_place.launch

Das Codebeispiel liegt in den Dokumenten als zip-Datei vor. Es handelt sich dabei um ein Tic-Tac-Toe Spiel, das mit einem Roboter gespielt werden kann. D
ie benutzten Bibliotheken sind ROS, Rviz und MoveIt. 
Das Projekt ist mit Linux umgesetzt worden. Die erstellten Nodes in C++ finden Sie unter: src-->panda_control-->src Um das Spiel zu starten, 

1 Terminal: cd ~/panda_tictactoe_ws source devel/setup.bash roslaunch panda_moveit_config demo.launch 
2 Terminal: cd ~/panda_tictactoe_ws source devel/setup.bash roslaunch panda_control pick_place.launch 

Das Projekt entstand im Zuge einer Vorlesung als Gruppenprojekt.
