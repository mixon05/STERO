# Ignacy Mermer
# Miłosz Więcyk

# Sprawozdanie
## Zad 1.
Komenda vcs import pozwoliła na dodanie plików do folderu src w projekcie. Dodane zostały podfoldery z tiago, gazebo zaimportowane z github usercontent.

## Zad 2.
Colcon build pozwolił na zbudowanie projektu i dodanie folderów build, install oraz log do folderu projektu tiago. 
Flaga --symlink-install pozwala na niekopiowanie wszystkich plików zbudowanych podczas kompilacji do katalogu instalacyjnego, zamiast tego tworzy linki symboliczne do plików w katalogu źródłowym lub katalogu build i w install są one dostępne przez linki symboliczne.

## Zad 3. 
Po zbudowaniu folderu stero i uruchomieniu komendy ros2 run hello_stero ... na ekranie terminalu zostało wypisane 
```hello world hello_stero package```

## Zad 4.
klient: //p/l/a/y/_/m/o/t/i/o/n/2_move_group_node, serwer: move_group

## Zad 5.
Klienci:
- //p/l/a/y/_/m/o/t/i/o/n/2_move_group_node
- rviz
serwer: 
- move_group

## Zad 6.
```
/move_group
  Subscribers:
    /clock: rosgraph_msgs/msg/Clock
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /throttle_filtering_points/filtered_points: sensor_msgs/msg/PointCloud2
    /trajectory_execution_event: std_msgs/msg/String
  Publishers:
    /display_contacts: visualization_msgs/msg/MarkerArray
    /display_planned_path: moveit_msgs/msg/DisplayTrajectory
    /filtered_cloud: sensor_msgs/msg/PointCloud2
    /motion_plan_request: moveit_msgs/msg/MotionPlanRequest
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /robot_description_semantic: std_msgs/msg/String
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /apply_planning_scene: moveit_msgs/srv/ApplyPlanningScene
    /check_state_validity: moveit_msgs/srv/GetStateValidity
    /clear_octomap: std_srvs/srv/Empty
    /compute_cartesian_path: moveit_msgs/srv/GetCartesianPath
    /compute_fk: moveit_msgs/srv/GetPositionFK
    /compute_ik: moveit_msgs/srv/GetPositionIK
    /get_planner_params: moveit_msgs/srv/GetPlannerParams
    /load_map: moveit_msgs/srv/LoadMap
    /move_group/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /move_group/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /move_group/get_parameters: rcl_interfaces/srv/GetParameters
    /move_group/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /move_group/list_parameters: rcl_interfaces/srv/ListParameters
    /move_group/set_parameters: rcl_interfaces/srv/SetParameters
    /move_group/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /plan_kinematic_path: moveit_msgs/srv/GetMotionPlan
    /query_planner_interface: moveit_msgs/srv/QueryPlannerInterfaces
    /save_map: moveit_msgs/srv/SaveMap
    /set_planner_params: moveit_msgs/srv/SetPlannerParams
  Service Clients:

  Action Servers:
    /execute_trajectory: moveit_msgs/action/ExecuteTrajectory
    /move_action: moveit_msgs/action/MoveGroup
  Action Clients:
```

```
/move_group_private_109754435045824
  Subscribers:
    /attached_collision_object: moveit_msgs/msg/AttachedCollisionObject
    /clock: rosgraph_msgs/msg/Clock
    /collision_object: moveit_msgs/msg/CollisionObject
    /joint_states: sensor_msgs/msg/JointState
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /planning_scene: moveit_msgs/msg/PlanningScene
    /planning_scene_world: moveit_msgs/msg/PlanningSceneWorld
  Publishers:
    /monitored_planning_scene: moveit_msgs/msg/PlanningScene
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /get_planning_scene: moveit_msgs/srv/GetPlanningScene
    /move_group_private_109754435045824/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /move_group_private_109754435045824/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /move_group_private_109754435045824/get_parameters: rcl_interfaces/srv/GetParameters
    /move_group_private_109754435045824/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /move_group_private_109754435045824/list_parameters: rcl_interfaces/srv/ListParameters
    /move_group_private_109754435045824/set_parameters: rcl_interfaces/srv/SetParameters
    /move_group_private_109754435045824/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```

move_group odpowiada za planowanie trajektorii i jej wykonywanie oraz przesyłanie statusu akcji.

move_group_private odpowiada za wewnętrzne dane pakietu MoveIt. Odpowiada za monitorowanie i planowanie obrazu

## Zad 7.
Wezęł /moveit_simple_control_manager pozwala nadzorowanie poszczególnych częsci robota. Przy analizie schematu rqt_graph informacja ze jest wtyczką move_group pozwala na połączenie tych węzłów mimo, ze nie mają bezpośredniego powiązania między sobą.

## Zad 8.
```
command interfaces
	arm_1_joint/effort [available] [unclaimed]
	arm_1_joint/position [available] [claimed]
	arm_1_joint/velocity [available] [unclaimed]
    ...
```

## Zad 9.

state_interfaces:state_start_arm_1_joint/position -> joint_state_broadcaster:state_end_arm_1_joint/position
                                                  -> arm_controller:state_end_arm_1_joint/position

state_interfaces:state_start_arm_1_joint/velocity -> joint_state_broadcaster:state_end_arm_1_joint/velocity

state_interfaces:state_start_arm_1_joint/effort -> joint_state_broadcaster:state_end_arm_1_joint/effort

arm_controller:command_start_arm_1_joint/position -> command_interfaces:command_end_arm_1_joint/position

# Zad 10.

Robot po uruchomieniu zaczął poruszać ręką.

```
[INFO] [1729152611.040644551] [move_group_interface]: Ready to take commands for planning group arm_torso.
[INFO] [1729152611.040880134] [move_group_interface]: MoveGroup action client/server ready
[INFO] [1729152611.063644997] [move_group_interface]: Planning request accepted
[INFO] [1729152611.398016223] [move_group_interface]: Planning request complete!
[INFO] [1729152611.398950684] [move_group_interface]: time taken to generate plan: 0.0711892 seconds
[INFO] [1729152611.403429882] [move_group_interface]: Execute request accepted
[INFO] [1729152653.388456539] [move_group_interface]: Execute request success!
```

# Zad 11.

Po naciśnięciu przysku Next odpalana jest wizualizacja i wizualizowany jest ruch robota i trasa po której podąza (Motion Planing).

# Zad 12.
Na wizualizacji pojawiła się przeszkoda (zielony prostopadłościan).

# Zad 13.
Klocek postawiony przy uzyciu Rviz dla szybszego dobrania pozycji, ruch robota i trajektoria jego jointa wizualizowana przy uzyciu motion planing.
