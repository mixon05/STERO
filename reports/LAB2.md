## Zad 3
```
/gazebo_ros_state
  Subscribers:

  Publishers:
    /link_states: gazebo_msgs/msg/LinkStates
    Stany linków
    /model_states: gazebo_msgs/msg/ModelStates
    Stany obiektów
  Service Servers:
    /get_entity_state: gazebo_msgs/srv/GetEntityState
    Pozwala na pobranie stanu określonego obiektu w symulacji. Uzyskanie informacji o pozycji, orientacji, prędkości i innych właściwościach obiektu.
    /set_entity_state: gazebo_msgs/srv/SetEntityState
    Umożliwia ustawienie stanu obiektu w symulacji. Zmiana położenia, orientacji oraz innych właściwości obiektów.
  Service Clients:

  Action Servers:

  Action Clients:
```

## Zad 5
```
[hello_moveit]: Uklad bazowy B: base_footprint
[hello_moveit]: Uklad koncowki E: arm_tool_link
arm_7_link(F): x: -0.003658867906779051 y:-0.022740164771676064, z:-0.6903448104858398
arm_tool_link(E): x: 0.1912352591753006, y:0.0.184370756149292, z:0.6260688304901123
base_footprint(B): x: 0.0007377051515504718 y:1.3758535715169273e-06, z:0
Pozycja F względem E: x: 0, y: 0, z: 0.046, orientacja: x: 0.5, y: 0.5, z: 0.5, w: 0.5
```

## Zad 6
```
ros2 launch hello_moveit tiago_gazebo.launch.py navigation:=True moveit:=True is_public_sim:=True world_name:=stero use_grasp_fix_plugin:=True
arm_tool_link(E): x: 0.63728, y:0.49961, z:0.76781, orientacja: x: -0.28463, y: 0.65575, z: 0.32315, w: 0.62014
```
