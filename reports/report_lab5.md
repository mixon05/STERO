# Laboratorium 5

## Zad 1.
W tym zadaniu wykonaliśmy komendę gazebo, a korzystając instrukcji utworzyliśmy dwa światy i zapisaliśmy w folderze worlds.
Nazwaliśmy je apartment oraz corridor, poniżej znajdują się zrzuty ekranu z gazebo.


## Zad 2.
Następnie edytowaliśmy plik pal_gazebo.launch.py poprzez

```
bash
world_name = LaunchConfiguration('world_name').perform(context)

world = ''
if os.path.exists(os.path.join(priv_pkg_path, 'worlds', world_name + '.world')):
    world = os.path.join(priv_pkg_path, 'worlds', world_name + '.world')
```

Przekazaliśmy do pal_gazebo.launch.py nazwę świata poprzez:
```
bash
gazebo = include_scoped_launch_py_description(
pkg_name='lab5',
paths=['launch', 'pal_gazebo.launch.py'],
env_vars=[gazebo_model_path_env_var],
launch_arguments={
    "world_name":  launch_args.world_name,
    "model_paths": packages,
    "resource_paths": packages,
})
```

## Zad 3, 4.
W tym zadaniu przy użyciu teleop_twist_keyboard.py zbudowaliśmy kompletną mapę środowiska. Poniżej znajdują się zrzuty ekranu podczas wykonywania mapy.
![alt text](lab5/images_map/1.png)
![alt text](lab5/images_map/2.png)
![alt text](lab5/images_map/3.png)


## Zad 5.
Następnie wyeksportowaliśmy mapę i zapisaliśmy do folderów corridor oraz apartment. Edytowaliśmy plik tiago_nav_bringup.launch.py poprzez edycję:
```
bash
map_path = os.path.join(lab4, "worlds", world_name, "map.yaml")
```


## Zad 6.
W celu wykonania tego zadania stworzyliśmy nowy węzeł goal_navigator.cpp, który przy wykorzystaniu biblioteki nav2_msgs był odpowiedzialny za wysyłanie celów dla robota na węźle /navigate_to_pose. Użytkownik podawał dwa parametry: destination i difficulty. Dodaliśmy w tym węźle wszystkie pokoje z mapy w świecie gazebo z podziałem na trzy poziomy trudności. Robot poruszał się bez problemów pomiędzy pomieszczeniami. W kolejnym kroku dodaliśmy trzy klocki na trasie robota i sprawdzaliśmy jak się zachowuje. Jeżeli klocki były zbyt blisko siebie lub ściany, robot czasami się zatrzymywał. Poniżej znajdują się zrzuty ekranu z testów powyższych funkcjonalności oraz część kodu programu. 

```
bash
void dispatch_goal(){
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "Action server unavailable after waiting");
        rclcpp::shutdown();
        return;
    }

    auto goal_msg = NavigateToPose::Goal();

    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = get_clock()->now();
    goal_msg.pose.pose.orientation.w = 1.0;

    configure_goal_position(goal_msg);

    RCLCPP_INFO(get_logger(), "Dispatching goal to: %s, difficulty: %s (x=%.2f, y=%.2f)",
                target_location_.c_str(),
                difficulty_level_.c_str(),
                goal_msg.pose.pose.position.x, 
                goal_msg.pose.pose.position.y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoalNavigator::goal_result_callback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
}

void configure_goal_position(NavigateToPose::Goal &goal_msg){
    if (target_location_ == "living_room") {
        assign_position(goal_msg, 1.0, 1.0, 5.0, 2.0, -4.0, -1.0);
    } else if (target_location_ == "bathroom") {
        assign_position(goal_msg, 0.0, 4.0, -1.0, 4.0, 2.2, 3.9);
    } else if (target_location_ == "hallway") {
        assign_position(goal_msg, -3.0, -3.0, -3.0, -3.9, -3.5, -3.5);
    } else if (target_location_ == "kitchen") {
        assign_position(goal_msg, -0.5, -4.0, 3.5, -3.0, 4.5, -0.5);
    } else if (target_location_ == "bedroom") {
        assign_position(goal_msg, -3.0, 3.0, -2.2, 4.0, -4.5, 4.0);
    }
}

void assign_position(NavigateToPose::Goal &goal_msg,
                        double easy_x, double easy_y,
                        double medium_x, double medium_y,
                        double hard_x, double hard_y){
    if (difficulty_level_ == "easy") {
        goal_msg.pose.pose.position.x = easy_x;
        goal_msg.pose.pose.position.y = easy_y;
    } else if (difficulty_level_ == "medium") {
        goal_msg.pose.pose.position.x = medium_x;
        goal_msg.pose.pose.position.y = medium_y;
    } else if (difficulty_level_ == "hard") {
        goal_msg.pose.pose.position.x = hard_x;
        goal_msg.pose.pose.position.y = hard_y;
    }
}
```

Przykładowa komenda:
```
bash
ros2 run lab5 goal_navigator --ros-args -p destination:=bedroom -p difficulty:=hard
```

![alt text](lab5/images_zad/1.png)
![alt text](lab5/images_zad/2.png)
![alt text](lab5/images_zad/3.png)
![alt text](lab5/images_zad/4.png)
![alt text](lab5/images_zad/5.png)
![alt text](lab5/images_zad/6.png)
![alt text](lab5/images_zad/7.png)
![alt text](lab5/images_zad/8.png)
![alt text](lab5/images_zad/9.png)
![alt text](lab5/images_zad/10.png)
