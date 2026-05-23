# Symulacja Robota w ROS2

## Uruchomienie symulacji

Uruchomiliśmy symulację robota przy użyciu polecenia:

```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py navigation:=True moveit:=True is_public_sim:=True use_grasp_fix_plugin:=True world_name:=empty
```

## Instalacja i uruchomienie `rqt_tf_tree`

Zainstalowaliśmy `rqt_tf_tree` i uruchomiliśmy je poleceniem:

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

Poniżej znajduje się zrzut ekranu.
![alt text](lab4_report/rqttftree.png)

Uruchomiliśmy również powyższe polecenie z flagą `--force-discover`:

```bash
ros2 run rqt_tf_tree rqt_tf_tree --force-discover
```

Poniżej znajduje się zrzut ekranu.
![alt text](./lab4_report/rqttftreeforcediscover.png)

## Uruchomienie `rqt_graph`

Uruchomiliśmy `rqt_graph` poleceniem:

```bash
ros2 run rqt_graph rqt_graph
```

Poniżej znajduje się zrzut ekranu.
![alt text](lab4_report/rosgraphAll.png)

Poniżej znajduje się zrzut ekranu tylko dla Nodes.
![alt text](lab4_report/rosgraphNodesOnly.png)

## Analiza tematów i węzłów

W kolejnych zadaniach przeanalizowaliśmy publikowane i subskrybowane tematy oraz węzły przy użyciu poleceń:

```bash
ros2 topic list
ros2 node list
```

Odpowiedzi powyższych komend znajdują się w plikach topic_list.txt oraz node_list.txt.

Uruchomiliśmy również polecenie:

```bash
ros2 doctor --report
```

Opdpowiedź tej komendy znajduje się w pliku ros_doctor.txt.

Polecenie to pozwala na diagnostykę w systemie ROS2 i wygenerowanie raportu.

## Identyfikacja tematów sterowania i sensorów robota

- **Sterowanie prędkością bazy**:
  - `/mobile_base_controller/cmd_vel_unstamped`
- **Odometria**:
  - `/mobile_base_controller/odom`
- **LIDAR**:
  - `/scan_raw`
- **Kamera**:
  - `/head_front_camera/depth_registered/image_raw`
  - `/head_front_camera/rgb/image_raw`

## Symulacja rysowania pięciokąta

Na laboratorium mieliśmy do zaimplementowania symulację rysowania pięciokąta. Głównym celem było dobranie tak parametrów skręcania robota, aby kąt obrotu był odpowiedni dla pięciokąta. Na początku spróbowaliśmy zaimplementować model idealny, aby robot obracał się o (2PI-2PI/5) na każdym wierzchołku. Jednak z powodu realnego symulatora (rozpędzania się robota oraz występujących tarć i opóźnień) musieliśmy tą wartość modyfikować tak aby narysowany został pięciokąt. 

```
bash
void move_robot()
{
    auto current_time = this->now();
    double delta_time = (current_time - last_time_).seconds() * 1000; // Oblicz rzeczywisty czas od ostatniego wywołania
    last_time_ = current_time;

    auto message = geometry_msgs::msg::Twist();

    if (current_state_ == MotionState::MOVE_FORWARD)
    {
        message.linear.x = linear_speed_;
        message.angular.z = 0.0;

        elapsed_time_ += delta_time;
        if (elapsed_time_ >= move_duration_)
        {
            elapsed_time_ = 0.0;
            current_state_ = MotionState::STOP;
            stop_next_turn_ = true; // Przejdź do STOP przed obrotem
            RCLCPP_INFO(this->get_logger(), "Stopping after moving forward.");
        }
    }
    else if (current_state_ == MotionState::TURN)
    {
        message.linear.x = 0.0;
        message.angular.z = angular_speed_;

        elapsed_time_ += delta_time;
        if (elapsed_time_ >= turn_duration_)
        {
            elapsed_time_ = 0.0;
            current_state_ = MotionState::STOP;
            stop_next_turn_ = false; // Przejdź do STOP po obrocie
            RCLCPP_INFO(this->get_logger(), "Stopping after turning.");

            ++completed_sides_;
        }
    }
    else if (current_state_ == MotionState::STOP)
    {
        message.linear.x = 0.0;
        message.angular.z = 0.0;

        elapsed_time_ += delta_time;
        if (elapsed_time_ >= stop_duration_)
        {
            elapsed_time_ = 0.0;
            if (stop_next_turn_)
            {
                current_state_ = MotionState::TURN;
                RCLCPP_INFO(this->get_logger(), "Turning robot.");
            }
            else
            {
                current_state_ = MotionState::MOVE_FORWARD;
                RCLCPP_INFO(this->get_logger(), "Moving robot forward.");
            }
        }
    }

    publisher_->publish(message);
}

enum class MotionState
{
    MOVE_FORWARD,
    TURN,
    STOP
};
```

### Zadanie 2

Uruchomienie:

```bash
ros2 run lab4 lab4_hello_moveit
```

Poniżej znajduje się zrzut ekranu z Rviz dla rysowanej figury.
![alt text](lab4_report/pentagon.png)

### Projekt 1

Przy wykonywaniu projektu bazowaliśmy na kodzie z laboratorium dla pięciokąta. Zmieniliśmy parametry i dodaliśmy obsługę danych wejściowych z pliku launch. Dodaliśmy również obliczanie błędów zbieranych z tematu symulowanego sensora oraz dane referencyjne z symulatora. Program po zakończeniu działania generuje tablicę w pliku .csv ze wszystkich zebranych danych oraz na podstawie tych danych oblicza błąd średniokwadratowy położenia i orientacji.

```
bash
void record_errors(){
    double pos_error = std::sqrt(std::pow((cur_x_odom - cur_x_odom2), 2) +
                                  std::pow((cur_y_odom - cur_y_odom2), 2));
    double yaw_error = std::fabs(angle_error(current_yaw1, current_yaw2));

    pos_errors_.push_back(pos_error);
    yaw_errors_.push_back(yaw_error);
}

void sum_loop_errors(){
    double pos_sum = 0.0;
    double yaw_sum = 0.0;
    for (auto e : pos_errors_) pos_sum += e*e;
    for (auto e : yaw_errors_) yaw_sum += e*e;

    double pos_rms = std::sqrt(pos_sum / pos_errors_.size());
    double yaw_rms = std::sqrt(yaw_sum / yaw_errors_.size());

    RCLCPP_INFO(this->get_logger(), "After loop %d: Position RMS=%.4f, Yaw RMS=%.4f",
                completed_sides_, pos_rms, yaw_rms);
}

void sum_all_errors(){
    double pos_sum = 0.0;
    double yaw_sum = 0.0;
    for (auto e : pos_errors_) pos_sum += e*e;
    for (auto e : yaw_errors_) yaw_sum += e*e;

    double pos_rms = std::sqrt(pos_sum / pos_errors_.size());
    double yaw_rms = std::sqrt(yaw_sum / yaw_errors_.size());

    RCLCPP_INFO(this->get_logger(), "Total: Position RMS=%.4f, Yaw RMS=%.4f",
                pos_rms, yaw_rms);

    std::ofstream ofs("errors_report.csv");
    ofs << "pos_error,yaw_error\n";
    for (size_t i=0; i<pos_errors_.size(); i++){
        ofs << pos_errors_[i] << "," << yaw_errors_[i] << "\n";
    }
    ofs.close();

    RCLCPP_INFO(this->get_logger(), "Errors saved to errors_report.csv");
}
```

Uruchomienie:

```bash
ros2 launch lab4 square.launch.py
```

Poniżej znajduje się zrzut ekranu dla rysowanego kwadratu.
![alt text](lab4_report/square.png)

## Architektura systemu robota-kelnera

Diagram odzwierciedla interakcje między klientem, obsługą i kuchnią, zapewniając jasny obraz kolejności działań w procesie obsługi w kawiarni. Robot-kelner zajmuje się odbieraniem oraz dostarczaniem zamówienia, przyjmuje też opłaty poprzez wbudowany terminal. Dzięki takiemu systemowi proces jest zautomatyzowany i nie wymaga żywych kelnerów.

![image](lab4_report/high_abstraction.png)

Poniżej znajduje się szczegółowy diagram architektury robota. Posiada on niezbędne komponenty do komunikacji werbalnej z klientem takie jak mikrofon oraz głośnik. Jest połączony również z siecią za pomocą sieci wifi, przez co nie musi jechać do kuchni aby złożyć zamówienie, otrzymuje również informacje o gotowych posiłkach przez co proces jest bardziej zautomatyzowany. Za pomocą kamery jest w stanie dostrzgać nowych klientów wchodzących do kawiarni i odbierać od nich zamówienia. Kluczową funkcją takiego robota jest oczywiście poruszanie się, które zapewnione jest przez napęd, oraz lidar, dzięki któremu robot wyznacza mapę i położenie oraz omija przeszkody.

![image](lab4_report/opd_diagram.png)

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


# Projekt 2 - STERO

Wykonywanie projektu rozpoczęliśmy od zmiany języka programowania na Python i skonfigurowaniu pakietu. Po utworzeniu wymaganych plików utworzyliśmy dwa pliki, potrzebne do rozwoju w kolejnych krokach: action_client.py oraz action_server.py. 

Następnie stworzyliśmy w folderze src całego repozytorium nowy pakiet dla nowego interfejsu akcji. Następnie w utworzonym pakiecie my_interfaces w katalogu action dodaliśmy plik Interface.action z poniższą tręścią:
```
float32[] route
---
int32 result
---
float32 progress
```

Następnie zaktualizowaliśmy plik CMakeLists.txt w naszym pakiecie project2 w celu uwzględnienia nowego interfejsu w kompilacji. 

W kolejnym kroku przeszliśmy do implementacji serwera akcji. Na bazie tutorialu Fibonaciego z dokumentacji ros stworzyliśmy szkielet komunikacji pomiędzy programami action_server.py oraz action_client.py. Następnie dodaliśmy przetwarzanie parametrów wejściowych od użytkownika w kliencie i przesyłanie ich na akcji 'navigate_action'. Po wysłaniu celu do serwera, klient nasłuchiwał na odpowiedź, która następnie jest przetwarzana w process_goal_response. W tej funkcji wypisywana jest informacja czy cel został zaakceptowany. Jeżeli został to klient dalej nasłuchuje na odpowiedź od serwera i jeżeli ta nadejdzie to obsługuje ją w funkcji process_result która na bazie przyjętych kodów komunikacji informuje jak dobrze udało się wykonać polecenie lub jakie błędy wystąpiły. Jednocześnie w tle jest wyświetlany status wykonania całości polecenia w funkcji handle_feedback_update. Poniżej znajduje się część kodu klienta:

```
def send_navigation_goal(self, route):
    goal_message = Interface.Goal()
    goal_message.route = route

    self.action_client.wait_for_server()

    self.future_goal = self.action_client.send_goal_async(
        goal_message, 
        feedback_callback=self.handle_feedback_update
    )
    self.future_goal.add_done_callback(self.process_goal_response)

def process_goal_response(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        self.get_logger().warning('The goal was not accepted by the action server.')
        return

    self.get_logger().info('The goal has been accepted.')
    self.future_result = goal_handle.get_result_async()
    self.future_result.add_done_callback(self.process_result)

def process_result(self, future):
    result_code = int(future.result().result.result)
    result_messages = {
        0: 'Goal execution succeeded!',
        1: 'The route is empty.',
        2: 'No initial pose detected.',
        3: 'Goal was canceled.',
        4: 'Goal execution failed.',
        5: 'Invalid return status from the server.'
    }

    result_message = result_messages.get(result_code, 'Received an unknown result code.')
    self.get_logger().info(f'Result: {result_message}')

    rclpy.shutdown()

def handle_feedback_update(self, feedback_msg):
    feedback = feedback_msg.feedback
    self.get_logger().info(f'Progress: {round(feedback.progress, 2)}% complete.')
```

W serwerze, który pełni trochę ważniejszą rolę nasłuchujemy komend na temacie navigate_action i gdy ta nadejdzie przetwarzamy ją w funkcji handle_goal_execution. W tej funkcji wykorzystujemy wcześniej utworzony interfejs do przechowywania danych o końcowym rezultacie i wykonanym progressie. Do nawigacji wykorzystujemy BasicNavigator z biblioteki nav2_simple_commander. Pozwala on na wyznaczenie trasy po punktach przy użyciu funkcji getPathThroughPoses z tej biblioteki, następnie możemy np. policzyć długość trasy, obliczając sumę kolejnych kawałków trasy korzystając z twierdzenia Pitagorasa. Później publikowany w pętli jest progress z częstotliwością 1.5 sekundy. Poniżej znajduje się kod przetwarzania zapytań od klienta:

```
def handle_goal_execution(self, goal_handle):
    self.is_active = True
    self.rideDist = 0
    self.get_logger().info('Starting navigation task')

    result = Interface.Result()

    if not goal_handle.request.route:
        result.result = 1.0
        self.is_active = False
        return result

    if not self.init_pose:
        result.result = 2.0
        self.is_active = False
        return result

    self.navigator.setInitialPose(self.init_pose)
    self.navigator.waitUntilNav2Active()

    self.setup_navigation_route(goal_handle.request.route)

    pathPoses = self.navigator.getPathThroughPoses(start=self.init_pose, goals=self.routes)
    pathDist = calculate_path_distance(pathPoses)
    self.get_logger().info(f'Total Path Length: {pathDist}')

    self.navigator.followWaypoints(self.routes)

    feedback = Interface.Feedback()
    while not self.navigator.isTaskComplete():
        nav_feedback = self.navigator.getFeedback()
        if nav_feedback:
            feedback.progress = round((self.rideDist / pathDist) * 100, 2)
            self.get_logger().info(f'Progress: {feedback.progress}%')
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)

    return self.finalize_task(goal_handle)
```

Dodaliśmy również opcję obracania głowy robota. Jest ona zaimplementowana w funkcji publish_head_trajectory, która jest wywoływana przez funkcję process_odometry, która działa na danych z tematu ground_truth_odom. Kąt obrotu głowy bazuje na warunku:
```
angle = max(min(self.current_z_twist, math.pi), -math.pi)
```
Kąt jest publikowany na temacie head_controller/joint_trajectory. Poniżej kod:
```
def publish_head_trajectory(self):
    angle = max(min(self.current_z_twist, math.pi), -math.pi)
    trajectory_msg = JointTrajectory(
        joint_names=['head_1_joint', 'head_2_joint'],
        points=[JointTrajectoryPoint(
            positions=[angle, 0.0],
            time_from_start=Duration(seconds=0.2).to_msg()
        )]
    )
    self.trajectory_publisher.publish(trajectory_msg)
```

Poniżej znajdują się zrzuty ekranu z działania.
![alt text](project2/images/1.png)
![alt text](project2/images/2.png)
![alt text](project2/images/3.png)



# Laboratorium 6

W trakcie laboratorium przeprowadziliśmy eksperymenty wykrywające prawdopodobne położenie robota. Im bliżej robot znajduje się wykrytego obiektu, w tym wypadku ściany. Im robot znajduje się bliżej ściany tym algorytm łatwiej jest w stanie obliczyć jego prawdopodobne położenie.

![image](carlo_wall.png)

Gdy robot nie będzie widzieć w pobliżu żadnych obiektów, jego prawdopodobne położenie jest bardzo płaskie, algorytm nie jest w stanie określić dokładnego położenia.

![image](carlo_no_wall.png)
