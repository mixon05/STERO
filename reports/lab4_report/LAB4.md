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
![alt text](rqttftree.png)

Uruchomiliśmy również powyższe polecenie z flagą `--force-discover`:

```bash
ros2 run rqt_tf_tree rqt_tf_tree --force-discover
```

Poniżej znajduje się zrzut ekranu.
![alt text](rqttftreeforcediscover.png)

## Uruchomienie `rqt_graph`

Uruchomiliśmy `rqt_graph` poleceniem:

```bash
ros2 run rqt_graph rqt_graph
```

Poniżej znajduje się zrzut ekranu.
![alt text](rosgraphAll.png)

Poniżej znajduje się zrzut ekranu tylko dla Nodes.
![alt text](rosgraphNodesOnly.png)

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
![alt text](pentagon.png)

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
![alt text](square.png)

## Architektura systemu robota-kelnera

Diagram odzwierciedla interakcje między klientem, obsługą i kuchnią, zapewniając jasny obraz kolejności działań w procesie obsługi w kawiarni. Robot-kelner zajmuje się odbieraniem oraz dostarczaniem zamówienia, przyjmuje też opłaty poprzez wbudowany terminal. Dzięki takiemu systemowi proces jest zautomatyzowany i nie wymaga żywych kelnerów.

![image](high_abstraction.png)

Poniżej znajduje się szczegółowy diagram architektury robota. Posiada on niezbędne komponenty do komunikacji werbalnej z klientem takie jak mikrofon oraz głośnik. Jest połączony również z siecią za pomocą sieci wifi, przez co nie musi jechać do kuchni aby złożyć zamówienie, otrzymuje również informacje o gotowych posiłkach przez co proces jest bardziej zautomatyzowany. Za pomocą kamery jest w stanie dostrzgać nowych klientów wchodzących do kawiarni i odbierać od nich zamówienia. Kluczową funkcją takiego robota jest oczywiście poruszanie się, które zapewnione jest przez napęd, oraz lidar, dzięki któremu robot wyznacza mapę i położenie oraz omija przeszkody.

![image](opd_diagram.png)
