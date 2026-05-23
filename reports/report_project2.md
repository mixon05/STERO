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

