# Одометрия Т4 для Unitree Go1

Распакуйте архив. Откройте терминал в папке с Dockerfile и каталогом T4.
Для Ubuntu x86_64 нужны Docker с Compose и Buildx, Git и Python 3. Для окна Gazebo нужен графический сеанс. Первый запуск скачивает образы.

```bash
xhost +si:localuser:"$(id -un)"
bash T4/start_gazebo.sh
```

Без окна выполните `bash T4/start_gazebo.sh --headless`. Если исходники симулятора уже скачаны, укажите их каталог через переменную GO1_SIM_DIR. Для локального образа симулятора задайте SIM_IMAGE. Скрипт собирает образ участника, запускает оба контейнера и остаётся в терминале до Ctrl+C.

## Проверка

Дождитесь Pair started. Во втором терминале войдите в контейнер симулятора, подставив его имя из строки Simulator= в первом терминале.

```bash
read -rp 'Имя контейнера Simulator: ' sim
docker exec -it "$sim" bash
```

Внутри контейнера:

```bash
source /opt/ros/humble/setup.bash
source /home/ros/go1_ws/install/setup.bash
source /etc/go1_sim_env.sh
ros2 node info /go1_t4_odometry
ros2 topic echo /odom --once --field pose.pose
```

Робот ждёт команд движения. Для ручной проверки в симуляторе подайте короткую серию команд, остановите робота и сравните позу с предыдущей:

```bash
ros2 topic pub -r 10 --times 30 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
ros2 topic echo /odom --once --field pose.pose
```

## Запуск ноды в собственном ROS 2 Humble workspace

Пакет находится в T4/src/go1_t4. Из каталога T4 на машине с ROS 2 Humble, colcon и настроенным rosdep:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select go1_t4
source install/setup.bash
ros2 run go1_t4 odometry --ros-args -p use_sim_time:=true
```

Этот вариант предполагает работающий симулятор в том же ROS_DOMAIN_ID и совместимые настройки DDS. При запуске через start_gazebo.sh нода уже работает.

Файл odom_node.py подписывается на /imu/data и /scan, публикует /odom и TF odom → trunk. Файл estimator.py сопоставляет сканы и оценивает смещение в плоскости.
