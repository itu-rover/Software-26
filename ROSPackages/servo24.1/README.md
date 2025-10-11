

---

# arm24_servo

arm24_servo, **MoveIt Servo** ile robot kolu için teleop ve **bilek singularity koruması** sağlayan bir ROS (Noetic) paketidir.

## Gereksinimler

- Ubuntu 20.04 + ROS Noetic
- Catkin workspace (`~/arm24_ws`)
- MoveIt / moveit_servo

## Kurulum

1. Paketi workspace'e alın:
   ```bash
   cd ~/arm24_ws/src
   git clone git@github.com:itu-rover/servo24.1.git arm24_servo
   ```

2. MoveIt Servo'yu yükleyin (APT ile, varsa):
   ```bash
   sudo apt update
   sudo apt install -y ros-noetic-moveit-servo ros-noetic-moveit
   ```
   Bulunamazsa kaynak kurulum:
   ```bash
   cd ~/arm24_ws/src
   git clone -b master https://github.com/ros-planning/moveit.git
   cd ~/arm24_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   catkin_make
   ```

3. Workspace'i derleyin:
   ```bash
   cd ~/arm24_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   catkin_make
   source ~/arm24_ws/devel/setup.bash
   ```

## Çalıştırma

1. Servo sunucusunu başlatın:
   ```bash
   roslaunch arm24_servo servo.launch
   ```
   - `servo.launch`, `config/naim24_servo.yaml` parametrelerini yükler.
   - Robotunuzun URDF/SRDF ve `robot_description` parametresi yüklü olmalı.

2. Teleop scriptini farklı bir terminalde çalıştırın:
   ```bash
   source ~/arm24_ws/devel/setup.bash
   rosrun arm24_servo twist_test.py
   ```
   - Tuşlar ve kullanım için script başındaki yardım metnine bakabilirsiniz.

## Yapılandırma

- `config/naim24_servo.yaml`: MoveIt Servo parametreleri
  - `move_group_name` (örn. manipulator)
  - `planning_frame` (örn. base_link)
  - `scale.linear` / `scale.rotary`
  - `twist_command_out_topic` (örn. /servo_server/delta_twist_cmds)
- `launch/servo.launch`: Robot tanımı ve MoveIt config yollarını kendi robotunuza göre ayarlayın.

## Dizin Yapısı

```
arm24_servo/
  ├─ launch/
  │   └─ servo.launch
  ├─ config/
  │   └─ naim24_servo.yaml
  ├─ scripts/
  │   └─ twist_test.py
  ├─ CMakeLists.txt
  ├─ package.xml
  └─ README.md
```


