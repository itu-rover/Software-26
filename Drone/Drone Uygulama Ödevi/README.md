# Drone Uygulama Ödevi

Bu ödevde, Gazebo ortamındaki bir quadrotor için temel bir otonom görev tamamlanacaktır.

Drone'un;

1. belirlenen irtifaya yükselmesi,
2. alanı taraması,
3. kırmızı hedefi kamera görüntüsünden tespit etmesi,
4. **ArUco ID 0** işaretçisini bulması,
5. işaretçinin üzerine hizalanarak kontrollü iniş yapması

beklenmektedir.

## Sistem Gereksinimleri

- Ubuntu 20.04 LTS
- ROS Noetic
- Gazebo 11


Bilgisayarınıza **Ubuntu 20.04** ve **ROS Noetic Desktop Full** kurunuz. ROS 2 veya farklı Ubuntu sürümleri bu ödev için desteklenmemektedir.

Gerekli ek paketler:

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy \
  ros-noetic-cv-bridge ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control
```

## Kurulum

Arşivi ev dizininize çıkardıktan sonra:

```bash
cd ~/Drone_Uygulama_Ödevi
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Düzenlenecek Dosya

Yalnızca aşağıdaki dosyada bulunan `TODO` bölümlerini tamamlayınız:

```text
src/drone_assignment/scripts/mission_template.py
```

Tamamlanması gereken bölümler:

- `TODO 1`: Kırmızı hedef tespiti
- `TODO 2`: ArUco ID 0 tespiti
- `TODO 3`: ArUco üzerine hizalanma ve iniş kontrolü

Verilen ROS düğüm yapısını, topic isimlerini, servis isimlerini ve görev durumlarını değiştirmeyiniz.

## Simülasyonu Çalıştırma

### Terminal 1

```bash
cd ~/Drone_Uygulama_Ödevi
source devel/setup.bash
roslaunch drone_assignment assignment_world.launch
```

### Terminal 2

```bash
cd ~/Drone_Uygulama_Ödevi
source devel/setup.bash
rosservice call /enable_motors "enable: true"
```

### Terminal 3

```bash
cd ~/Drone_Uygulama_Ödevi
source devel/setup.bash
rosrun drone_assignment mission_template.py
```

## Teslim

Aşağıdaki dosyayı tamamlanmış hâliyle teslim ediniz:

```text
src/drone_assignment/scripts/mission_template.py
```
