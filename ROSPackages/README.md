## ROSPackages Klasörü

### arm23_config
Bu paket, robot kolunun workspace'inde bulunan başlangıç pozisyonu, simülasyonda çalışması için gerekli temel bilgiler gibi ayarları içerir.

### arm23_control
Robot kolunu sürmek için gerekli olan ve gömülü sistem ile iletişimi kurmaya yarayan temel kodları içeren pakettir.

### arm23_teleop
Joystick ile gömülü sistem arasında iletişimi sağlayan kodları ve ilgili launch dosyalarını içerir.

### bio_ik
Temel kinematik hesaplamaların yapıldığı pakettir.

### fiducials
Marker tabanlı lokalizasyon için gerekli ROS paketlerini içerir.

### naim23_urdf
Robotun ölçülerinin tanımlandığı URDF dosyasını ve ona bağlı diğer dosyaları içerir.

### ros_control_boilerplate
Robotun donanımı ile haberleşmede kullanılan hazır bir pakettir. Tek başına kullanılmaz, `arm23_control` paketi içerisinde kullanılır.

### rosparam_shortcuts
ROS projelerinde parametreleri kolay ve güvenli bir şekilde yüklemek için kullanılan bir C++ kütüphanesidir.

### inverse_kinematics
Eski kullanıcıların yazdığı kodlardan oluşan bir pakettir. Spesifik bir işlevi yoktur.

### ar_track_alvar
Aruco gibi görsel işaretçileri kamera görüntüsünden algılayıp konumlarını ROS'ta yayınlayan pakettir. (?)

### auto_nav
Spiral arama gibi otonom görevlerin behavior tree mantığını ve üst seviye görev akışını yöneten pakettir.

### gps_goal
Enlem ve boylam olarak verilen GPS hedeflerine gitmek için gerekli dönüşümleri ve kontrol mantığını içeren pakettir.

### imu_gps_ekf
robot_localization paketini kullanarak IMU ve GPS odometrisi gibi sensör verilerini Extended Kalman Filter (EKF) ile birleştiren ana lokalizasyon paketidir.

### liorf
Sistemin ana lokalizasyon yöntemi olmamasına rağmen, robotun 3D modelini TF ağacında yayınlamak için robot_state_publisher dosyasını barındıran pakettir.

### lora_comm 
Uzun menzilli LoRa haberleşme modülü üzerinden teleop komutlarını alan pakettir. (?)

### nmea_msgs
GPS alıcılarının kullandığı standart NMEA veri formatı için gerekli olan ROS mesaj tanımlarını içerir. (?)

### no_steering_urdf
Rover'ın fiziksel yapısını, boyutlarını, eklemlerini ve sensörlerin konumlarını tanımlayan 3D robot model (URDF) dosyalarını içeren pakettir.

### ouster-ros
Ouster marka Lidar sensöründen point cloud verisini alıp ROS'ta yayınlayan sürücü paketidir.

### robot_navigation
move_base kullanarak yol planlama, engelden kaçınma ve costmap gibi standart ROS navigasyon yeteneklerini yapılandıran pakettir.

### rover23_localization
Projeye özel odometri script'lerini, sensör konfigürasyonlarını ve lokalizasyonla ilgili launch dosyalarını içeren üst seviye pakettir.

### rover_23_control
Navigation stack'ten gelen hız komutlarını alıp aracın motorlarını süren VCU'ya gönderen alt seviye kontrol paketidir.

### rtcm_msgs
Hassas konumlandırma (DGPS/RTK) için kullanılan RTCM düzeltme verileri için gerekli olan ROS mesaj tanımlarını içerir. (?)

### sbg_ros_driver
SBG marka IMU'lar için standart driver ve bu cihazlara özel ROS mesaj (sbg_msgs) tanımlarını barındıran pakettir. (gerek olmama ihtimali var, artık imu verisi kendi scriptimiz ile alınıyor ancak zararı yok)

### ublox
U-blox marka GPS alıcısından veri alıp işleyerek EKF'nin ana girdilerinden biri olan /ublox/odom odometrisini üreten sürücü ve işlem paketidir.
