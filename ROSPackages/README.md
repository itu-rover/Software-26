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
