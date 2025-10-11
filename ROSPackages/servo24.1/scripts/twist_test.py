#!/usr/bin/env python3
import sys, select, termios, tty, math
import rospy
from geometry_msgs.msg import TwistStamped

HELP = """
Twist Teleop (MoveIt Servo - unitless)

Linear (x,y,z):
  w/s : +x / -x
  a/d : +y / -y
  r/f : +z / -z

Angular (roll x, pitch y, yaw z):
  u/o : +roll / -roll
  i/k : +pitch / -pitch
  j/l : +yaw   / -yaw

Hız ayarı:
  =   : adım (+)
  -   : adım (-)
 
Diğer:
  space : tüm komutları sıfırla
  h     : yardım yazdır
  q     : çıkış

Not: MODE=unitless → değerler [-1,1]. YAML’de command_in_type=unitless olmalı.
"""

def get_key(timeout=0.02):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

def clamp(x, lo, hi): return max(lo, min(hi, x))

def main():
    rospy.init_node("twist_teleop")
    topic    = rospy.get_param("~topic", "/servo_server/delta_twist_cmds")
    frame_id = rospy.get_param("~frame_id", "arm_base")
    rate_hz  = float(rospy.get_param("~rate", 100.0))

    # mode: "unitless" ([-1,1]) ya da "speed_units" (m/s, rad/s).
    mode     = rospy.get_param("~mode", "unitless").lower()

    # unitless için step, speed_units için mutlak max hızlar (yine de step kullanıyoruz)
    step            = float(rospy.get_param("~step", 0.03))   # her tuşta değişim
    lin_max_mps     = float(rospy.get_param("~lin_max", 0.25))
    ang_max_radps   = float(rospy.get_param("~ang_max", 0.60))
    decay           = float(rospy.get_param("~decay", 0.90))  # tuş gelmezse sönüm

    pub = rospy.Publisher(topic, TwistStamped, queue_size=50)
    rate = rospy.Rate(rate_hz)

    # hız hedefleri (unitless veya speed_units’e göre)
    lx=ly=lz=0.0
    ax=ay=az=0.0

    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print(HELP)
    print("Publishing to %s @ %s Hz | frame_id=%s | MODE=%s" % (topic, rate_hz, frame_id, mode.upper()))

    try:
        while not rospy.is_shutdown():
            key = get_key(1.0/rate_hz)

            if key:
                if   key == 'w': lx += step
                elif key == 's': lx -= step
                elif key == 'a': ly += step
                elif key == 'd': ly -= step
                elif key == 'r': lz += step
                elif key == 'f': lz -= step

                elif key == 'u': ax += step
                elif key == 'o': ax -= step
                elif key == 'i': ay += step
                elif key == 'k': ay -= step
                elif key == 'j': az += step
                elif key == 'l': az -= step

                elif key == '=': step = min(0.5, step + 0.01); print("step=%.3f" % step)
                elif key == '-': step = max(0.01, step - 0.01); print("step=%.3f" % step)

                elif key == ' ':  # stop
                    lx=ly=lz=ax=ay=az=0.0
                    print("STOP")

                elif key == 'h':
                    print(HELP.strip())

                elif key == 'q':
                    break

                # clamp
                if mode == "unitless":
                    lx = clamp(lx, -1.0, 1.0); ly = clamp(ly, -1.0, 1.0); lz = clamp(lz, -1.0, 1.0)
                    ax = clamp(ax, -1.0, 1.0); ay = clamp(ay, -1.0, 1.0); az = clamp(az, -1.0, 1.0)
                else:
                    lx = clamp(lx, -lin_max_mps, lin_max_mps)
                    ly = clamp(ly, -lin_max_mps, lin_max_mps)
                    lz = clamp(lz, -lin_max_mps, lin_max_mps)
                    ax = clamp(ax, -ang_max_radps, ang_max_radps)
                    ay = clamp(ay, -ang_max_radps, ang_max_radps)
                    az = clamp(az, -ang_max_radps, ang_max_radps)

            else:
                # tuş yoksa sönümle (yumuşak bırakma)
                lx *= decay; ly *= decay; lz *= decay
                ax *= decay; ay *= decay; az *= decay

                # çok küçükse sıfırla
                if abs(lx) < 1e-3: lx = 0.0
                if abs(ly) < 1e-3: ly = 0.0
                if abs(lz) < 1e-3: lz = 0.0
                if abs(ax) < 1e-3: ax = 0.0
                if abs(ay) < 1e-3: ay = 0.0
                if abs(az) < 1e-3: az = 0.0

            # TwistStamped doldur
            msg = TwistStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frame_id

            if mode == "unitless":
                msg.twist.linear.x  = lx
                msg.twist.linear.y  = ly
                msg.twist.linear.z  = lz
                msg.twist.angular.x = ax
                msg.twist.angular.y = ay
                msg.twist.angular.z = az
            else:
                # speed_units: doğrudan m/s ve rad/s
                msg.twist.linear.x  = lx
                msg.twist.linear.y  = ly
                msg.twist.linear.z  = lz
                msg.twist.angular.x = ax
                msg.twist.angular.y = ay
                msg.twist.angular.z = az

            pub.publish(msg)
            rate.sleep()

    finally:
        # çıkarken sıfır komut yayınla
        z = TwistStamped()
        z.header.stamp = rospy.Time.now()
        z.header.frame_id = frame_id
        pub.publish(z)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
        print("\nbye.")

if __name__ == "__main__":
    main()
