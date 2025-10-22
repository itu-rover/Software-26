#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
import math

class WristGuard:
    def __init__(self):
        # Parametreler (radyan!)
        self.joint_name   = rospy.get_param("~joint_name", "joint5")
        self.pref_sign    = float(rospy.get_param("~pref_sign",  1.0))   # +1: pozitif kır, -1: negatif kır
        self.target_mag   = float(rospy.get_param("~target_mag", 0.35))  # hedef kırıklık (rad)
        self.eps_enter    = float(rospy.get_param("~eps_enter",  0.25))  # koridor giriş eşiği (rad)
        self.eps_exit     = float(rospy.get_param("~eps_exit",   0.35))  # koridor çıkış eşiği (rad)
        self.max_rad_s    = float(rospy.get_param("~max_rad_s",  0.20))  # en fazla rad/s
        self.gain         = float(rospy.get_param("~gain",       2.0))   # basit P kazancı
        self.scale_joint  = float(rospy.get_param("~scale_joint",0.5))   # YAML'daki scale.joint ile aynı olmalı
        self.pub_rate     = float(rospy.get_param("~pub_rate",   50.0))  # Hz

        self.q5 = 0.0
        self.have_q5 = False
        self.active = False  # histerezis için

        rospy.Subscriber("joint_states", JointState, self.cb_js, queue_size=10)
        self.pub = rospy.Publisher("/servo_server/delta_joint_cmds", JointJog, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0/self.pub_rate), self.on_timer)
        rospy.loginfo("wrist_guard running (joint=%s, target=%.2f rad, enter=%.2f, exit=%.2f)",
                      self.joint_name, self.pref_sign*self.target_mag, self.eps_enter, self.eps_exit)

    def cb_js(self, msg: JointState):
        if not msg.name:
            return
        try:
            idx = msg.name.index(self.joint_name)
            self.q5 = msg.position[idx]
            self.have_q5 = True
        except ValueError:
            pass

    def on_timer(self, _evt):
        if not self.have_q5:
            return

        # Histerezis: yakınsa aktive et, yeterince uzaksa kapat
        if not self.active and abs(self.q5) < self.eps_enter:
            self.active = True
        elif self.active and abs(self.q5) > self.eps_exit:
            self.active = False

        if not self.active:
            return  # hiçbir şey yayınlama

        # Hedef açı: tercih edilen yönde sabit kırıklık
        q_target = self.pref_sign * self.target_mag
        err = q_target - self.q5
        v_rad_s = max(-self.max_rad_s, min(self.gain * err, self.max_rad_s))  # saturasyon

        # unitless'a çevir (±1)
        u = v_rad_s / max(self.scale_joint, 1e-6)
        u = max(-1.0, min(u, 1.0))

        jj = JointJog()
        jj.header.stamp = rospy.Time.now()
        jj.header.frame_id = "arm_base"   # fark etmez; boş da olabilir
        jj.joint_names = [self.joint_name]
        jj.velocities  = [u]              # unitless hız
        jj.displacements = [0.0]          # kullanmıyoruz ama dolduralım
        self.pub.publish(jj)

if __name__ == "__main__":
    rospy.init_node("wrist_guard")
    WristGuard()
    rospy.spin()
