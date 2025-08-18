#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#Author @Umut Baran Akyol
from pexpect import pxssh
import rospy
from std_msgs.msg import String
import subprocess

class StatusData:
    def __init__(self):
        rospy.loginfo("AAAAA")
        self.freq = 0
        self.channel = 0
        self.signal = 0
        self.ping = 0
        self.ccq = 0
        self.usageRAM = 0
        self.usageCPU = 0
        self.tempCPU = 0
        self.ipList = list()
        rospy.init_node("status_publisher")
        self.rate = rospy.Rate(0.5)
        self.s = pxssh.pxssh()
        self.pub = rospy.Publisher("/pc_status", String, queue_size=10)

        self.connect()

    def connect(self):
        self.success = self.s.login ('192.168.1.20', 'ubnt', '123123')
        if not self.success:
            print("SSH session failed on login.")
        else:
            print("SSH session login successful")
        self.getData()

    def getIpList(self):
        self.ipList.clear()
        arpoutput = subprocess.getoutput("arp -n").splitlines()
        for i in arpoutput:
            arplist = i.split(" ")
            arplist = [i for i in arplist if i]
            # print(arplist)
            adress = arplist[0]
            if(arplist[-1] == "eth0" and arplist[1] != "(incomplete)"):
                pingOutput = subprocess.getoutput("ping "+ arplist[0] +" -c 1 -w 0| grep time=")
                if(pingOutput != ""):
                    self.ipList.append(arplist[0])

    def getData(self):
        while not rospy.is_shutdown():
            self.getIpList()
            self.s.sendline('mca-status')
            self.s.prompt()
            self.data = self.s.before.decode(encoding='UTF-8',errors='strict').split()
            self.freq = self.data[10].split("=")[1]
            self.signal = self.data[14].split("=")[1]
            self.ccq = self.data[18].split("=")[1]
            pingOutput = subprocess.getoutput("ping "+ self.ipList[0] +" -c 1 | grep time=")
            statsOutput = subprocess.getoutput("tegrastats | head -n 1")
            self.usageRAM = int(statsOutput.split(" ")[3].split("/")[0]) / int(statsOutput.split(" ")[3].split("/")[1].split("MB")[0]) * 100
            cpudata = statsOutput.split(" ")[11].split(",")[1:][:-1]
            self.usageCPU = 0
            for i in cpudata:
                coredata = i.split("%")[0]
                self.usageCPU += int(coredata) / 6
            self.tempCPU = statsOutput.split(" ")[17].split("@")[1].split("C")[0]
            self.ping = pingOutput.split("time=")[1].split(" ")[0]
            self.status = "Frequency: " + self.freq + ",Signal: " + self.signal + ",CCQ: " + self.ccq + ",Ping: " + self.ping + ",Master IP: " + self.ipList[0]+ ",Connected IPs: " + str(self.ipList)+ ",CPU Usage: " + str(self.usageCPU) + ",CPU Temp: " + str(self.tempCPU) + ",RAM Usage: " + str(self.usageRAM)  
            self.pub.publish(self.status)
            self.rate.sleep()

    def close(self):
        self.s.logout()
        print("Closed")

if __name__ == '__main__':
    try:
        StatusData()
    except rospy.ROSInterruptException:
        print("a")
        
