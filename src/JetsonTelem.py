#!/usr/bin/env python

import os

import rospy
from std_msgs.msg import String
from jetson_telem.msg import CPU, Thermal

class TelemetryTracker():
    def __init__(self):
        self.lastCPUTotals = None
        self.lastCPULoads = None
        self.cpuCount = self.getCPUCount()
    def getCPUCount(self):
        cpucount = 0
        with open("/proc/cpuinfo") as file:
            lines = file.readlines()
            for line in lines:
                if line.startswith("processor"):
                    cpucount += 1
        return cpucount

    def getCPUSpeeds(self):
        cpuspeeds = []
        for i in range(self.cpuCount):
            with open("/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq"%i) as file:
                txt = file.read()
                try:
                    num = float(txt)
                except ValueError:
                    num = float('NaN')
                cpuspeeds.append(num)
        return cpuspeeds

    def getCPUUsages(self):
        cpuusages = []
        firstRunFlag = False
        if self.lastCPULoads is None:
            self.lastCPULoads = [0]*self.cpuCount
            self.lastCPUTotals = [0]*self.cpuCount
            firstRunFlag = True
        with open("/proc/stat") as file:
            lines = file.readlines()
            for i in range(self.cpuCount):
                thisLine = lines[i+1]
                chunks = thisLine.split(' ')
                [user, nice, system, idle, iowait, irq, softirq] = [int(x) for x in chunks[1:8]]
                total = user + nice + system + idle + iowait + irq + softirq
                load = total - idle
                if not firstRunFlag:
                    try:
                        thisUsage = float((load-self.lastCPULoads[i]))/float((total-self.lastCPUTotals[i]))
                    except ZeroDivisionError:
                        thisUsage = 0.
                    cpuusages.append(thisUsage)
                self.lastCPUTotals[i] = total
                self.lastCPULoads[i] = load
        if firstRunFlag:
            return [float('NaN')]*self.cpuCount
        return cpuusages

    def getThermalZones(self):
        thermalZones = []
        items = os.listdir("/sys/devices/virtual/thermal/")
        for item in items:
            if item.startswith("thermal_zone"):
                thermalZones.append(item)
        return thermalZones


    def getThermalZoneTypes(self):
        zoneTypes = []
        zones = self.getThermalZones()
        zones.sort()
        for zone in zones:
            zType = "???"
            with open("/sys/devices/virtual/thermal/%s/type"%zone) as file:
                zType = file.read().strip()
            zoneTypes.append(zType)
        return zoneTypes

    def getThermalZoneTemps(self):
        zoneTemps = []
        zones = self.getThermalZones()
        zones.sort()
        for zone in zones:
            zTemp = "NaN"
            with open("/sys/devices/virtual/thermal/%s/temp"%zone) as file:
                zTempStr = file.read().strip()
                zTempRaw = float(zTempStr)
                zTemp = zTempRaw/1000.
            zoneTemps.append(zTemp)
        return zoneTemps

def talker():
    cpupub = rospy.Publisher('jetson/cpu', CPU, queue_size=10)
    thermpub = rospy.Publisher('jetson/thermal', Thermal, queue_size=10)
    rospy.init_node('jetson', anonymous=True)
    rate = rospy.Rate(5)
    this = TelemetryTracker()
    while not rospy.is_shutdown():
        speeds = this.getCPUSpeeds()
        usages = this.getCPUUsages()
        rospy.loginfo("CPU speeds: %s"%str(speeds))
        rospy.loginfo("CPU usages: %s"%str(usages))

        thisCPUmsg = CPU()
        thisCPUmsg.CPUspeeds = speeds
        thisCPUmsg.CPUusages = usages
        cpupub.publish(thisCPUmsg)

        tZoneTypes = this.getThermalZoneTypes()
        tZoneTemps = this.getThermalZoneTemps()
        rospy.loginfo("Thermal zone types: %s"%tZoneTypes)
        rospy.loginfo("Thermal zone temps: %s"%tZoneTemps)
        thisThermMsg = Thermal()
        thisThermMsg.ThermalZoneNames = tZoneTypes
        thisThermMsg.ThermalZoneTemps = tZoneTemps
        thermpub.publish(thisThermMsg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
