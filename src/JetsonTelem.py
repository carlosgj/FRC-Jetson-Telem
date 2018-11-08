#!/usr/bin/env python

import os
import glob
import rospy
from std_msgs.msg import String
from jetson_telem.msg import CPU, GPU, Thermal, Power

class TelemetryTracker():
    def __init__(self):
        self.lastCPUTotals = None
        self.lastCPULoads = None
        self.cpuCount = self.getCPUCount()
        self.gpuCount = self.getGPUCount()

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

    def getPowerSysfsPath(self):
        basedir = None
        if os.path.exists("/sys/devices/7000c400.i2c/i2c-1/1-0040/iio_device"):
            #This one works for me
            basedir = "/sys/devices/7000c400.i2c/i2c-1/1-0040/iio_device"
        elif os.path.exists("/sys/devices/platform/7000c400.i2c/i2c-1/1-0040/iio_device/"):
            #Specified in https://devtalk.nvidia.com/default/topic/950341/jetson-tx1/jetson-tx1-ina226-power-monitor-with-i2c-interface-/post/4998393/
            basedir = "/sys/devices/platform/7000c400.i2c/i2c-1/1-0040/iio_device/"
        else:
            rospy.logwarn("Couldn't find power sysfs nodes.")

        return basedir


    def getPowerBusNames(self):
        names = []
        basedir = self.getPowerSysfsPath()
        if basedir:
            for i in range(3):
                with open(os.path.join(basedir, "rail_name_%d"%i)) as file:
                    name = file.read().strip()
                    names.append(name)
        return names

    def getPowerBusVoltages(self):
        voltages = []
        basedir = self.getPowerSysfsPath()
        if basedir:
            for i in range(3):
                with open(os.path.join(basedir, "in_voltage%d_input"%i)) as file:
                    voltageStr = file.read().strip()
                    voltage = float(voltageStr)
                    voltage /= 1000.
                    voltages.append(voltage)
        return voltages

    def getPowerBusCurrents(self):
        currents = []
        basedir = self.getPowerSysfsPath()
        if basedir:
            for i in range(3):
                with open(os.path.join(basedir, "in_current%d_input"%i)) as file:
                    currentStr = file.read().strip()
                    current = float(currentStr)
                    current /= 1000.
                    currents.append(current)
        return currents

    def getGPUCount(self):
        files = glob.glob("/sys/devices/gpu.*")
        return len(files)

    def getGPULoads(self):
        loads = []
        for i in range(self.gpuCount):
            with open("/sys/devices/gpu.%d/load"%i) as file:
                loadStr = file.read()
                load = float(loadStr)
                load /= 1000 #Convert to real (not percentage)
                loads.append(load)
        return loads

    def getGPUSpeeds(self):
        speeds = []
        for i in range(self.gpuCount):
            with open("/sys/devices/gpu.%d/devfreq/57000000.gpu/cur_freq"%i) as file:
                speedStr = file.read()
                speed = float(speedStr)
                speeds.append(speed)
        return speeds

def talker():
    cpupub = rospy.Publisher('jetson/cpu', CPU, queue_size=10)
    thermpub = rospy.Publisher('jetson/thermal', Thermal, queue_size=10)
    powerpub = rospy.Publisher('jetson/power', Power, queue_size=10)
    gpupub = rospy.Publisher('jetson/gpu', GPU, queue_size=10)
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

        busNames = this.getPowerBusNames()
        busVoltages = this.getPowerBusVoltages()
        busCurrents = this.getPowerBusCurrents()
        rospy.loginfo("Power rail names: %s"%busNames)
        rospy.loginfo("Power rail voltages: %s"%busVoltages)
        rospy.loginfo("Power rail currents: %s"%busCurrents)
        thisPowerMsg = Power()
        thisPowerMsg.PowerBusNames = busNames
        thisPowerMsg.PowerBusVoltages = busVoltages
        thisPowerMsg.PowerBusCurrents = busCurrents
        powerpub.publish(thisPowerMsg)

        gpuUsages = this.getGPULoads()
        gpuSpeeds = this.getGPUSpeeds()
        rospy.loginfo("GPU speeds: %s"%gpuSpeeds)
        rospy.loginfo("GPU usages: %s"%gpuUsages)
        thisGPUMsg = GPU()
        thisGPUMsg.GPUspeeds = gpuSpeeds
        thisGPUMsg.GPUusages = gpuUsages
        gpupub.publish(thisGPUMsg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
