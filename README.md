# FRC-Jetson-Telem

This is a [ROS](http://www.ros.org/) package designed to publish topics regarding "housekeeping" information (CPU/GPU speeds, utilization, temperatures, power consumption, etc.) for an NVIDIA Jetson (TK1/TX1/TX2). 

Note that I work at JPL, and as such approach telemetry with a JPL philosophy. That is, no information regarding warnings or limit violations will be published; only the quantitative data. Determining if a telemetry point is in a danger zone, and alerting the operators of such, is the responsibility of the "ground system" (i.e., whatever monitoring devices are subscribed to the telemetry topics). 
