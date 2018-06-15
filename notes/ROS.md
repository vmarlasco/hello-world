# ROS msg and srv

## Intro
* **msg:** msg file are txt files that describes the files of a ROS message.
* **srv:** srv file describes a service. It is composed of two parts, request and response.

                 *msg files are stored in msg directory, and srv files in srv directory*

The field types you can use are:
 * int8, int16, in32, int64
 * float32, float64,
 * string
 * time, duration
 * other msg files
 * variable-length array[], fixed-length array[C]
 * Header (contains a timestamp and coordinate frame information that are commonly used in ROS)
 
 > For making sure that our messages are turned into source, we have to change the file *package.xml* and make sure that lines *<build_depend>message_generation</build_depend>* and *<exec_depend>message_runtime</exec_depend>* are **uncomented**
 > There are several step for making the message. Go to the [msg/srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) wiki.
 
 ```
 rosmsg -h
 rossrv -h
 ```
