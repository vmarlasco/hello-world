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

# Publisher
We example shown is a node that would continually broadcast a message

```python
## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

##First of all we make sure that our code will be trated as python code
#!/usr/bin/env python 

import rospy    #Compulsory to import rospy for being trated as a ros node
from std_msgs.msg import String   #We would reuse the std_msgs/String message for our publications

##We would define the interface, the behaviour of our node.
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)   #Publishing to the chatter topic with message String
    rospy.init_node('talker', anonymous=True)   #Tells rospy the name of the node (talker) and anonymous for init
    rate = rospy.Rate(10) # 10hz    #Rate object creation. The mehod sleep allow us to control the rate (10Hz)
    while not rospy.is_shutdown():    #If the program is running
        hello_str = "hello world %s" % rospy.get_time() 
        rospy.loginfo(hello_str)  #Message printed in screen, written in Node's log and written in rosout (debugging)
        pub.publish(hello_str)    #Publish a String in the chatter topic
        rate.sleep()    #Sleep for a while

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:   #rospy.sleep() may rise and interruption with ctrl+C
        pass
```
