# Create catkin workspace
```
$ source /opt/ros/kinetic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
```

# Packages in catkin workspace
## Workspace configuration
```
workspace_folder/         -- WORKSPACE
  src/                    -- SOURCE SPACE
    CMakeLists.txt        -- The 'toplevel' CMake file
    package_1/
      CMakeLists.txt
      package.xml
      ...
    package_n/
      CATKIN_IGNORE       -- Optional empty file to exclude package_n from being processed
      CMakeLists.txt
      package.xml
      ...
  build/                  -- BUILD SPACE
    CATKIN_IGNORE         -- Keeps catkin from walking this directory
  devel/                  -- DEVELOPMENT SPACE (set by CATKIN_DEVEL_PREFIX)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin
    env.bash
    setup.bash
    setup.sh
    ...
  install/                -- INSTALL SPACE (set by CMAKE_INSTALL_PREFIX)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin             
    env.bash
    setup.bash
    setup.sh
    ...
```

## Create a catkin package
```
$ cd ~/catkin_ws/src
$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
$ cd ~/catkin_ws
$ catkin_make
$ . ~/catkin_ws/devel/setup.bash
```

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
As example, we show a node that would continually broadcast a message.

```python
#!/usr/bin/env python
##First of all we make sure that our code will be trated as python code

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

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

We can simply run a the node with the command:
```
        rosrun beginner_tutorials talker.py       #rosrun (package) (node)
```

# Subscriber
Node that will receive the messages. Both nodes should be subscrited to the topic chatter in order to perform the communication.

```python
#!/usr/bin/env python
##First of all we make sure that our code will be trated as python code

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

## Callback-based mechanism for subscribing massages
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)   #Node subscribe to topic chatter, and execute callback when reception

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

We can simply run a the node with the command:
```
        rosrun beginner_tutorials listener.py       #rosrun (package) (node)
```

# Service node
Services are like-functions nodes, in the sense that they are ready for "serve" or carry with some task when it is requested. The file **scripts/add_two_ints_server.py** is written.

> In previous tutorials it was created the file *AddTwoInts.srv* with two inputs (*int64 a*, *int64 b*) an one output (*int64 sum)

```python
 #!/usr/bin/env python
 
 from beginner_tutorials.srv import *   #In first place, we have to import the created nodes
 import rospy   #import rospy for defining as ROS node
 
 ## We define in a fuction the real behaviour (handling) of our server: summation (and print the result by screen)
 def handle_add_two_ints(req):
     print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
     return AddTwoIntsResponse(req.a + req.b)
 
 ## We define the node and we make it known for the master
 def add_two_ints_server():
     rospy.init_node('add_two_ints_server')
     s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints) #We degine the name, srv, and handling function
     print "Ready to add two ints."
     rospy.spin()   #Keep node running until service is shutdown
 
 if __name__ == "__main__":
     add_two_ints_server()
```

**Do not forget to make the nodes executable**
```
chmod +x scripts/add_two_ints_server.py
```

# Client node
The client node will request a task from the service node. The file **scripts/add_two_ints_server.py** is written. As we would see this is a client node, so we do not have to initiate a node with *init_node()* because it is written only for request an isolated service.

```python
#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')    #Method that blocks until server is available
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)   #Once we have the resource, we create a handle for calling
        resp1 = add_two_ints(x, y)    #We use the handle as a common function
        return resp1.sum    #We have to realize that .srv are objects (AddTwoIntsRequest & AddTwoIntsResponse)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
```



