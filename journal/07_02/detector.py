#!/usr/bin/env python
# Author: Victor Martinez

# Future-type libraries
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

# Tool libraries
import sys
import cv2
import numpy as np
import tensorflow as tf
import time
import datetime

# Detector libraries
from util import bbox_transform
from config import base_model_config
from roe_squeezeDet_config import roe_squeezeDet_config
from squeezeDet import SqueezeDet

# Raspberry Camera libraries
from picamera.array import PiRGBArray
from picamera import PiCamera


# ROS libraries
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

############################################ LABELS ###########################
FLAGS = tf.app.flags.FLAGS

base_dir = "/catkin_ws/src/roe_detector/scripts/"
#base_dir = "/media/victor/DATA/MEGA/THESIS/SOFTWARE/Docker/Files/NN"

tf.app.flags.DEFINE_string(
    'checkpoint', base_dir + '/data/model_checkpoints/model.ckpt-9900',
    """Path to the model parameter file.""")

tf.app.flags.DEFINE_boolean(
    'debug_mode', True, "Used for controlling console messages")


class RoeDetector():
    """docstring for ClassName"""
    def __init__(self):

        ##### Initializing the neede objects for image capture
        self._bridge = CvBridge()
        self._camera = PiCamera()
        self._camera.resolution = (640, 480)

        #time.sleep(0.1)     #allow the camera to warmup. Do not needed due time until first capture

        ##### ROS node intialization
        self._image_pub = rospy.Publisher('/ROE/DetectedImage', Image, queue_size=1)
        # TODO(Victor) : Declare another publisher for managing information
        # TODO(Victor) : Declare a subscriber for node control. In this way it could be possible to add pause etc.

        ##### Tensorflow initialization
        self._mc = roe_squeezeDet_config()      #configure squeezenet
        self._mc.BATCH_SIZE = 1
        self._mc.LOAD_PRETRAINED_MODEL = False  #model parameters will be restored from checkpoint
        self._model = SqueezeDet(self._mc)      #loading the net

        self._saver = tf.train.Saver(self._model.model_params)  #Defining the saver

        self._sess = tf.InteractiveSession(config=tf.ConfigProto(allow_soft_placement=True))    #Start interactive session

        self._saver.restore(self._sess, FLAGS.checkpoint)       #Restoring the paremeters from checkpoint

        ##### Utils
        self._cls2clr = {
                    'extinguisher': (255, 191, 0),
                    'exitsign': (0, 191, 255),
                    'lightswitch': (70, 191, 255),
                    'door': (255, 0, 191)
                }


    def _draw_box(self, im, box_list, label_list, color=(0,255,0), cdict=None, form='center'):
        assert form == 'center' or form == 'diagonal', \
            'bounding box format not accepted: {}.'.format(form)

        for bbox, label in zip(box_list, label_list):

            if form == 'center':
                bbox = bbox_transform(bbox)

            xmin, ymin, xmax, ymax = [int(b) for b in bbox]

            l = label.split(':')[0] # text before "CLASS: (PROB)"
            if cdict and l in cdict:
                c = cdict[l]
            else:
                c = color

            # draw box
            cv2.rectangle(im, (xmin, ymin), (xmax, ymax), c, 1)
            # draw label
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(im, label, (xmin, ymax), font, 0.3, c, 1)

    
    def startOperation(self):

        #TODO(Victor) : Manage exceptions with key board interrupt, and other errors and finally to free TF session
        rawCapture = PiRGBArray(self._camera)

        try:    
            while not rospy.is_shutdown():
                if FLAGS.debug_mode:
                    a = datetime.datetime.now().replace(microsecond=0)
                #Take a capture
                self._camera.capture(rawCapture, format="bgr")
                im = rawCapture.array
                rawCapture.truncate(0)  #We have to free the array between captures
                #im = im.astype(np.float32, copy=False)
                #im = cv2.resize(im, (mc.IMAGE_WIDTH, mc.IMAGE_HEIGHT))
                input_image = im - self._mc.BGR_MEANS

                #Detection
                det_boxes, det_probs, det_class = self._sess.run(
                    [self._model.det_boxes, self._model.det_probs, self._model.det_class],
                    feed_dict={self._model.image_input: [input_image]})

                #Filtering
                final_boxes, final_probs, final_class = self._model.filter_prediction(
                    det_boxes[0], det_probs[0], det_class[0])

                keep_idx = [idx for idx in range(len(final_probs)) \
                                if final_probs[idx] > self._mc.PLOT_PROB_THRESH]
                final_boxes = [final_boxes[idx] for idx in keep_idx]
                final_probs = [final_probs[idx] for idx in keep_idx]
                final_class = [final_class[idx] for idx in keep_idx]

                # Draw boxes
                self._draw_box(
                    im, final_boxes,
                    [self._mc.CLASS_NAMES[idx] + ': (%.2f)' % prob for idx, prob in zip(final_class, final_probs)],
                    cdict=self._cls2clr,
                )

                # output
                try:
                    self._image_pub.publish(self._bridge.cv2_to_imgmsg(im, "bgr8"))
                except CvBridgeError as e:
                    print(e)
                    raise

                if FLAGS.debug_mode:
                    b = datetime.datetime.now().replace(microsecond=0)
                    print("\n| Image processing time -----> {}  ||".format(b-a))
        except:
            self._sess.close()
            raise

    
    def _callback(self):
        raise NotImplementedError



def main(args=None):
    detector = RoeDetector()
    rospy.init_node('image_detector', anonymous=True)
    print("\n\n ######## STARTING ######")
    try:
        detector.startOperation()
    except KeyboardInterrupt:
        print("\n>>Shutting down :(")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    tf.app.run()
