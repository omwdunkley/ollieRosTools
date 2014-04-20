#!/usr/bin/env python
import roslib; 
roslib.load_manifest("ollieRosTools")
import rospy
import tf
import copy
import tf.msg as TFMSG
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import ColorRGBA as ColorMSG
from sensor_msgs.msg import CameraInfo as CameraInfoMSG
from sensor_msgs.msg import Image as ImageMSG
from geometry_msgs.msg import Point as PointMSG
from geometry_msgs.msg import Transform as TransformMSG
from geometry_msgs.msg import Quaternion as QuatMSG
from geometry_msgs.msg import Vector3 as TranslationMSG
from visualization_msgs.msg import MarkerArray as MarkerArrayMSG
from visualization_msgs.msg import Marker as MarkerMSG
import roslib.packages
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from ollieRosTools.cfg import synth_paramsConfig as synthCFG
import glob


import csv
import time, sys, datetime
from optparse import OptionParser
from math import pi as PI
from math import sqrt, sin, cos, degrees, radians, atan2, atan, copysign
import numpy as np
import cv2
from cv2 import cv as cv

from ollieRosTools.msg import synthFrame as synthMSG



def xyzWXYZ_xyzXYZW(data):
    return np.concatenate((data[:,0:3],-data[:,4:5],-data[:,5:6],-data[:,6:7],data[:,3:4]),1)

def toQuat(tq):
    return tq[3:7]

def toXYZ(tq):
    return tq[0:3]

def toRotMsg(tq):
    q = QuatMSG()
    q.x, q.y, q.z, q.w = toQuat(tq)
    return q

def toTranslationMsg(tq):
    t = TranslationMSG()
    t.x, t.y, t.z = toXYZ(tq)
    return t

def toTransformMsg(tq):
    t = TransformMSG()
    t.rotation = toRotMsg(tq)
    t.translation = toTranslationMsg(tq)
    return t


class synthData:
    def __init__(self, options):
        self.options = options
        self.preconfig = None
        
        # Advertise publishers 
        self.pub_synth    = rospy.Publisher("/synthetic/frame", synthMSG)
        self.pub_image    = rospy.Publisher("/synthetic/image_raw", ImageMSG)
        self.pub_caminfo  = rospy.Publisher("/synthetic/camera_info", CameraInfoMSG)
        self.pub_marker  = rospy.Publisher("/synthetic/world_points", MarkerMSG)
        self.pub_tf    = tf.TransformBroadcaster()     
        self.bridge = CvBridge()
        
        # Subscribers           
        self.sub_tf    = tf.TransformListener()   
        
        self.TCAM = []
        self.TOBJ = []
        self.OBJ2CAM = []
        self.CAM2IMG = []
        self.IMAGES = []
        self.KPS = []
        self.CLOUD = []
        self.forwards = 1;
        
        self.cloudMarker = MarkerArrayMSG()
        self.load_data(options.path)


        self.step = 0
        self.pause = False
        self.playRate = float(self.options.rate)
        self.rosRate = rospy.Rate(self.playRate)
        self.stepSize = int(self.options.stepSize)
        self.nr = len(self.IMAGES)
        self.pxNoise = 0.0;
        self.matchOutlier = 0.0;
        rospy.loginfo("Loaded %d images", self.nr)
        
        # Dynserver                
        self.dynserver = DynamicReconfigureServer(synthCFG, self.reconfigure)
        
        while not rospy.is_shutdown():
            self.sendSynth()
            self.rosRate.sleep()
        
    
    def reconfigure(self, config, level):

        self.pxNoise = config.pxNoise
        self.matchOutlier = config.matchOutlier
        
        if self.playRate!=config.playRate:
            self.playRate=config.playRate
            self.rosRate = rospy.Rate(self.playRate)    
    
        
        # Preserve direction, whether its going forwards or backwards
        if abs(self.stepSize) != abs(config.stepSize):
            self.stepSize = config.stepSize*self.forwards

        self.stepSize = config.stepSize
        
        self.pause = config.pause
        if config.step:
            config.step = False
            self.sendSynth(True)  
                           
        self.preconfig = config

        return config
    


    def sendSynth(self, force=False):
        if (not self.options.active and self.pub_synth.get_num_connections()<1) or self.pause and not force:
            return
        
        self.doStep()
        
        # Show output
        if self.options.vis:
            cv2.imshow("img",self.IMAGES[self.step])
            cv2.waitKey(10)    
        
        # Build frame msg
        msg = synthMSG()
        msg.frameId = self.step+1 #to be consistent with matlab
        

        
        # kps
        msg.pts2d = self.KPS[self.step][:]

        msg.pts2d_noise = copy.deepcopy(self.KPS[self.step]) #Copy


        # Add noise

        image = np.copy(self.IMAGES[self.step])
        for i in range(len(msg.pts2d_noise)):
            msg.pts2d_noise[i].x += np.random.randn()*self.pxNoise
            msg.pts2d_noise[i].y += np.random.randn()*self.pxNoise
            #cv2.circle(image, (int(msg.pts2d[i].x), int(msg.pts2d[i].y)), 2, (0,255,0), 1, cv.CV_AA)
            cv2.circle(image, (int(msg.pts2d_noise[i].x), int(msg.pts2d_noise[i].y)), 1, (0,0,0), 1)#, cv.CV_AA)
            #cv2.line(image, (int(msg.pts2d_noise[i].x), int(msg.pts2d_noise[i].y)), (int(msg.pts2d[i].x), int(msg.pts2d[i].y)), (255,0,0) ,3,cv2.CV_AA)




        # Fill out img
        msg.img = self.bridge.cv2_to_imgmsg(image, "bgr8")
        msg.img.header.stamp = rospy.Time.now()
        msg.img.header.frame_id = "/synCamGT"




        # Fill out cam info
        msg.camInfo = self.camInfo
        msg.camInfo.header.stamp = msg.img.header.stamp  
        
        # imu2cam Transform
        msg.imu2cam = self.OBJ2CAMmsg     

        # IMU rotation only
        msg.imu.rotation = toRotMsg(self.TOBJ[self.step])
        
        # Full GT pose of cam and imu in world frame
        msg.imuPose = toTransformMsg(self.TOBJ[self.step])
        msg.camPose =  toTransformMsg(self.TCAM[self.step])
                
        #Send all
        self.pub_tf.sendTransform(toXYZ(self.TCAM[self.step]), toQuat(self.TCAM[self.step]), msg.camInfo.header.stamp, "/synCamGT", "/world")
        self.pub_tf.sendTransform(toXYZ(self.TOBJ[self.step]), toQuat(self.TOBJ[self.step]), msg.camInfo.header.stamp, "/synObjGT", "/world")
        self.pub_tf.sendTransform(toXYZ(self.OBJ2CAM), toQuat(self.OBJ2CAM), msg.camInfo.header.stamp, "/synCamGTviaObj", "/synObjGT")
        self.pub_tf.sendTransform(toXYZ(self.OBJ2CAM), toQuat(self.OBJ2CAM), msg.camInfo.header.stamp, "/cam", "/cf_attitude")
        self.pub_tf.sendTransform([0,0,0], toQuat(self.TOBJ[self.step]), msg.camInfo.header.stamp, "/cf_attitude", "/world") #TODO ADD NOISE
        
        # Publish all messages
        self.pub_synth.publish(msg)
        self.pub_image.publish(msg.img)
        self.pub_caminfo.publish(msg.camInfo)
        rospy.loginfo("Sent Frame %d", self.step)

        # Publish Map points
        self.CLOUD.header.stamp = msg.img.header.stamp 
        if self.step%20==0:
            rospy.loginfo("Updated Point Cloud")
            self.pub_marker.publish(self.CLOUD)       

        
                
        
    def doStep(self):        
        # Looping / exit condition
        self.step += self.stepSize*self.forwards
        if self.step>=self.nr or self.step<0:
            if self.options.loop:
                self.forwards *= -1;
                rospy.loginfo("Looping data")
                self.pub_marker.publish(self.CLOUD) 
                self.step += self.stepSize * self.forwards
            else:
                rospy.loginfo("Finished sending all data")
                rospy.signal_shutdown("Finished sending all data")
                self.step -= self.stepSize*self.forwards

                


    def load_data(self, path=""):
        if path == "":
            path = roslib.packages.get_pkg_dir('ollieRosTools')+"/matlab/Data/exported/"
     
        # Load Trajectory
        with open(path+'trajectory/TCAM.csv', 'rb') as f:
            reader = csv.reader(f)
            for row in reader:
                self.TCAM.append(row)    
        self.TCAM = np.array(self.TCAM)
        self.TCAM =xyzWXYZ_xyzXYZW(self.TCAM.astype(np.double))
       
        # Load IMU Trajectory
        with open(path+'/trajectory/TOBJ.csv', 'rb') as f:
            reader = csv.reader(f)
            for row in reader:
                self.TOBJ.append(row)   
        self.TOBJ = np.array(self.TOBJ)
        self.TOBJ = xyzWXYZ_xyzXYZW(self.TOBJ.astype(np.double))
                
        # Load cloud        
        with open(path+'points3d/cloud.csv', 'rb') as f:
            reader = csv.reader(f)
            for row in reader:
                self.CLOUD.append(row)  
        self.CLOUD = np.array(self.CLOUD)
        self.CLOUD = self.CLOUD.astype(np.double)   

        mincol = np.min(self.CLOUD,0)
        maxcol = np.max(self.CLOUD,0)
        col = self.CLOUD-mincol
        col = col/np.abs(mincol - maxcol )


        m = MarkerMSG()
        m.action = m.ADD
        m.frame_locked = True
        m.ns = "WorldPointsGT"
        m.header.frame_id = "/world"
        m.type = m.POINTS
        m.pose.orientation.w = 1.0
        m.pose.orientation.z = 0
        m.pose.orientation.y = 0
        m.pose.orientation.x = 0
        m.id = 0
        m.pose.position.x = 0
        m.pose.position.y = 0
        m.pose.position.z = 0
        #m.color.a = 0.6
        #m.color.r = 1.0
        #m.color.b = 0.0
        #m.color.g = 0.0       
        m.scale.x = 0.05
        m.scale.y = 0.05
        m.scale.z = 0.05
        for wp,co in zip(self.CLOUD, col):   
            p = PointMSG()            
            p.x = wp[1]
            p.y = wp[2]
            p.z = wp[3]   
            c = ColorMSG()
            c.a = 0.75
            c.r = co[1]
            c.g = co[2]
            c.b = co[3]
            m.points.append(p)
            m.colors.append(c)
        self.CLOUD = m
                 
                 
        # Load Camera
        with open(path+'meta/cam2img.csv', 'rb') as f:
            reader = csv.reader(f)
            for row in reader:
                self.CAM2IMG.append(row)    
        self.CAM2IMG = np.array(self.CAM2IMG)
        self.CAM2IMG = self.CAM2IMG.astype(np.double)    
        self.CAM2IMG = np.array(self.CAM2IMG)
        self.camInfo = CameraInfoMSG()
        self.camInfo.width = self.CAM2IMG[0,2]*2
        self.camInfo.height = self.CAM2IMG[1,2]*2
        self.camInfo.distortion_model = "plumb_bob"
        self.camInfo.P = self.CAM2IMG.flatten()
        self.camInfo.K = self.CAM2IMG[0:3,0:3].flatten()
        self.camInfo.R = np.eye(3).flatten()
        self.camInfo.D = [0.,0.,0.,0.,0.]
        self.camInfo.header.frame_id = "/synCamGT"                              
        
        # Load IMU Cam transform
        with open(path+'meta/imu2cam.csv', 'rb') as f:
            reader = csv.reader(f)
            for row in reader:
                self.OBJ2CAM.append(row) 
        self.OBJ2CAM = np.array(self.OBJ2CAM)
        self.OBJ2CAM = xyzWXYZ_xyzXYZW(self.OBJ2CAM.astype(np.double))[0]
        self.OBJ2CAMmsg = toTransformMsg(self.OBJ2CAM)

        
        #Load KPS
        files = sorted(glob.glob(path+'features/f2d*.csv'))
        for file in files:
            f2dmsg = []
            with open(file, 'rb') as fi:
                reader = csv.reader(fi)
                f2d = []
                for row in reader:
                    f2d.append(row)              
                f2d = np.array(f2d)
                f2d = f2d.astype(np.double)  
                for f in f2d:
                    p= PointMSG()
                    p.x = f[1]
                    p.y = f[2]
                    p.z = f[0] #id
                    f2dmsg.append(p)
            self.KPS.append(f2dmsg)             
        

            
        #Load images
        files = sorted(glob.glob(path+'images/img*.png'))
        for file in files:
            self.IMAGES.append(cv2.imread(file))
            
        
        

def run(args=None):    
    parser = OptionParser(description='Synthethic Data Broadcaster')
    parser.add_option('--rate', '-r',
                        dest='rate',
                        default='25.',
                        help='Rate to send synthetic data at, eg 25 for 25hz')  
    parser.add_option('--step', '-s',
                        dest='stepSize',
                        default='1',
                        help='Frame step. eg -s4 means show every 4th frame')      
    parser.add_option('--loop', '-l',
                        dest='loop',
                        action="store_true",
                        default=False,
                        help='Keep looping the file')      
    parser.add_option('--visualise', '-v',
                        dest='vis',
                        action="store_true",
                        default=False,
                        help='Show frames')  
    parser.add_option('--active', '-a',
                        dest='active',
                        action="store_true",
                        default=False,
                        help='Publish even if not being listened to')
    parser.add_option('--path', '-p',
                        dest='path',
                        default='',
                        help='Path of data to load. Defaults to package/matlab/Data/exported')
            
    (options, leftargs) = parser.parse_args(args=args)
        
    #Load some DB depending on options 
    rospy.init_node('SynthDataSource')   
    synth = synthData(options)

    #START NODE
#     try:        
#         rospy.spin()
#     except KeyboardInterrupt:    
#         rospy.loginfo('...exiting due to keyboard interrupt')
    #synth.shutdown()


if __name__ == "__main__":   
    run(sys.argv)
