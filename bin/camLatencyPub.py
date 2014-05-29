#!/usr/bin/python
from PyQt4 import QtGui, QtCore, QtOpenGL
from PyQt4.QtOpenGL import QGLWidget
import OpenGL.GL as gl
import sys
import roslib;
roslib.load_manifest('crazyflieROS')
from std_msgs.msg import Header as HeaderMSG
import rospy
 
class GLPlotWidget(QGLWidget):
    # default window size
    width, height = 60, 60
 
    def initializeGL(self):
        self.white = 1
        gl.glClearColor(0,0,0,0)

    def toggle(self):
        self.white = 1-self.white
 
    def paintGL(self):
        """Paint the scene.
        """



        gl.glClearColor(self.white,self.white,self.white,0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        gl.glColor3f(1-self.white,1-self.white,1-self.white)

        gl.glBegin(gl.GL_QUADS)
        x1,x2 = 0.2, 0.8
        y1,y2 = 0.2, 0.8
        gl.glVertex2f(x1, y1)
        gl.glVertex2f(x2, y1)
        gl.glVertex2f(x2, y2)
        gl.glVertex2f(x1, y2)
        gl.glEnd()

        #gl.glClear(gl.GL_COLOR_BUFFER_BIT)

 
    def resizeGL(self, width, height):
        """Called upon window resizing: reinitialize the viewport.
        """
        # update the window size
        self.width, self.height = width, height
        # paint within the whole window
        gl.glViewport(0, 0, width, height)
        # set orthographic projection (2D only)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        # the window corner OpenGL coordinates are (-+1, -+1)
        gl.glOrtho(0, 1, 0, 1, -1, 1)



if __name__ == '__main__':
    class TestWindow(QtGui.QMainWindow):
        def __init__(self):
            super(TestWindow, self).__init__()

            rospy.init_node('camLatencyPublisher')
            self.pubTime = rospy.Publisher("/camLatency", HeaderMSG)

            self.widget = GLPlotWidget()

            geo = QtGui.QDesktopWidget().screenGeometry(1)
            self.setGeometry(30, 30, geo.width()-100, geo.height()-100)
            self.setCentralWidget(self.widget)
            self.widget.repaint()
            self.showFullScreen()
            #self.show()

            r = rospy.Rate(4)
            for x in range(25*5):
                msg = HeaderMSG()
                msg.seq = x+1
                self.widget.repaint()
                t = rospy.Time.now()
                msg.stamp = t
                self.pubTime.publish(msg)
                r.sleep()
                if x%5:
                    rospy.sleep(1./25)
                if x==0:
                    self.widget.repaint()
                    rospy.sleep(3.)
                self.widget.toggle()

            msg = HeaderMSG()
            msg.seq = 0 # reset
            self.widget.repaint()
            t = rospy.Time.now()
            msg.stamp = t
            self.pubTime.publish(msg)
            sys.exit()


    app = QtGui.QApplication(sys.argv)
    window = TestWindow()
    window.show()
    app.exec_()
