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
    width, height = 600, 600
 
    def initializeGL(self):
        self.white = 1
        gl.glClearColor(0,0,0,0)
 
    def paintGL(self):
        """Paint the scene.
        """
        # clear the buffer
        if self.white:
            gl.glClearColor(1,1,1,0)
        else:
            gl.glClearColor(0,0,0,0)
        self.white = 1 - self.white

        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
 
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
        gl.glOrtho(-1, 1, -1, 1, -1, 1)



if __name__ == '__main__':
    class TestWindow(QtGui.QMainWindow):
        def __init__(self):
            super(TestWindow, self).__init__()

            rospy.init_node('camLatencyPublisher')
            self.pubTime = rospy.Publisher("/camLatency", HeaderMSG)

            self.widget = GLPlotWidget()

            geo = QtGui.QDesktopWidget().screenGeometry()
            self.setGeometry(30, 30, geo.width()-100, geo.height()-100)
            self.setCentralWidget(self.widget)

            self.show()
            #self.showFullScreen()

            d = rospy.Duration(0.08) # 80ms
            for x in range(30):
                rospy.sleep(d)
                msg = HeaderMSG()
                msg.seq = x
                self.widget.repaint()
                t = rospy.Time.now()
                msg.stamp = t
                self.pubTime.publish(msg)

            sys.exit()


    app = QtGui.QApplication(sys.argv)
    window = TestWindow()
    window.show()
    app.exec_()