import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import CompressedImage
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError


# class Image:
#    def __init__(self):
#        self.img = None
   
#    def setImg(self,image):
#        self.img = image
#        self.gray = cv.cvtColor(image,cv.COLOR_BGR2GRAY) 
       


# def callback(ros_data):
#    global image


#    np_arr = np.fromstring(ros_data.data, np.uint8)
#    img = cv.imdecode(np_arr, cv.IMREAD_COLOR)
#    image.setImg(img)



# image = Image()
# subscriber = rospy.Subscriber("/endoscope/raw/left/image_raw/compressed", CompressedImage, callback, queue_size=1)

CAMERA_NUMBER = 0   #Try zero to start, then increase until it works...

# #create two named windows:
# cv2.namedWindow('Input',cv2.WINDOW_NORMAL)
# cv2.namedWindow('Output', cv2.WINDOW_NORMAL)


#open the camera
cap = cv.VideoCapture(CAMERA_NUMBER)

cap.set(cv.CAP_PROP_FOURCC,cv.VideoWriter.fourcc('M','J','P','G') )

#maximum resolution please!
cap.set(cv.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)


#camera parameters old
dist = np.float32([0.0102759524664842,	-0.0281907871027420, -7.57488700682001e-06,	-0.000432049924196620,	0.00449774409692344])


mtx = np.array([[595.030335934627,	0,	998.310793867376],
                [0,	594.224429631422,	524.342417190547],
               [0.000000, 0.000000, 1.000000]])




# chessboard
cm = 0.001
rows = 5
cols = 7


criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 40, cm)


def find_T_target2cam(objp):
   global corners2
   ret = False
#    objp=np.array([[ 0.06463591, -0.01264559,  0.23058901],[ 0.05250788, -0.01364509,  0.23018265],[ 0.04006838, -0.01516527,  0.22891427],[ 0.02844729, -0.01618801,  0.23018191],[ 0.01595919, -0.01801897,  0.22886736],[ 0.06641487, -0.02371446,  0.22561036],[ 0.05447879, -0.02490239,  0.22517249],[ 0.04192066, -0.02544112,  0.2252945 ],[ 0.03028297, -0.0265494 ,  0.22528261],[ 0.01846013, -0.02811912,  0.22523395],[ 0.06758255, -0.03329634,  0.22111439],[ 0.05614933, -0.03463924,  0.2210725 ],[ 0.04430692, -0.036444  ,  0.22053121],[ 0.03210793, -0.03768687,  0.22055502],[ 0.01968751, -0.03901227,  0.22012174],[ 0.06941415, -0.0442896 ,  0.21643337],[ 0.05671108, -0.04580395,  0.2158769 ],[ 0.04619669, -0.0471747 ,  0.21664309],[ 0.03361652, -0.04804001,  0.21620071],[ 0.02116428, -0.05027646,  0.2151679 ],[ 0.07075571, -0.05575863,  0.21164418],[ 0.05842813, -0.0568196 ,  0.21128119],[ 0.04682173, -0.05815223,  0.21155556],[ 0.03473878, -0.05956703,  0.21117657],[ 0.02347875, -0.06104376,  0.21143893]])
#    objp=np.array([[ 0.08680129, -0.00527777,  0.22868165],
#  [ 0.0639283,  -0.00763846,  0.2286768 ],
#  [ 0.04017326, -0.0093069,   0.2271926 ],
#  [ 0.01760379, -0.01186398 , 0.22661625],
#  [-0.00588981, -0.01456948 , 0.22523522],
#  [-0.02876158, -0.01715521,  0.22586644],
#  [ 0.08888344, -0.02670895,  0.2189254 ],
#  [ 0.06565239 ,-0.02813443,  0.21858321],
#  [ 0.0432962 , -0.03014519,  0.2185284 ],
#  [ 0.02035026 ,-0.03309129,  0.21733205],
#  [-0.00344884, -0.03591139,  0.21614808],
#  [-0.0262778 , -0.0380994 ,  0.21572258],
#  [ 0.09120452 ,-0.04764841,  0.21026774],
#  [ 0.06827804 ,-0.04952578, 0.20929411],
#  [ 0.04571741, -0.05154165,  0.20906448],
#  [ 0.02260912, -0.05396559,  0.2079601 ],
#  [-0.00036639, -0.0565296 , 0.20742516],
#  [-0.02333873, -0.05899468 , 0.20672415],
#  [ 0.09339183 ,-0.06839818,  0.20102695],
#  [ 0.07024533, -0.07098345,  0.20028627],
#  [ 0.04824245, -0.07213303,  0.20015045],
#  [ 0.02492294 ,-0.07436718 , 0.19952215],
#  [ 0.00285313 ,-0.07703864,  0.19896294],
#  [-0.02052784 ,-0.08018141 , 0.19806205],
#  [ 0.09606054, -0.08942018,  0.19112462],
#  [ 0.0731999 , -0.09186399,  0.19107673],
#  [ 0.05031209 ,-0.09374124 , 0.19042338],
#  [ 0.02692913 ,-0.09573482,  0.18960053],
#  [ 0.00526501, -0.09832215,  0.19016571],
#  [-0.01866862, -0.10054615,  0.18876182]])
   while ret == False:
       print("Find Corners...")
       ret, corners = cv.findChessboardCorners(imageL, (cols,rows), None)
       # print(corners)
       if ret == True:
           corners2 = cv.cornerSubPix(gray, corners, (5,5), (-1,-1), criteria)
           print(corners2)
        #    for i in corners2:
        #     x, y = i.ravel()
        #     cv.circle(gray, (x, y), 3, (0, 255, 0), -1)
           # Find the rotation and translation vectors.
           ret,rvecs,tvecs,_ = cv.solvePnPRansac(objp, corners2, mtx, dist) #These define the pose of the object in the camera RF
           # rotation_matrix, _ = cv.Rodrigues(rvecs) #3x3 matrix
           print(tvecs)


   return rvecs, tvecs


def callbackPSM2_ECM(ros_data):
   global imageL, gray
   ret,frame = cap.read()
        # imagePair = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
   imagePair = cv.flip(frame,-1)  #I like the USB cable pointing down... 
    # print("Image size:", imagePair.shape)
   height,width,chaneel = imagePair.shape


    #We assume side by side images and do no error checking!
   imageL = imagePair[0:height, 0:width//2]
   imageR = imagePair[0:height, width//2:width]

   gray = cv.cvtColor(imageL,cv.COLOR_BGR2GRAY) 

   

   cv.imshow("Disparity", imageL)
   cv.waitKey(1)


   global PSM2_ECM_position, PSM2_ECM_rotation
   pos = ros_data.pose.position
   rot = ros_data.pose.orientation
   PSM2_ECM_position = np.array([pos.x, pos.y, pos.z])
   PSM2_ECM_rotation = [rot.x,rot.y,rot.z,rot.w]


subscriber_PSM2_ECM = rospy.Subscriber("/dvrk/PSM2/position_cartesian_current", PoseStamped, callbackPSM2_ECM, queue_size=1)


   
if __name__ == '__main__':


   rospy.init_node("demo")
   rate = rospy.Rate(25) # 10hz
   N = int(input("How many acquisitions?\n")) # number of acquisitions


   objp= []
   out = True


   while not rospy.is_shutdown() and out:
       for i in range(N):
           print("Move PSM2 to a new pose")
           enable = 0
           enable=input("Press ENTER key when ready\n")
           if enable == 1:
               if imageL is None:
                   continue
           # Get current target pose in the camera's frame
           print("+1前",PSM2_ECM_position)
           rotation_matrix = quaternion_matrix(PSM2_ECM_rotation)[:3, :3]
         #   PSM2_ECM_position[2] += 0.01
           PSM2_ECM_position = np.dot(rotation_matrix, np.array([0, 0, 0.008])) + PSM2_ECM_position
           print("+1后",PSM2_ECM_position)
           objp.append(PSM2_ECM_position)
           print(objp)
           rate.sleep()
       
       objp = np.array(objp, np.float32)
       assert objp.shape == (N, 3)
       print(objp)
       rvecs, tvecs = find_T_target2cam(objp)




       print('rvecs =', rvecs)
       print('tvecs =', tvecs)
       
       out = False
   cv.destroyAllWindows()


#    [array([0.0875949 , 0.02296267, 0.19633698]), array([0.06633764, 0.02018287, 0.19605611]), array([0.04394418, 0.01949408, 0.19540388]), array([0.02151885, 0.01655096, 0.19570627]), array([-0.00077124,  0.01267502,  0.19538416]), array([-0.02324343,  0.00997388,  0.19611563]), array([0.09052711, 0.00353169, 0.18830997]), array([0.06819045, 0.00130119, 0.18754927]), array([ 0.04642437, -0.00178471,  0.18683893]), array([ 0.02453377, -0.00373111,  0.18730093]), array([ 0.00118206, -0.00699733,  0.18707227]), array([-0.02025671, -0.01032679,  0.18751021]), array([ 0.0928889 , -0.0166735 ,  0.17951339]), array([ 0.07120624, -0.01888295,  0.17852902]), array([ 0.04958642, -0.02154523,  0.17809304]), array([ 0.02798301, -0.02409539,  0.17995277])]