import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
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
cm = 0.01
rows = 5
cols = 8


criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, cm)


def find_T_target2cam(objp):
   global corners2
   ret,frame = cap.read()
        # imagePair = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
   imagePair = cv.flip(frame,-1)  #I like the USB cable pointing down... 
    # print("Image size:", imagePair.shape)
   height,width,chaneel = imagePair.shape


    #We assume side by side images and do no error checking!
   imageL = imagePair[0:height, 0:width//2]
   imageR = imagePair[0:height, width//2:width]

   gray = cv.cvtColor(imageL,cv.COLOR_BGR2GRAY) 


   ret = False
   while ret == False:
       print("Find Corners...")
       ret, corners = cv.findChessboardCorners(imageL, (cols,rows), None)
       # print(corners)
       if ret == True:
           corners2 = cv.cornerSubPix(gray, corners, (5,5), (-1,-1), criteria)
           print(corners2)
           for i in corners2:
            x, y = i.ravel()
            print(x,y)
            cv.circle(gray, (int(x), int(y)), 10, (0, 255, 0), -1)
           # Find the rotation and translation vectors.
           ret,rvecs,tvecs,_ = cv.solvePnPRansac(objp, corners2, mtx, dist) #These define the pose of the object in the camera RF
           rotation_matrix, _ = cv.Rodrigues(rvecs) #3x3 matrix
           print(tvecs,rvecs)
   cv.imshow("Disparity", gray)
   cv.waitKey(1)

  
   
if __name__ == '__main__':

   while True:
    #    for i in range(N):
        #    print("Move PSM2 to a new pose")
        #    enable = 0
        #    enable=input("Press ENTER key when ready\n")
        #    if enable == 1:
        #        if imageL is None:
        #            continue
           # Get current target pose in the camera's frame
        #    objp.append(PSM2_ECM_position)
        #    print(objp)
        #    rate.sleep()
       
    #    objp = np.array(objp, np.float32)
    #    assert objp.shape == (N, 3)
       objp=[np.array([0.08458405, 0.03057832, 0.21401877]), np.array([0.06299192, 0.02733521, 0.21324629]), np.array([0.04143956, 0.02263927, 0.21273572]), np.array([0.01891876, 0.02031183, 0.21210968]), np.array([-0.00348666,  0.01600946,  0.21121578]), np.array([-0.02658169,  0.01246915,  0.21004998]), np.array([0.08905207, 0.01049452, 0.20472179]), np.array([0.06730388, 0.00697676, 0.20419172]), np.array([0.0446152 , 0.0029974 , 0.20367808]), np.array([ 0.02221853, -0.00084381,  0.2022247 ]), np.array([ 0.00027382, -0.00367447,  0.20229065]), np.array([-0.02100709, -0.00921655,  0.20262666]), np.array([ 0.09310766, -0.00918013,  0.19622002]), np.array([ 0.07114203, -0.01323088,  0.19536586]), np.array([ 0.04918264, -0.01687228,  0.19465046]), np.array([ 0.02695855, -0.02101749,  0.1943438 ]), np.array([ 0.00435999, -0.02486691,  0.1931799 ]), np.array([-0.01778962, -0.02934282,  0.19294144]), np.array([ 0.09626806, -0.02929478,  0.18811592]), np.array([ 0.07511663, -0.03317397,  0.18719018]), np.array([ 0.05277934, -0.03690856,  0.18642416]), np.array([ 0.03043166, -0.04117343,  0.18540811]), np.array([ 0.00858679, -0.04454663,  0.18572186]), np.array([-0.01362738, -0.04886   ,  0.18483058]), np.array([ 0.09954802, -0.04965733,  0.18035944]), np.array([ 0.0788165 , -0.05325237,  0.1791374 ]), np.array([ 0.05678078, -0.05709374,  0.17897239]), np.array([ 0.03403533, -0.06066195,  0.17793954]), np.array([ 0.01168514, -0.06527448,  0.17705105]), np.array([-0.0102008 , -0.06918739,  0.17662098])]
       find_T_target2cam(objp)




        
       
   
   cv.destroyAllWindows()


#    [array([0.0875949 , 0.02296267, 0.19633698]), array([0.06633764, 0.02018287, 0.19605611]), array([0.04394418, 0.01949408, 0.19540388]), array([0.02151885, 0.01655096, 0.19570627]), array([-0.00077124,  0.01267502,  0.19538416]), array([-0.02324343,  0.00997388,  0.19611563]), array([0.09052711, 0.00353169, 0.18830997]), array([0.06819045, 0.00130119, 0.18754927]), array([ 0.04642437, -0.00178471,  0.18683893]), array([ 0.02453377, -0.00373111,  0.18730093]), array([ 0.00118206, -0.00699733,  0.18707227]), array([-0.02025671, -0.01032679,  0.18751021]), array([ 0.0928889 , -0.0166735 ,  0.17951339]), array([ 0.07120624, -0.01888295,  0.17852902]), array([ 0.04958642, -0.02154523,  0.17809304]), array([ 0.02798301, -0.02409539,  0.17995277])]