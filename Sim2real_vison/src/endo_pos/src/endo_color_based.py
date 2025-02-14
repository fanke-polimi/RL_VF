import cv2
from ultralytics import YOLO
import rospy
import open3d as o3d
import cv2
import numpy as np
# import pcl
from sensor_msgs.msg import Image, PointCloud2,CompressedImage
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from cv_bridge import CvBridge, CvBridgeError
from cam_params import stereoCamera
from depth_utils import preprocess, undistortion, getRectifyTransform, rectifyImage,\
     stereoMatchSGBM
import os
import torch

class endo_stereo:
    def __init__(self):
        rospy.init_node('stereo_subscriber', anonymous=True)
        # self.model = YOLO(model="/media/fk/Elements11/yolov8/ultralytics-main/runs/detect/train3/weights/last.pt") 
        self.bridge = CvBridge()
        self.left_image_sub = rospy.Subscriber('/endoscope/raw/left/image_raw', Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber('/endoscope/raw/right/image_raw', Image, self.right_image_callback)
        # self.right_image_sub = rospy.Subscriber("/endoscope/raw/right/image_raw/compressed", CompressedImage, self.right_image_callback)
        # self.left_image_sub = rospy.Subscriber("/endoscope/raw/left/image_raw/compressed", CompressedImage, self.left_image_callback)
        self.pointcloud_pub = rospy.Publisher('/camera/point_cloud', PointCloud2, queue_size=10)
        self.left_image = None
        self.right_image = None
        self.count = 0
        self.img_save1 =0

        

     
        self.config=stereoCamera()

        self.camera_matrix_left = self.config.cam_matrix_left
        self.camera_matrix_right = self.config.cam_matrix_right
        self.dist_coeffs_left = self.config.distortion_l
        self.dist_coeffs_right = self.config.distortion_r
        self.R = self.config.R
        self.T = self.config.T


    def left_image_callback(self, msg):
        # try:
        self.left_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # except CvBridgeError as e:
        #     rospy.logerr(f"Failed to convert left image: {e}")
        #     return
        self.process_images()

    def right_image_callback(self, msg):
        # try:
        self.right_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # except CvBridgeError as e:
        #     rospy.logerr(f"Failed to convert right image: {e}")
        #     return
        # self.process_images()

    def process_images(self):
        # if self.left_image is not None and self.right_image is not None:
        #     if self.left_image.size > 0 and self.right_image.size > 0:

                folder_path = "/media/fk/Elements11/yolo_dataset/cali"
                timeF = 20
                # global count,bridge,img_save1
                # self.count = count + 1
                self.img_save1 = self.img_save1+1
                # if count == 1:
                #     count = 0
                # cv_img = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
                
                # cv2.imshow("framel" , cv_img)
                # cv2.imshow("framelr" , cv_img)
                cv2.waitKey(1)
                if(self.img_save1 % timeF == 0):
                    print("zuo边",self.img_save1)
                    cv2.imwrite(folder_path +"/l-6/"+str(int(self.img_save1/timeF))+"l.jpg", self.left_image)
                    cv2.imwrite(folder_path +"/r-6/"+str(int(self.img_save1/timeF))+"r.jpg", self.right_image)






                map1x, map1y, map2x, map2y, Q = getRectifyTransform(1080, 1920, self.config)
                self.Q = Q
                # iml_rectified, imr_rectified = rectifyImage(self.left_image, self.right_image, map1x, map1y, map2x, map2y)
                iml = undistortion(self.left_image, self.config.cam_matrix_left, self.config.distortion_l)
                imr = undistortion(self.right_image, self.config.cam_matrix_right, self.config.distortion_r)

                

                iml_rectified_l, imr_rectified_r = rectifyImage(iml, imr, map1x, map1y, map2x, map2y)
                # red_left, radius_left = self.detect_red_point(iml_rectified_l)
                # red_right, radius_right = self.detect_red_point(imr_rectified_r)

                # if red_left is not None and red_right is not None:
                #     # Draw circles on the red points
                #     cv2.circle(iml_rectified_l, red_left, radius_left*100, (0, 0, 255), 2)  # Circle with border
                #     # cv2.circle(imr_rectified_r, red_right, radius_right, (0, 0, 255), 2)  # Circle with border

                #     # Compute disparity and depth
                #     disparity = red_left[0] - red_right[0]
                #     print("视差",red_left[0])
                #     depth = self.calculate_depth(disparity)

                #     # Put text on the image
                #     depth_text = f"Depth: {depth:.2f}m"
                #     cv2.putText(iml_rectified_l, depth_text, (red_left[0] - 50, red_left[1] - 30),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                #     cv2.putText(imr_rectified_r, depth_text, (red_right[0] - 50, red_right[1] - 30),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                #     rospy.loginfo(f"Red Point Depth: {depth} meters")
                # cv2.imshow('Disparity', iml_rectified_l)


                iml_, imr_ = preprocess(iml_rectified_l, imr_rectified_r)
                displ, dispr, left_matcher= stereoMatchSGBM(iml_, imr_, True) 

                wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
                wls_filter.setLambda(8000)
                wls_filter.setSigmaColor(1.1)
                # wls_filter.setLRCthresh(24)
                # wls_filter.setDepthDiscontinuityRadius(3)

                # filtered_disparity = wls_filter.filter(displ, iml_, None, dispr)
                filtered_disparity = displ
                # filteredImg = cv2.normalize(src=filtered_disparity, dst=filtered_disparity, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
                # filtered_disparity = np.uint8(filteredImg)
                # filtered_disparity = cv2.normalize(filtered_disparity, None, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
                # filtered_disparity = np.uint8(filtered_disparity)

                # # Compute depth map
                # points_3D = cv2.reprojectImageTo3D(displ, Q)
                # colors = cv2.cvtColor(iml_rectified_l, cv2.COLOR_BGR2RGB)

                # mask = filtered_disparity > filtered_disparity.min()
                # output_points = points_3D[mask]
                # output_colors = colors[mask]

                # self.publish_point_cloud(output_points, output_colors)

                # self.visualize_point_cloud(output_points, output_colors)
                # self.save_point_cloud(output_points, output_colors, "point_cloud.ply")

                # Normalize the disparity map for visualization
                disparity_vis = (filtered_disparity - filtered_disparity.min()) / (filtered_disparity.max() - filtered_disparity.min())
                disparity_vis = (disparity_vis * 255).astype(np.uint8)

                self.filtered_disparity = filtered_disparity
                self.Q = Q





               

                cv2.imshow("Disparity", self.left_image)
                cv2.imshow("YOLOv8 Inference_r",self.right_image)
                cv2.setMouseCallback('Disparity', self.on_mouse_click)
                # cv2.imshow("original", iml_rectified_l)
                # cv2.imshow("original_r", imr_rectified_r)
                cv2.waitKey(1)

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # if hasattr(self, 'displ'):
                disparity_value = self.filtered_disparity[y, x]
                print(disparity_value)
                points_3d = cv2.reprojectImageTo3D(self.filtered_disparity, self.Q)
                print(points_3d.shape)
                depth = points_3d[int(y), int(x), 0]
                
                # depth = self.calculate_depth(disparity_value)
                rospy.loginfo(f"Disparity: {disparity_value}, Depth: {depth} meters")
                # xy = "%d,%d" % (x, y)

                # cv2.circle(img, (x, y), 1, (255, 0, 0), thickness=-1)
                # cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                #             1.0, (255, 255, 255), thickness=1)
                # cv2.imshow("image", img)


    def calculate_depth(self, disparity_value):
        points_3d = cv2.reprojectImageTo3D(disp, Q)
        if disparity_value > 0:
            depth = self.Q[2, 3] / (disparity_value + self.Q[3, 3])
            # print(depth)
            return depth
        else:
            return float('inf')


    def start(self):
        rospy.spin()
        cv2.destroyAllWindows()
        # self.vis.destroy_window()

if __name__ == '__main__':
    stereo_subscriber = endo_stereo()
    stereo_subscriber.start()



# # Load the YOLOv8 model
# model = YOLO("yolov8n.pt")

# # Open the video file
# # video_path = "path/to/your/video/file.mp4"
# cap = cv2.VideoCapture(0)

# # Loop through the video frames
# while cap.isOpened():
#     # Read a frame from the video
#     success, frame = cap.read()

#     if success:
#         # Run YOLOv8 inference on the frame
#         results = model(frame)

#         # Visualize the results on the frame
#         annotated_frame = results[0].plot()
#         print(results[0].boxes)

#         # Display the annotated frame
#         cv2.imshow("YOLOv8 Inference", annotated_frame)

#         # Break the loop if 'q' is pressed
#         if cv2.waitKey(1) & 0xFF == ord("q"):
#             break
#     else:
#         # Break the loop if the end of the video is reached
#         break

# # Release the video capture object and close the display window
# cap.release()
# cv2.destroyAllWindows()