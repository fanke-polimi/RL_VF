import cv2
import numpy as np
import cam_params   #导入相机标定的参数
# import pcl
# import pcl.pcl_visualization


# 预处理
def preprocess(img1, img2):
    # 彩色图->灰度图
    if(img1.ndim == 3):#判断为三维数组
        img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)  # 通过OpenCV加载的图像通道顺序是BGR
    if(img2.ndim == 3):
        img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    # 直方图均衡
    img1 = cv2.equalizeHist(img1)
    img2 = cv2.equalizeHist(img2)

    return img1, img2


# 消除畸变
def undistortion(image, camera_matrix, dist_coeff):
    undistortion_image = cv2.undistort(image, camera_matrix, dist_coeff)

    return undistortion_image


# 获取畸变校正和立体校正的映射变换矩阵、重投影矩阵
# @param：config是一个类，存储着双目标定的参数:config = stereoconfig.stereoCamera()
def getRectifyTransform(height, width, config):
    # 读取内参和外参
    # gpu_image = cv2.cuda_GpuMat()
    # gpu_image.upload(image)
    left_K = config.cam_matrix_left
    right_K = config.cam_matrix_right
    left_distortion = config.distortion_l
    right_distortion = config.distortion_r
    R = config.R
    T = config.T

    # 计算校正变换
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(left_K, left_distortion, right_K, right_distortion, 
                                                    (width, height), R, T, alpha=0)

    map1x, map1y = cv2.initUndistortRectifyMap(left_K, left_distortion, R1, P1, (width, height), cv2.CV_32FC1)
                #    cv2.cuda.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, camera_matrix, image.shape[:2], cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(right_K, right_distortion, R2, P2, (width, height), cv2.CV_32FC1)
    # print(" Q   矩阵", Q)

    # return P1,P2, map1x, map1y, map2x, map2y, Q
    return map1x, map1y, map2x, map2y, Q


# 畸变校正和立体校正
def rectifyImage(image1, image2, map1x, map1y, map2x, map2y):
    rectifyed_img1 = cv2.remap(image1, map1x, map1y, cv2.INTER_LINEAR,cv2.BORDER_CONSTANT)
    rectifyed_img2 = cv2.remap(image2, map2x, map2y, cv2.INTER_LINEAR,cv2.BORDER_CONSTANT)

    return rectifyed_img1, rectifyed_img2


# 立体校正检验----画线
def draw_line(image1, image2):
    # 建立输出图像
    height = max(image1.shape[0], image2.shape[0])
    width = image1.shape[1] + image2.shape[1]

    output = np.zeros((height, width, 3), dtype=np.uint8)
    output[0:image1.shape[0], 0:image1.shape[1]] = image1
    output[0:image2.shape[0], image1.shape[1]:] = image2

    # 绘制等间距平行线
    line_interval = 50  # 直线间隔：50
    for k in range(height // line_interval):
        cv2.line(output, (0, line_interval * (k + 1)), (2 * width, line_interval * (k + 1)), (0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

    return output


# 视差计算
def stereoMatchSGBM(left_image, right_image, down_scale=False):
    # SGBM匹配参数设置
    if left_image.ndim == 2:
        img_channels = 1
    else:
        img_channels = 3
    blockSize = 5
    paraml = {'minDisparity': 0,
             'numDisparities': 128,
             'blockSize': blockSize,
             'P1': 32 * img_channels * blockSize ** 2,
             'P2': 64 * img_channels * blockSize ** 2,
             'disp12MaxDiff': 10,
             'preFilterCap': 10,
             'uniquenessRatio': 10,
             'speckleWindowSize': 10,
             'speckleRange': 10,
             'mode': cv2.STEREO_SGBM_MODE_SGBM_3WAY
             }
    # left_matcher  = cv2.StereoSGBM_create(minDisparity=1,
    #                                numDisparities=32,
    #                                blockSize=blockSize,
    #                                P1=8 * img_channels * blockSize * blockSize,
    #                                P2=32 * img_channels * blockSize * blockSize,
    #                                disp12MaxDiff=-1,
    #                                preFilterCap=1,
    #                                uniquenessRatio=10,
    #                                speckleWindowSize=100,
    #                                speckleRange=100,
    #                                mode=cv2.STEREO_SGBM_MODE_HH)

    # 构建SGBM对象
    left_matcher = cv2.StereoSGBM_create(**paraml)
    paramr = paraml
    paramr['minDisparity'] = -paraml['numDisparities']
    right_matcher = cv2.StereoSGBM_create(**paramr)

    # 计算视差图
    size = (left_image.shape[1], left_image.shape[0])
    if down_scale == False:
        disparity_left = left_matcher.compute(left_image, right_image)
        disparity_right = right_matcher.compute(right_image, left_image)

    else:
        left_image_down = cv2.pyrDown(left_image)
        right_image_down = cv2.pyrDown(right_image)
        factor = left_image.shape[1] / left_image_down.shape[1]

        disparity_left_half = left_matcher.compute(left_image_down, right_image_down)
        disparity_right_half = right_matcher.compute(right_image_down, left_image_down)
        disparity_left = cv2.resize(disparity_left_half, size, interpolation=cv2.INTER_AREA)
        disparity_right = cv2.resize(disparity_right_half, size, interpolation=cv2.INTER_AREA)
        disparity_left = factor * disparity_left
        disparity_right = factor * disparity_right

    # 真实视差（因为SGBM算法得到的视差是×16的）
    # trueDisp_left = disparity_left.astype(np.float32) /16
    displ = np.int16(disparity_left)
    dispr = np.int16(disparity_right)
    # trueDisp_left = np.divide(disparity_left.astype(np.float32), 16.)
    # trueDisp_right = disparity_right.astype(np.float32) / 16

    

    return displ, dispr, left_matcher



# 利用opencv函数计算深度图
def getDepthMapWithQ(disparityMap : np.ndarray, Q : np.ndarray) -> np.ndarray:
    points_3d = cv2.reprojectImageTo3D(disparityMap, Q)
    depthMap = points_3d[:, :, 2]
    reset_index = np.where(np.logical_or(depthMap < 0.0, depthMap > 65535.0))
    depthMap[reset_index] = 0
 
    return depthMap.astype(np.float32)
 
 
# 根据公式计算深度图
# def getDepthMapWithConfig(disparityMap : np.ndarray, config : stereoconfig.stereoCamera) -> np.ndarray:
#     fb = config.cam_matrix_left[0, 0] * (-config.T[0])
#     doffs = config.doffs
#     depthMap = np.divide(fb, disparityMap + doffs)
#     reset_index = np.where(np.logical_or(depthMap < 0.0, depthMap > 65535.0))
#     depthMap[reset_index] = 0
#     reset_index2 = np.where(disparityMap < 0.0)
#     depthMap[reset_index2] = 0
#     return depthMap.astype(np.float32)


# # 将h×w×3数组转换为N×3的数组
def hw3ToN3(points):
    height, width = points.shape[0:2]
 
    points_1 = points[:, :, 0].reshape(height * width, 1)
    points_2 = points[:, :, 1].reshape(height * width, 1)
    points_3 = points[:, :, 2].reshape(height * width, 1)
 
    points_ = np.hstack((points_1, points_2, points_3))
 
    return points_
 
 
# # 深度、颜色转换为点云
def DepthColor2Cloud(points_3d, colors):
    rows, cols = points_3d.shape[0:2]
    size = rows * cols
 
    points_ = hw3ToN3(points_3d)
    colors_ = hw3ToN3(colors).astype(np.int64)
 
    # 颜色信息
    blue = colors_[:, 0].reshape(size, 1)
    green = colors_[:, 1].reshape(size, 1)
    red = colors_[:, 2].reshape(size, 1)
 
    rgb = np.left_shift(blue, 0) + np.left_shift(green, 8) + np.left_shift(red, 16)
 
    # 将坐标+颜色叠加为点云数组
    pointcloud = np.hstack((points_, rgb)).astype(np.float32)
 
    # 删掉一些不合适的点
    X = pointcloud[:, 0]
    Y = pointcloud[:, 1]
    Z = pointcloud[:, 2]
 
    # 下面参数是经验性取值，需要根据实际情况调整
    remove_idx1 = np.where(Z <= 0)
    remove_idx2 = np.where(Z > 15000)  #// 注意单位是mm
    remove_idx3 = np.where(X > 10000)
    remove_idx4 = np.where(X < -10000)
    remove_idx5 = np.where(Y > 10000)
    remove_idx6 = np.where(Y < -10000)
    remove_idx = np.hstack((remove_idx1[0], remove_idx2[0], remove_idx3[0], remove_idx4[0], remove_idx5[0], remove_idx6[0]))
 
    pointcloud_1 = np.delete(pointcloud, remove_idx, 0)
 
 
    return pointcloud_1
 
 
# # 点云显示
# def view_cloud(pointcloud):
#     cloud = pcl.PointCloud_PointXYZRGBA()
#     cloud.from_array(pointcloud)
 
#     try:
#         visual = pcl.pcl_visualization.CloudViewing()
#         visual.ShowColorACloud(cloud)
#         v = True
#         while v:
#             v = not (visual.WasStopped())
#     except:
#         pass

#     return disparity_left, trueDisp_right


# # 将h×w×3数组转换为N×3的数组
# def hw3ToN3(points):
#     height, width = points.shape[0:2]

#     points_1 = points[:, :, 0].reshape(height * width, 1)
#     points_2 = points[:, :, 1].reshape(height * width, 1)
#     points_3 = points[:, :, 2].reshape(height * width, 1)

#     points_ = np.hstack((points_1, points_2, points_3))

#     return points_


# # 深度、颜色转换为点云
# def DepthColor2Cloud(points_3d, colors):
#     rows, cols = points_3d.shape[0:2]
#     size = rows * cols

#     points_ = hw3ToN3(points_3d)
#     colors_ = hw3ToN3(colors).astype(np.int64)

#     # 颜色信息
#     blue = colors_[:, 0].reshape(size, 1)
#     green = colors_[:, 1].reshape(size, 1)
#     red = colors_[:, 2].reshape(size, 1)

#     rgb = np.left_shift(blue, 0) + np.left_shift(green, 8) + np.left_shift(red, 16)

#     # 将坐标+颜色叠加为点云数组
#     pointcloud = np.hstack((points_, rgb)).astype(np.float32)

#     # 删掉一些不合适的点
#     X = pointcloud[:, 0]
#     Y = -pointcloud[:, 1]
#     Z = -pointcloud[:, 2]

#     remove_idx1 = np.where(Z <= 0)
#     remove_idx2 = np.where(Z > 15000)
#     remove_idx3 = np.where(X > 10000)
#     remove_idx4 = np.where(X < -10000)
#     remove_idx5 = np.where(Y > 10000)
#     remove_idx6 = np.where(Y < -10000)
#     remove_idx = np.hstack((remove_idx1[0], remove_idx2[0], remove_idx3[0], remove_idx4[0], remove_idx5[0], remove_idx6[0]))

#     pointcloud_1 = np.delete(pointcloud, remove_idx, 0)

#     return pointcloud_1