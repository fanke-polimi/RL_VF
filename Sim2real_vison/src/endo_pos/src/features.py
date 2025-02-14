# import cv2
# import numpy as np


# def find_correspondence_points(img1, img2,mask1,mask2):
#     sift = cv2.xfeatures2d.SIFT_create()
#     # sift = cv2.xfeatures2d.SURF_create(400)
#     # sift = cv2.ORB_create()

#     # find the keypoints and descriptors with SIFT
#     kp1, des1 = sift.detectAndCompute(img1, mask1)
#     kp2, des2 = sift.detectAndCompute(img2, mask2)
#     # kp1, des1 = sift.detectAndCompute(
#     #     cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY), None)
#     # kp2, des2 = sift.detectAndCompute(
#     #     cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY), None)

#     # Find point matches
#     FLANN_INDEX_KDTREE = 0
#     index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
#     search_params = dict(checks=50)
#     flann = cv2.FlannBasedMatcher(index_params, search_params)
#     matches = flann.knnMatch(des1, des2, k=2)


# #     bf = cv2.BFMatcher(cv2.NORM_HAMMING)

# #  # 对描述子进行匹配
# #     matches = bf.match(des1, des2)

#     # Apply Lowe's SIFT matching ratio test
#     good = []
#     for m, n in matches:
#         if m.distance < 0.75 * n.distance:
#             good.append(m)

#     src_pts = np.asarray([kp1[m.queryIdx].pt for m in good])
#     dst_pts = np.asarray([kp2[m.trainIdx].pt for m in good])

#     # src_pts = np.asarray([kp1[m.queryIdx].pt for m in matches])
#     # dst_pts = np.asarray([kp2[m.trainIdx].pt for m in matches])

#     # Constrain matches to fit homography
#     retval, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 100.0)
#     mask = mask.ravel()

#     # We select only inlier points
#     pts1 = src_pts[mask == 1]
#     pts2 = dst_pts[mask == 1]








import cv2
import numpy as np


def find_correspondence_points(img1, img2, mask1,mask2):
    sift = cv2.xfeatures2d.SIFT_create()
    
    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(
        cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY), mask1)
    kp2, des2 = sift.detectAndCompute(
        cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY), mask2)

    # Find point matches
    # FLANN_INDEX_KDTREE = 0
    # index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    # search_params = dict(checks=50)
    # flann = cv2.FlannBasedMatcher(index_params, search_params)
    # matches = flann.knnMatch(des1, des2, k=2)

    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
    matches = bf.knnMatch(des1, des2, k=2)

    # 应用距离比率测试
    # good_matches = []
    # for m, n in matches:
    #     if m.distance < 0.75 * n.distance:
    #         good_matches.append(m)

    # Apply Lowe's SIFT matching ratio test
    good = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            good.append(m)

    src_pts = np.asarray([kp1[m.queryIdx].pt for m in good])
    dst_pts = np.asarray([kp2[m.trainIdx].pt for m in good])
    # dst_pts = np.asarray([kp2[m.queryIdx].pt for m in good])
    # print(src_pts.shape)
    # Constrain matches to fit homography
    if src_pts.shape[0] >4 :
        retval, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC)#, 100.0)))
        mask = mask.ravel()

        # We select only inlier points
        pts1 = src_pts[mask == 1]
        pts2 = dst_pts[mask == 1]
    else:
        pts1 = np.zeros((50, 2))
        pts2 = np.zeros((50, 2))

    return  matches,kp1,kp2, pts1, pts2 #,src_pts, dst_pts ,
