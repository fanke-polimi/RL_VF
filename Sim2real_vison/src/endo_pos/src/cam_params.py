import numpy as np


####################仅仅是一个示例###################################


# 双目相机参数
class stereoCamera(object):
    def __init__(self):
        # 左相机内参1766.668333, 0.0, 820.5461251898, 0.0, 1772.4568118, 615.876298, 0.0, 0.0, 1.0
       
        # self.cam_matrix_left = np.array([   [594.740585514313,	0,	998.834903029934],
        #                                     [0,	593.867182483117,	523.477821056540],
        #                                     [       0.0,         0.0,         1.0]
        #                                 ])
        self.cam_matrix_left = np.array([   [595.030335934627,	0,	998.310793867376],
                                            [0,	594.224429631422,	524.342417190547],
                                            [       0.0,         0.0,         1.0]
                                        ])
        
        # 右相机内参
        # self.cam_matrix_right = np.array([   [595.676020529071,	0,	968.515619605628],
        #                                      [0,	594.881186546266,	528.288600579002],
        #                                     [       0.0,         0.0,         1.0]
        #                                 ])
        self.cam_matrix_right = np.array([  [596.297770053963,	0,	968.155576879698],
                                            [0,	595.599563117171,	529.027451418728],
                                            [      0.0,	0.0,	1.0]
                                        ])
        

        # 左右相机畸变系数:[k1, k2, p1, p2, k3]
        # self.distortion_l = np.array([[0.00900730804342922,	-0.0272125067103261,	-0.000145654657654917,	-0.000142512881231350,	0.00426644094737014]])
        # self.distortion_r = np.array([[0.00964274027559225,	-0.0283608557361426,	-0.000211695514538673,	0.000146478854172871,	0.00483240597972475]])
        self.distortion_l = np.array([[0.0102759524664842,	-0.0281907871027420, -7.57488700682001e-06,	-0.000432049924196620,	0.00449774409692344]])
        self.distortion_r = np.array([[0.0102379041259811,	-0.0284709370241626, -0.000219717757441214,	-4.87558997145740e-05,	0.00475500527923818]])

        # 旋转矩阵0.9792737230184961, 0.06813883616331005, -0.1907356139052262, -0.07022721121792985, 0.9975221669408194, -0.004203006809919951, 0.1899766149031143, 0.01751072437112624, 0.9816324466529985
        # 0.9999221419601694 -0.00835058176818008 -0.009272421577981228 0.008042339532302567 0.9994296821630961 -0.03279681670863498 0.009541005850225475 0.032719691250156585 0.9994190267409664
        self.R = np.array([ [ 0.999985189640651,	0.000797097611089761,	0.00538378442632721],
[-0.000810139550677748,	0.999996741932381,	0.00242070207423903],
[-0.00538183734975297,	-0.00242502783946751,	0.999982577381586]   
                            ])
#         self.R = np.array([ [0.999983206499056,	0.000843812321026910,	0.00573364636445306],
# [-0.000856280228873169,	0.999997273706642,	0.00217240959619944],
# [-0.00573179762686746,	-0.00217728272185807,	0.999981202791289]   
#                             ])

        # 平移矩阵
        # self.T = np.array([[-60.0760974237901],	[0.163615867358755],	[0.239738899566852]])
        self.T = np.array([[-60.0497788968993],	[0.170115478009263],	[0.377948083005438]])

        # 焦距
        # self.focal_length = 859.367 # 默认值，一般取立体校正后的重投影矩阵Q中的 Q[2,3]

        # 基线距离
        self.baseline = 0.00565332185606285 # 单位：mm， 为平移向量的第一个参数（取绝对值）

