
# define the type of coordinate system for camera calibration: 0: X-Z ground plane; 1: X-Y ground plane (default: 0)
COORD_SYS_TYP = 1
# define the type of Vy estimation: 0: computing center of mass; 1: RANSAC (default: 1)
VY_EST_TYP = 1
# define the type of Linf estimation: 0: linear regression; 1: RANSAC (default: 1)
LINF_EST_TYP = 1
# define the type of principal point estimation: 0: assuming as the image center; 1: the point with minimum distance to the perpendicular line to Linf (default: 0)
PRIN_PT_EST_TYP = 0
# define the number of iterations in RANSAC for Vy estimation, necessary when  VY_EST_TYP = 1 (default: 100)
VY_RS_ITER_NUM = 100
# define the threshold for the distance (divided by the frame width) of RANSAC inliers to Vy, necessary when  VY_EST_TYP = 1 (default: 2.0)
VY_RS_DIST_THLD = 2.0
# define the number of iterations in RANSAC for Linf estimation, necessary when  LINF_EST_TYP = 1 (default: 100)
LINF_RS_ITER_NUM = 100
# define the threshold for the distance (divided by the frame height) of RANSAC inliers to Linf, necessary when  LINF_EST_TYP = 1 (default: 0.15)
LINF_RS_DIST_THLD = 0.20
# define the threshold for the distance (divided by the frame height) of RANSAC inliers to Linf, necessary when  LINF_EST_TYP = 1 (default: 0.15)
LINF_RS_DIST_THLD = 0.20#! define the range for focal length in ratio in EDA optimization (default: 0.2f)
EDA_RNG_F = 0.2
# define the range for principal point coordinates in pixels in EDA optimization (default: 100)
EDA_RNG_PRIN_PT = 100
# define the range for rotation angles in degrees in EDA optimization (default: 45.0)
EDA_RNG_ROT_ANG = 45.0
# define the initial population of EDA (default: 20000)
EDA_INIT_POP = 20000
# define the selected population of EDA (default: 20)
EDA_SEL_POP = 20
# define the number of iterations of EDA (default: 100)
EDA_ITER_NUM = 100
# define the threshold of ratio of reprojection errors between iterations (default: 0.10)
EDA_REPROJ_ERR_THLD = 0.01
# define image expansion ratio for plotting vanishing points and horizon line (default: 2.0f)
IMG_EXPN_RAT = 2.0