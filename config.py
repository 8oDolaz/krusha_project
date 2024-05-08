import cv2 as cv

# Camera configuration
default_camera_config = {
	'format': 'XRGB8888',
	'size': (800, 600)
}

# ArUco dictionary
aruco_type = cv.aruco.DICT_4X4_1000
