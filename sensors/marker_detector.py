import config

import cv2 as cv
import numpy as np
from picamera2 import Picamera2


class Marker_Detector:
	def __init__(self, camera_config=None):
		self.camera = Picamera2()

		if camera_config is None:
			camera_config = config.default_camera_config
		camera_config = self.camera.create_preview_configuration(camera_config)
		self.camera.configure(camera_config)

		self.camera.start()


	def __take_picture(self, stream='main'):
		# Takes picture of specified stream
		# 
		# Input arguments:
		# 	stream (default 'main') - stream to take picture from, also lores available.
		# Can be changed with suited config provided in init
		# 
		# Returns BGR (with default config) image of np.array

		image = self.camera.capture_array(stream)
		image = np.array(image)
		image = image[:, :, :3]

		return image


	def detect_markers(self, aruco_type=None, debug=False):
		# Detects all ArUco markers in image
		# 
		# Input pararms:
		# 	aruco_type (not required) - type of ArUco markers
		# 	debug (default: false) - if True also return corners of markers and taken image
		# 
		# Returns just ids of detected markers if debug is diabled,
		# else image, corners and ids

		if aruco_type is None:
			aruco_type = config.aruco_type

		aruco_markers = cv.aruco.Dictionary_get(aruco_type)
		aruco_params = cv.aruco.DetectorParameters_create()

		image = self.__take_picture()
		corners, ids, rejected = cv.aruco.detectMarkers(
			image, aruco_markers, parameters=aruco_params
		) 

		if ids is not None:
			corners = corners[0]
			ids = ids.flatten()
		else:
			ids = []

		if debug:
			return (image, corners, ids)

		return ids


	@staticmethod
	def draw_markers(image, corners, ids):
		# Method draws all ArUco marker on image with it's id
		# 
		# Input parametrs:
		# 	image - np.array of shape (:, :, 3) and np.uint8 dtype
		# 	corners - np.array of corners for every marker
		# 	ids - np.array of shape (:) for every marker
		# 
		# Returns image (np.array) with drawn contours, ids and centers of markers

		line_width = 2
		line_color = (0, 0, 255)

		center_radius = 4
		center_color = (0, 0, 255)

		font_scale = 0.5
		font_thickness = 2

		for marker_corner, marker_id in zip(corners, ids):
			# Convert array to int and get every corner
			marker_corner = marker_corner.astype(np.int64)

			top_left, top_right, bottom_right, bottom_left = marker_corner

			# Draw contour of ArUco marker
			cv.line(image, top_left, top_right, line_color, line_width)
			cv.line(image, top_right, bottom_right, line_color, line_width)
			cv.line(image, bottom_right, bottom_left, line_color, line_width)
			cv.line(image, bottom_left, top_left, line_color, line_width)

			# Compute and draw center of marker
			center_x = (top_left[0] + bottom_right[0]) // 2
			center_y = (top_left[1] + bottom_right[1]) // 2

			cv.circle(image, (center_x, center_y), center_radius, center_color, -1)

			# Draw the marker ID
			id_coords = (top_left[0], top_left[1] - 15)
			cv.putText(
				image, str(marker_id), id_coords, 
				cv.FONT_HERSHEY_SIMPLEX, font_scale, line_color, font_thickness
			)

		return image
