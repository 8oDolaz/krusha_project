import config

import time

import cv2 as cv
import numpy as np
from sensors.marker_detector import Marker_Detector


marker_detector = Marker_Detector()

time.sleep(2)

try:
	while True:
		image, corners, ids = marker_detector.detect_markers(debug=True)
		image = np.ascontiguousarray(image, dtype=np.uint8)

		if len(ids):
			image = marker_detector.draw_markers(image, corners, ids)

		cv.imshow('test', image)
		cv.waitKey(50)

except KeyboardInterrupt:
	print('Goodbye!')
