from enum import Enum
from numpy import array
import cv2

class State(Enum):
	SEARCH_RED_TARGET = 0
	IDENTIFY_TARGET = 1
	MOVE_TO_TARGET = 2
	LAND_ON_TARGET = 3

class M1():
	current_state = State.SEARCH_RED_TARGET

	lower_cyan = array([40, 50, 0], int)
	upper_cyan = array([100, 255, 255], int)

	found_targets : list[float] = []

	def FindRed(self, front_img, bottom_img) -> tuple[int, int, int, int] | None:
		cv2.waitKey(24)

		cv2.imshow("front", front_img)
		cv2.imshow("bottom", bottom_img)

		match self.current_state:
			case State.SEARCH_RED_TARGET:
				frame = self.FindRedFromImage(front_img)
				cv2.imshow('searching', frame)
				res = self.GetContours(frame)
				res = self.FindLargestContour(res)
				if res is None:
					return None
				self.current_state = State.IDENTIFY_TARGET
				return 0, 0, 500, 0

			case State.IDENTIFY_TARGET:
				frame = self.FindRedFromImage(front_img)
				cv2.imshow('identify', frame)
				res = self.GetContours(frame)
				if res is None:
					self.current_state = State.SEARCH_RED_TARGET
					return 0, 0, 500, 0
				else:
					lc = self.FindLargestContour(res)
					if lc is not None:
						self.current_state = State.MOVE_TO_TARGET
						return 0, 0, 500, 0


			case State.MOVE_TO_TARGET:
				frame = self.FindRedFromImage(front_img)
				cv2.imshow('moving', frame)
				res = self.GetContours(frame)
				res = self.FindLargestContour(res, 8)
				if res is not None:
					cx = int(res['m10'] / res['m00'])
					cy = int(res['m01'] / res['m00'])
					# Secilen c'nin momentinden merkezini hesapla
					diff_center = int(front_img.shape[1] // 2 - cx) * 10
					return max(0, int(1000 -(abs(diff_center) * 2))), 0, 500, diff_center

				frame = self.FindRedFromImage(bottom_img)
				res = self.GetContours(frame)
				res = self.FindLargestContour(res, 8)
				if res is not None:
					cx = int(res['m10'] / res['m00'])
					cy = int(res['m01'] / res['m00'])
					diff_x = int(front_img.shape[1] // 2 - cx)
					diff_y = int(front_img.shape[0] // 2 - cy)
					if abs(diff_x) > 50 or abs(diff_y) > 50:
						return int(diff_y) * 5, int(diff_x) * 5, 500, 0
					self.current_state = State.LAND_ON_TARGET

				return 1000, 0, 500, 0

			case State.LAND_ON_TARGET:
				frame = self.FindRedFromImage(bottom_img)
				cv2.imshow("landing", frame)
				res = self.GetContours(frame)
				res = self.FindLargestContour(res)
				if res is not None:
					cx = int(res['m10'] / res['m00'])
					cy = int(res['m01'] / res['m00'])
					diff_x = int(front_img.shape[1] // 2 - cx)
					diff_y = int(front_img.shape[0] // 2 - cy)
					z = 500
					if abs(diff_x) < 50 or abs(diff_y) < 50:
						z = 0
					return int(diff_y) * 5, int(diff_x) * 5, z, 0
				return 0, 0, -1000, 0

	def FindRedFromImage(self, image) -> cv2.UMat:
		# image = cv2.resize(image, (200, 150))
		image = cv2.bitwise_not(image)
		image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		image = cv2.inRange(image, self.lower_cyan, self.upper_cyan)
		return image

	@staticmethod
	def GetVertexCount(contour) -> int:
		return len(cv2.approxPolyDP(
			contour, 0.01 * cv2.arcLength(contour, True), True
		))

	@staticmethod
	def GetContours(frame):
		return cv2.findContours(
			frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
		)[0]

	@staticmethod
	def FindLargestContour(contours, min_vertex_count = 6) -> cv2.typing.Moments | None:
		anlik_max_c_alani, scm = 0.0, None

		for c in contours:
			approx = cv2.approxPolyDP(
				c, 0.01 * cv2.arcLength(c, True), True
			)
			if len(approx) < min_vertex_count:
				continue

			M = cv2.moments(c)

			if M['m00'] > anlik_max_c_alani:
				scm , anlik_max_c_alani = M , M['m00']
				# Bulunan en buyuk c'yi anlik olarak kaydet
				# Not : scm -> secilen c momenti

		return scm