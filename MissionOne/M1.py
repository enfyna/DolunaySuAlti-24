from enum import Enum
from numpy import array, ones, uint8
import cv2

class State(Enum):
	SEARCH_RED_TARGET = 0
	IDENTIFY_TARGET = 1
	MOVE_TO_TARGET = 2
	LAND_ON_TARGET = 3
	NORMALIZE_POSITION = 4

class M1():
	current_state = State.SEARCH_RED_TARGET

	lower_cyan = array([40, 50, 0], int)
	upper_cyan = array([100, 255, 255], int)

	found_targets : list[float] = []
	went_forward : int = 0
	check_vertex : int = 0

	location_set : bool = False

	def FindRed(self) -> tuple[int, int, int, int] | None:
		cv2.waitKey(24)

		_, front_img = self.arac.Camera.get_front_cam()
		_, bottom_img = self.arac.Camera.get_bottom_cam()

		left_distance, _ = self.arac.Distance.getLeftDistance()

		match self.current_state:
			case State.NORMALIZE_POSITION:
				if self.went_forward > 0:
					self.went_forward -= 1
					return -1000, 0, 500, 0
				self.current_state = State.SEARCH_RED_TARGET

			case State.SEARCH_RED_TARGET:
				hx = front_img.shape[1] >> 1
				hex_x = hx >> 4
				front_img = front_img[:, hx - hex_x: hx + hex_x]

				frame = self.FindRedFromImage(front_img)
				cv2.imshow('searching', frame)

				# frame = cv2.erode(frame, ones((5, 5), uint8))
				# cv2.imshow('erode', frame)

				for row in frame:
					if any(row):
						if len(self.found_targets) > 0 and self.found_targets[-1] > left_distance - 4:
							return None
						self.found_targets.append(left_distance)
						self.current_state = State.IDENTIFY_TARGET
						return 0, 0, 500, 0
				return None

			case State.IDENTIFY_TARGET:
				frame = self.FindRedFromImage(front_img)
				cv2.imshow('center', frame)

				res = self.GetContours(frame)
				if res is None:
					self.current_state = State.SEARCH_RED_TARGET
					return None

				props = []
				for c in res:
					M = cv2.moments(c)

					cy = int(M['m01'] / M['m00']) if M['m00'] > 0 else 0
					props.append((cy, (c, M)))

				props.sort(key=lambda x: x[0], reverse=True)
				print("loop:")
				for cy, (c, M) in props:
					vc = self.GetVertexCount(c)
					print(vc)
					if cy < front_img.shape[0] * 0.8:
						self.went_forward += 1
						return 1000, 0, 500, 0
					if vc >= 6 and M['m00'] > 75:
						front_img = cv2.drawContours(front_img, [c], -1, 0xfff, 1)
						cv2.imshow('init', front_img)
						print("->>",vc)
						self.current_state = State.MOVE_TO_TARGET
						return 0, 0, 500, 0
				self.current_state = State.NORMALIZE_POSITION
				return 0, 0, 500, 0

			case State.MOVE_TO_TARGET:
				frame = self.FindRedFromImage(front_img)
				# cv2.imshow('moving', frame)
				res = self.GetContours(frame)
				print(len(res))
				props = []
				for c in res:
					M = cv2.moments(c)

					props.append((M['m00'], (c, M)))

				props.sort(key=lambda x: x[0], reverse=True)

				for area, (c, M) in props:
					if self.GetVertexCount(c) >= 6:
						front_img = cv2.drawContours(front_img, [c], -1, 0xfff, 1)
						cv2.imshow('moving', front_img)

						cx = int(M['m10'] / M['m00'])

						diff_center = int(front_img.shape[1] // 2 - cx) * 10
						if abs(diff_center) < 20:

							cy = int(M['m01'] / M['m00'])

							if cy > front_img.shape[0] * 3 // 4:
								self.current_state = State.LAND_ON_TARGET
							return 1000, 0, 500, -diff_center

						return 0, 0, 500, -diff_center

			case State.LAND_ON_TARGET:
				frame = self.FindRedFromImage(bottom_img)
				res = self.GetContours(frame)
				
				props = []
				for c in res:
					M = cv2.moments(c)

					props.append((M['m00'], (c, M)))

				props.sort(key=lambda x: x[0], reverse=True)

				if self.location_set:
					for area, (c, M) in props:
						cx = int(M['m10'] / M['m00'])
						cy = int(M['m01'] / M['m00'])
						diff_x = int(front_img.shape[1] // 2 - cx)
						diff_y = int(front_img.shape[0] // 2 - cy)
						return diff_y * 5, -diff_x * 5, 0, 0
					return 0, 0, 0, 0

				for area, (c, M) in props:
					if self.GetVertexCount(c) >= 6:
						cx = int(M['m10'] / M['m00'])
						cy = int(M['m01'] / M['m00'])
						diff_x = int(front_img.shape[1] // 2 - cx)
						diff_y = int(front_img.shape[0] // 2 - cy)
						if abs(diff_x) > 50 or abs(diff_y) > 50:
							return diff_y * 5, -diff_x * 5, 500, 0
						
						self.location_set = True
						return diff_y * 5, -diff_x * 5, 0, 0

				return 1000, 0, 500, 0


	def FindRedFromImage(self, image) -> cv2.UMat:
		# image = cv2.resize(image, (200, 150))
		image = cv2.bitwise_not(image)
		image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		image = cv2.inRange(image, self.lower_cyan, self.upper_cyan)
		return image

	@staticmethod
	def GetContourArea(contour) -> float:
		return cv2.moments(contour)['m00']

	@staticmethod
	def GetCenterPoint(moment) -> tuple[int, int]:
		return int(moment['m10'] / moment['m00']), int(moment['m01'] / moment['m00'])

	@staticmethod
	def GetVertexCount(contour) -> int:
		return len(cv2.approxPolyDP(
			contour, 0.03 * cv2.arcLength(contour, True), True
		))

	@staticmethod
	def GetContours(frame):
		return cv2.findContours(
			frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
		)[0]

	@staticmethod
	def FindLargestContour(contours, min_vertex_count = 6) -> tuple[cv2.typing.Moments, cv2.UMat]:
		current_max_area, moment, contour = 0.0, None, None

		for c in contours:
			approx = cv2.approxPolyDP(
				c, 0.01 * cv2.arcLength(c, True), True
			)
			if len(approx) < min_vertex_count:
				continue

			M = cv2.moments(c)

			if M['m00'] > current_max_area:
				moment, contour, current_max_area = M, c, M['m00']
				# Bulunan en buyuk c'yi anlik olarak kaydet
				# Not : moment -> secilen c momenti

		return moment, contour

	def SetMissionVehicle(self, vehicle) -> None:
		self.arac = vehicle
		return