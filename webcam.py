import cv2
import time
from numpy import *
#TODO more logging options


class Cam():
	# colour filter 1 settings:
	Hue = array([0, 255], dtype=uint8)
	Saturation = array([0, 255], dtype=uint8)
	Value = array([0, 70], dtype=uint8)

	# image filter settings:
	morphologicalElementDiam = 7 #WHY?
	contourRadius = array([50, 150], dtype=uint16)
	deltaRadius = 10 # WHY?

	# output settings
	frameToShow = "default"  

	def __init__(self, logger, trackingTime = 100, startPosMinRectSize = 100, referenceTimestamp = 5):
		self.logger = logger
		
		self.trackingTime = trackingTime
		self.trackedPoints = [[0],[0],[0.]]
		self.startPosMinRectSize = startPosMinRectSize
		self.firstFrame = None
		self.totalDistance = 0
		self.averageSpeed = 0
		self.speedBtwnPoints = []  # speedBtwnPoints[i] = the speed between trackedPoints[i] and [i+1]
		self.boundingRectOfPath = None # the minimum bounding rect around the tracked path WHY?
		self.startPosMinRect = None # the rectangle around the starting position that the center of the robot should leave for larger reward
		self.robotLeavesStartRect = False
		self.invTotalDistance_LeavingRect = trackingTime * 1280  # init value, maximum possible value
		self.resultTracking = {"operationTimeStamp": referenceTimestamp}
		
		self.structureElement = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.morphologicalElementDiam, self.morphologicalElementDiam))
# perform settings:

		self.hsvThresholds = zeros((2,3), dtype= uint8)
		self.hsvThresholds[:,0] = self.Hue
		self.hsvThresholds[:,1] = self.Saturation
		self.hsvThresholds[:,2] = self.Value

	def run(self):
		vc = self.open()
		start = time.time()
		while( time.time() - start < self.trackingTime):
			self.capture(vc)
			key = cv2.waitKey(1)
			if key == 27: # exit on ESC:
				break
		self.finish(vc)

	def open(self):
		if True:
			vc = cv2.VideoCapture(0)
			cv2.namedWindow("Trackingview", cv2.WINDOW_NORMAL)
			cv2.resizeWindow("Trackingview", 960, 540) 
			rval, frame = vc.read()
			if rval:
				self.firstFrame = frame
			else:
				self.logger.error("webcam is disconnected")
			while False:
				rval, frame = vc.read()
				cv2.imshow("Trackingview", frame)
				key = cv2.waitKey(1)
				if key == 27: # exit on ESC:
					break
		return vc

	def captureTEMP(self, vc):
		rval, frame = vc.read()
		cv2.imshow("Trackingview", frame)
		print 'capture'


	def capture(self, vc):
	#INIT
		filtered_res_MorphOp_MaskParams = array([[], [], []])
		found_objects_in_frame = array([[], [], []])
		closestObjectIndex = 0  # int, i
	
		if self.structureElement.shape != (self.morphologicalElementDiam, self.morphologicalElementDiam): 
			self.structureElement = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.morphologicalElementDiam, self.morphologicalElementDiam))

		# READ FRAME 
		rval, frame = vc.read()
		if not rval:
			self.logger.error("Camera could not obtain any image")
		frameTimeStamp = time.time()
		frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		# imageOperation 2: threshold HSV image:
		maskColourThreshold = cv2.inRange(frame_hsv, self.hsvThresholds[0,:], self.hsvThresholds[1, :])
		# imageOperation 3: perform morphological filter
		res_MorphOp = cv2.morphologyEx(maskColourThreshold, cv2.MORPH_OPEN, self.structureElement)

		imageContours, contours, contourHierarchy = cv2.findContours(res_MorphOp.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		#self.firstFrame = frame.copy() # USELESS??

		# FILTER CONTOURS: of the multiple contours found,select the right one corresponding to the ball
		for i in range(0, len(contours)):
			# fit a circle around a given contour:
			fittedCenter, fittedRadius = cv2.minEnclosingCircle(contours[i])
			# determine if contour is desired based on shape of enclosing circle:
			if(fittedRadius >= self.contourRadius[0] and fittedRadius <= self.contourRadius[1]):
				# use this circle around the accepted contour as a mask on res_MorphOp to determine the best enclosing circle
				filtered_res_MorphOp = cv2.bitwise_and(res_MorphOp, res_MorphOp, mask= cv2.circle(zeros(res_MorphOp.shape, uint8), (int(round(fittedCenter[0])), int(round(fittedCenter[1]))),
														int(round(fittedRadius)) + self.deltaRadius, 1, -1))
				filtered_res_MorphOp_MaskParams = append(filtered_res_MorphOp_MaskParams, array([[int(fittedCenter[0])], [int(fittedCenter[1])], [int(fittedRadius) + self.deltaRadius]]), axis=1)
				# fit a new circle around the filtered_res_MorphOp:
				fittedCenter, fittedRadius = cv2.minEnclosingCircle(transpose(asarray(where(filtered_res_MorphOp > 0))))

				# save newly found circle in temp array: # fittedCenter[1] = x-coordinate, fittedCenter[0] = Y-coordinate
				found_objects_in_frame = append(found_objects_in_frame, array([[int(fittedCenter[1])], [int(fittedCenter[0])], [int(fittedRadius)]]), axis=1)
			#endIf valid contour found
		#endFor all found contours

		# perform logic on found objects before adding info to tracking records
		if found_objects_in_frame.shape[1] == 0:
			self.logger.warn("Tracking:Track: Could not find an object in frame")
			""
		elif found_objects_in_frame.shape[1] == 1:
			# successfully found 1 ball:
			self.trackedPoints[0].append(found_objects_in_frame[0, 0])
			self.trackedPoints[1].append(found_objects_in_frame[1, 0])
			self.trackedPoints[2].append(frameTimeStamp)
		else:
			# found multiple objects in frame
			# record the object closest to previous location
			closestOjectDistance = 1500
			for i in range(0, found_objects_in_frame.shape[1]):
				distance = linalg.norm(found_objects_in_frame[[0,1], [i, i]] - array(self.trackedPoints)[[0,1], [-1, -1]])
				if distance < closestOjectDistance:
					closestOjectDistance = distance
					closestObjectIndex = i
			# append closest distance
			self.trackedPoints[0].append(found_objects_in_frame[0, closestObjectIndex])
			self.trackedPoints[1].append(found_objects_in_frame[1, closestObjectIndex])
			self.trackedPoints[2].append(frameTimeStamp)


		# determine which frame to show
		if self.frameToShow == "maskColourThreshold":
			cv2.imshow("Trackingview", maskColourThreshold)
		elif self.frameToShow == "res_MorphOp":
			cv2.imshow("Trackingview", res_MorphOp)
		elif self.frameToShow == "contourFilters_on_res_MorphOp":
			# plot the contour filter masks as circles on res_MorphOp:
			# convert res_MorphOp to BGR image
			outputIm = zeros((res_MorphOp.shape[0], res_MorphOp.shape[1], 3), dtype = uint8)
			outputIm[res_MorphOp > 0, :] = 255
			# draw the filter areas as circles
			for i in range(0, filtered_res_MorphOp_MaskParams.shape[1]):
				outputIm = cv2.circle(outputIm, (int(round(filtered_res_MorphOp_MaskParams[0, i])), 
					int(round(filtered_res_MorphOp_MaskParams[1, i]))), int(round(filtered_res_MorphOp_MaskParams[2, i])), 
					(0,255,0) if i == closestObjectIndex else (0,0,255), 3)

			# show the image
			cv2.imshow("Trackingview", outputIm)
			#cv2.imwrite("Output/DBoutput_ContourFilter - {0}.png".format(time.strftime("%Y-%m-%d %H-%M-%S")), outputIm)
		elif self.frameToShow == "TrackedBall":
			outputIm = frame.copy()
		elif self.frameToShow == "default":
			outputIm = frame
			for i in range(0, filtered_res_MorphOp_MaskParams.shape[1]):
				outputIm = cv2.circle(outputIm, (int(round(filtered_res_MorphOp_MaskParams[0, i])), 
					int(round(filtered_res_MorphOp_MaskParams[1, i]))), int(round(filtered_res_MorphOp_MaskParams[2, i])), 
					(0,255,0) if i == closestObjectIndex else (0,0,255), 3)

			cv2.imshow("Trackingview", outputIm)
		else:
			raise NotImplementedError, "the requested option is not implemented"

	def finish(self, vc):
		# draw resulting path
		pts = transpose(array(self.trackedPoints, int32)[:, 1:])
		pathIm = cv2.polylines(self.firstFrame, [(pts[:, 0:2]).reshape((-1, 1, 2))], False, (255, 150, 0))
		cv2.imshow("Trackingview", pathIm)

		# determine minimum rectangle around start position
		minRectHalfSize = self.startPosMinRectSize / 2.
		self.startPosMinRect = ((pts[0, 0], pts[0, 1]), (self.startPosMinRectSize, self.startPosMinRectSize), 90)
		self.boundingRectOfPath = cv2.minAreaRect(pts[:, 0:2])
		resIntersection, intersectionReg = cv2.rotatedRectangleIntersection(self.startPosMinRect, self.boundingRectOfPath)

		self.robotLeavesStartRect = resIntersection == 1  # both rectangles contain the starting position => rectangles can only intersect or are fully contained
		# write rectangles screenshot
		rectsIm = cv2.drawContours(self.firstFrame, [int0(cv2.boxPoints(self.startPosMinRect))], 0, (0,0,255))
		rectsIm = cv2.drawContours(rectsIm, [int0(cv2.boxPoints(self.boundingRectOfPath))], 0, (0,255,0))
		cv2.imwrite("distance - Rects.png", rectsIm) 

		# work with float arrays:
		ptsFlt = transpose(array(self.trackedPoints)[:, 1:])
		# perform evaluation on recorded path
		for i in range(0, ptsFlt.shape[0] - 1):
			distance = linalg.norm(ptsFlt[i + 1, 0:2] - ptsFlt[i, 0:2])
			deltaTime = ptsFlt[i + 1, 2] - ptsFlt[i, 2]
			if distance == 0. or deltaTime == 0.:
				self.speedBtwnPoints.append(0.)
			else:
				self.speedBtwnPoints.append(distance / deltaTime)

			self.totalDistance += distance

		#endFor loop over tracked path
		self.averageSpeed = self.totalDistance / (ptsFlt[-1, 2] - ptsFlt[0,2])

		# calculate inverse reward here
		self.invTotalDistance_LeavingRect -= self.totalDistance
		if not self.robotLeavesStartRect:
			self.invTotalDistance_LeavingRect += 100 * self.trackingTime  # punishment
		vc.release()
		cv2.destroyWindow("Trackingview")

	@staticmethod
	def quickTest():
		vc = cv2.VideoCapture(0)
		cv2.namedWindow("Trackingview", cv2.WINDOW_NORMAL)
		cv2.resizeWindow("Trackingview", 960, 540) 
		while(True):
			rval, frame = vc.read()
			cv2.imshow("Trackingview", frame)
			key = cv2.waitKey(1)
			if key == 27: # exit on ESC:
				break
		vc.release()
		cv2.destroyWindow("Trackingview")
