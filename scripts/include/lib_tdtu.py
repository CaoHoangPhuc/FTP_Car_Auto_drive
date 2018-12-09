import cv2,imutils,time
from copy import copy
import numpy as np

class Detect(object):
	"""docstring for Detect"""
	def __init__(self, debug):
		self.debug = debug
		self.lower_hsv = np.array([[0, 0, 0],[40, 5, 90],[85, 20, 35],[30, 0, 70]])
		self.higher_hsv = np.array([[0, 0, 0],[0, 0, 0],[115, 105, 75],[90, 25, 100]])
		self.dilate = (3,3)
		self.erode = (2,4)
		self.gauss = (3,3)
		self.bin_thr = 10
		self.thr_area = 30
		self.source = np.float32([[120,80],[200,80],[0,150],[320,150]])
		self.destin = np.float32([[0,0],[100,0],[0,200],[100,200]])

	def filter(self,data):		
		if self.debug: cv2.imshow('Original',data)

		frame = cv2.getPerspectiveTransform(self.source,self.destin)
		frame = cv2.warpPerspective(data,frame,(100,200))
		#frame = cv2.GaussianBlur(frame,self.gauss,0)
		if self.debug: cv2.imshow('Perspective',frame) 

		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		temp = []
		for i in range(0,len(self.lower_hsv)):
			mask = cv2.inRange(hsv, self.lower_hsv[i], self.higher_hsv[i])
			temp.append(cv2.bitwise_and(frame, frame, mask=mask))
		if self.debug: cv2.imshow('HSV_Object0',temp[0])

		for i in range(1,len(self.lower_hsv)):temp[0] = cv2.bitwise_or(temp[0],temp[i])
		frame = temp[0]
		if self.debug: cv2.imshow('HSV_Total_Objects',frame)

		frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)		
		frame = cv2.morphologyEx(frame,cv2.MORPH_CLOSE,np.ones(self.dilate,np.uint8))
		frame = cv2.erode(frame,np.ones(self.erode,np.uint8),iterations=1)
		#frame = cv2.dilate(frame,np.ones(self.dilate,np.uint8),iterations=1) 
		if self.debug: cv2.imshow('dilate_erode',frame)

		frame = cv2.threshold(frame,self.bin_thr,255,cv2.THRESH_BINARY)	[1]
		if self.debug: cv2.imshow('Final_filter',frame)
		
		return frame

	def get_point(self,data,val):
		temp = [[] for i in range(0,val)]
		point = [[] for i in range(0,val)]
		low = int(data.shape[0]/val)
		high = int(data.shape[0]/val)-1
		for i in range(0,val):
			point[i] = cv2.findContours(data[i*low:i*low+high,:],\
				cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
			point[i] = point[i][0] if imutils.is_cv2() else point[i][1]
			for c in point[i]:
				M= cv2.moments(c)
				if M['m00'] >self.thr_area:
					cx= int(M["m10"]/(M["m00"]+0.0001))
					cy= int(M["m01"]/(M["m00"]+0.0001))+low*i
					cv2.circle(data,(cx,cy),1,(0,0,0),-1)
					temp[i].append((cx,cy)) 
		if self.debug: cv2.imshow('Maker',data)

		return np.array(temp)

	def process_image(self,data,val):
		frame = self.filter(data)
		point = self.get_point(frame,val)
		return point

	def update(self,mode,arg):
		if mode:
			self.lower_hsv,self.higher_hsv = arg[0],arg[1]
			self.dilate,self.erode = arg[2],arg[3]
			self.gauss,self.bin_thr = arg[4],arg[5]
			self.thr_area = arg[6]
		else:
			return self.lower_hsv,self.higher_hsv,self.dilate,self.erode,\
			self.gauss,self.bin_thr,self.thr_area


class Control(object):
	"""docstring for Control"""
	def __init__(self, debug):
		self.debug = debug
		self.Kpid = [1.8,0.3,0.6]
		self.Hpid = [0.,0.,0.,time.time()]
		self.range_st = [-50,50]
		self.range_sp = [40,60,90]
		self.Speed = self.range_sp[2]
		self.Steer = 0.

	def Sp_control(self,point,pos,tn):
		pt = [-1,0]
		for i in range(0,len(point)):
			if len(point[i]) == 1: 
				if pt[0] == -1: pt[0] = point[i][0][0]
				pt = [0.8*pt[0]+0.2*point[i][0][0],pt[1]+1]
			elif len(point[i]) == 2:
				pt[1]+=1
				pt[0] = 0.8*pt[0]+0.1*(point[i][0][0]+point[i][1][0])
		if tn<0: self.Speed = self.range_sp[0]
		elif tn>0: self.Speed = self.range_sp[0]
		elif pt[1]<30: self.Speed = self.range_sp[0]
		elif pt[1]<44: self.Speed = self.range_sp[1]
		else: self.Speed = self.range_sp[2]#1 if self.Speed < self.range_sp[2] else 0		
		return pt

	def St_control(self,pos,pt):
		dt = time.time()-self.Hpid[3]
		error = pt[0]- pos
		p = error*self.Kpid[0]
		i = 0.9*self.Hpid[0] + self.Kpid[1]*error*dt
		d = self.Kpid[2]*(error-self.Hpid[1])+0.1*self.Hpid[2]
		out = self.range_st[1] if (p+i+d)>self.range_st[1] else (p+i+d)
		out = self.range_st[0] if (p+i+d)<self.range_st[0] else (p+i+d)
		self.Hpid = i,error,d,time.time()
		self.Steer = out

	def drive(self,pos,point,tn):
		pt = self.Sp_control(point,pos,tn)
		self.St_control(pos,pt)
		return self.Speed,self.Steer

	def update(self,mode,arg):
		if mode:
			self.Kpid = arg[0]
			self.range_st = arg[1]
			self.range_sp = arg[2]
		else:
			return self.Kpid,self.range_st,self.range_sp



