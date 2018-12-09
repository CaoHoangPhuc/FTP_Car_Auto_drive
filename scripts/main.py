#!/usr/bin/python3
import rospy,cv2,time,os,sys
os.chdir(os.path.dirname(__file__))
import numpy as np
from multiprocessing import Process, Queue
from std_msgs.msg import Float32,String
from sensor_msgs.msg import CompressedImage
from include.lib_tdtu import Detect,Control
import yolo

def call_terminal(data):
    global terminal
    if str(data)[7] == "t": terminal=1
    else: terminal = 0

def call_speed(data):
    global pv_sp
    pv_sp = data

def get_p(data):
	try:
		while 1:IM.get(False)
	except: IM.put(data)
	temp,temp2 = 50,0
	while not RL.empty():temp = RL.get()
	RL.put(temp)
	while not ST.empty():temp2 = ST.get()
	ST.put(temp2)
	return temp,temp2

def call_image(data):
	global img
	temp = np.fromstring(data.data,np.uint8)
	img = cv2.imdecode(temp,cv2.IMREAD_COLOR)
	rl,tn = get_p(img)
	point = lane.process_image(img,50)
	speed,steer = control.drive(rl,point,tn)
	sp.publish(speed)
	st.publish(steer)

def setup(Team,debug):
    global terminal, pv_sp
    terminal, pv_sp = 0, 0
    rospy.init_node(Team)
    speed = rospy.Publisher("/"+Team+"_speed", Float32, queue_size=1)
    steer = rospy.Publisher("/"+Team+"_steerAngle", Float32, queue_size=1)
    rospy.Subscriber("/"+Team+"_image/compressed",CompressedImage,call_image)
    rospy.Subscriber("/restart",String,call_terminal)
    #rospy.Subscriber("/get_speed",Float32,call_speed)
    lane = Detect(debug)
    control = Control(debug)
    return speed,steer,lane,control

debug = 0
def main(IM,RL,ST):
    global sp,st,lane,control,terminal
    sp,st,lane,control = setup("TDTU",debug)
    try: rospy.spin()
    finally: pass

def decision(obs,rl):
	# (-) slide left
	temp,tn = 48,0
	if obs[1]>0.1: temp+= -8
	if obs[2]>0.4: 
		tn = -1
		temp+= -8
	if obs[3]>0.4: 
		tn = 1
		temp+= +12
	temp = 0.5*rl+0.5*temp
	ST.put(tn)
	RL.put(temp)
	return temp	

def You_only_look_once(Yolo):
	obstacle = [0,0,0,0,0,0]
	rl = 50
	while 1:
		res,boxes,score,classes,timer = Yolo.detect_image(IM.get())

		for i in range(len(classes)):
			obstacle[classes[i]] = score[i]\
			if score[i]>obstacle[classes[i]] else obstacle[classes[i]]
		for i in range(6):obstacle[i]*=0.96
		rl = decision(obstacle,rl)
		cv2.imshow('res',res)
		cv2.waitKey(1)
		#print(1/timer)
		#if np.amax(obstacle)>0.9: cv2.imwrite('detect_img/'+str(time.time())+'.jpg',res)

try:
	print('\nWait for initial setup...\n')
	IM,RL,ST = Queue(),Queue(),Queue()
	Yolo = yolo.YOLO()
	Process(target=main,args=(IM,RL,ST)).start()
	print('\nTesting CNN detection.\n')		
	Yolo.detect_image(cv2.imread('Training/Data/Training/1_1.jpg'))
	print('\nOK, all done.. Please restart simulation\n')
	You_only_look_once(Yolo)

finally:
	cv2.destroyAllWindows()
	#os.system('killall -9 python3')
