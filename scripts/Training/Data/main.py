import os,glob
import xml.etree.ElementTree as ET

Train = open('train.txt','w+')
Class = open('classes.txt','w+')

tempC = []

for file in glob.glob('Annot/*.xml'):
	temp = open(file,'r')
	tree = ET.parse(temp)
	root = tree.getroot()

	Train.write(root.find('path').text[35:]+' ')

	for obj in root.iter('object'):
		C = obj.find('name').text
		if C not in tempC: tempC.append(C)
		ID = tempC.index(C)
		B = obj.find('bndbox')
		Train.write(B.find('xmin').text+','+B.find('ymin').text+','+\
		B.find('xmax').text+','+B.find('ymax').text+','+str(ID)+' ')

	Train.write('\n')
		
for i in range(0,len(tempC)): Class.write(tempC[i]+'\n')