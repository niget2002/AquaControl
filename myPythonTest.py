#!/usr/bin/env python

from __future__ import division
import logging
import subprocess
from subprocess import PIPE, Popen, call
import re
import requests
import os.path
import sys
import psutil
from datetime import datetime,time
from functools import partial
from time import sleep
import csv
import serial
import ConfigParser

import kivy
from kivy.app import App
from kivy.clock import Clock
from kivy.uix.widget import Widget
from kivy.uix.button import Button
from kivy.uix.popup import Popup
from kivy.uix.progressbar import ProgressBar
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout

Config=ConfigParser.ConfigParser()
# values are TOTAL - 1 for 0
numLights=2
numPumps=4
numOther=4
numAlarms=4
mylogger = logging.getLogger('simpleExample')
mylogger.setLevel(logging.DEBUG)
fh = logging.FileHandler('/var/log/aquarium.log')
fh.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
ch.setFormatter(formatter)
fh.setFormatter(formatter)

mylogger.addHandler(ch)
mylogger.addHandler(fh)

def get_uptime():
	with open('/proc/uptime', 'r') as f:
		uptime_seconds = float(f.readline().split()[0])
	return uptime_seconds

def is_running(process):
	s = subprocess.Popen(["ps", "axw"],stdout=subprocess.PIPE)
	for x in s.stdout:
		if re.search(process, x):
			return True
	command = "/usr/bin/fbcp"
	process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
	return False

def getCpuTemperature():
	process = Popen(['vcgencmd', 'measure_temp'], stdout=PIPE)
	output, _error = process.communicate()
	celsius = float(output[output.index('=') + 1:output.rindex("'")])
	return 9.0/5.0 * celsius + 32

def celsiusToFer(celsius):
	return 9.0/5.0 * celsius + 32

def ConfigSectionMap(section):
	dict1 = {}
	options = Config.options(section)
	for option in options:
		try:
			dict1[option] = Config.get(section, option)
			if dict1[option] == -1:
				DebugPrint("skip: %s" % option)
		except:
			print("exception on %s!" % option)
			dict1[option] = None
	return dict1

def sendToServer(pump0,pump1,pump2,pump3,heater,chiller,co2,om,temperature,ph0,ph1,sw0,sw1,sw2,sw3,sw4):

	dataCSV = pump0+","+pump1+","+pump2+","+pump3+","+heater+","+chiller+","+co2+","+om+","+temperature+","+ph0+","+ph1+","+sw0+","+sw1+","+sw2+","+sw3+","+sw4

	url = ConfigSectionMap("website")["url"]
	blogin = ConfigSectionMap("website")["blogin"]
	passin = ConfigSectionMap("website")["passin"]
	payload = {'blogin': blogin, 'passin': passin, 'stringdata': dataCSV}

	# POST with form-encoded data
	try:
		r = requests.get(url, params=payload)
	except:
		mylogger.info('URL Update Failed')
	else:
		mylogger.info('%s', payload)

def shutdownSystem(self) :
	command = "/sbin/halt"
	process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
	output = process.communicate()[0]
	print output


class MyWidget(BoxLayout,Widget):
    	def __init__(self, **kwa):
		super(MyWidget, self).__init__(**kwa)
		while(os.path.exists('/dev/ttyACM0')==False) :
			print "Still waiting or ttyACM0"
			sleep(30)
			continue

		logging.info('Started')

		self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
		global Config
		Config.read("/home/pi/aquarium.ini")

		self.orientation = 'vertical'
		self.spacing = 10
		self.padding = [5,5,5,5]

		myFont='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'

		#Create Labels
		self.labels = [Label(text="Initialize Time"), Label(text="pH1"), Label(text="pH2"), Label(text="Temperature"), Label(text="CurrState",markup=True)]
		for i, val in enumerate(self.labels) :
			self.labels[i].font_size=30
			self.labels[i].font_name=myFont
			self.add_widget(self.labels[i])

		self.admin = Button(text='Managment', font_size=34, size_hint=(1,3), font_name=myFont)
		self.admin.background_color = [.5,0,0,1]
		self.admin.bind(on_press=self.createAdminPopup)

		self.aLayout=BoxLayout()
		self.aLayout.orientation='vertical'
		self.pLayout=BoxLayout()
		self.pLayout.orientation='vertical'
		self.oLayout=BoxLayout()
		self.oLayout.orientation='vertical'

		self.aPopup=Popup(title="Maintenance", content=self.aLayout)
		self.pPopup=Popup(title="Pumps", content=self.pLayout)
		self.oPopup=Popup(title="Other", content=self.oLayout)

		self.maint = Button(text='Maintenance', font_size=32, size_hint=(1,3), font_name=myFont)
		self.maint.background_color = [0,.5,0,1]
		self.maint.bind(on_press=self.openMaint)

		self.maintpButton = Button(text="Pumps", font_size=32, size_hint=(1,5), font_name=myFont)
		self.maintpButton.bind(on_press=self.openPumps)
		self.maintoButton = Button(text="Other", font_size=32, size_hint=(1,5), font_name=myFont)
		self.maintoButton.bind(on_press=self.openOther)

		self.maintaClose = Button(text="Close", font_size=32, size_hint=(1,5), font_name=myFont)
		self.maintpClose = Button(text="Close", font_size=32, size_hint=(1,5), font_name=myFont)
		self.maintoClose = Button(text="Close", font_size=32, size_hint=(1,5), font_name=myFont)


		self.aLayout.add_widget(self.maintpButton)
		self.aLayout.add_widget(self.maintoButton)
		self.aLayout.add_widget(self.maintaClose)
		self.maintaClose.bind(on_press=self.aPopup.dismiss)

		self.pump=[0 for i in range(numPumps)]
		self.buttons=[0 for i in range(numPumps)]
		for i in range(numPumps) :
			thisOne="pumpname"+str(i)
			self.buttons[i]=Button(text=ConfigSectionMap("pumps")[thisOne])
                       	self.buttons[i].font_size=34
                       	self.buttons[i].size_hint=(1,5)
                       	self.buttons[i].font_name=myFont
                       	self.buttons[i].bind(on_press=partial(self.buttonSwap,i,'P'))
                       	self.pLayout.add_widget(self.buttons[i])
			self.pump[i]=0
		self.pLayout.add_widget(self.maintpClose)
		self.maintpClose.bind(on_press=self.pPopup.dismiss)


		self.other=[0 for i in range(numOther)]
                self.others=[0 for i in range(numOther)]
                for i in range(numOther) :
			thisOne="othername"+str(i)
                        self.others[i]=Button(text=ConfigSectionMap("other")[thisOne])
                        self.others[i].font_size=34
                        self.others[i].size_hint=(1,5)
                        self.others[i].font_name=myFont
                        self.others[i].bind(on_press=partial(self.buttonSwap,i,'O'))
                        self.oLayout.add_widget(self.others[i])
                        self.other[i]=0
                self.oLayout.add_widget(self.maintoClose)
                self.maintoClose.bind(on_press=self.oPopup.dismiss)

		self.alarms=[0 for i in range(numAlarms)]

		self.phoffset = [ 0,0 ]

		#C eate Maintenance Panel
		self.add_widget(self.maint)
		#Create Admin Panel
		self.add_widget(self.admin)

		Clock.schedule_interval(self.updateData, 10)

		self.lighton = [ 0,0 ]
		self.lightoff = [ 0,0 ]
		for i, val in enumerate(self.lighton) :
			thisOne="lighton"+str(i)
			thisToo="lightoff"+str(i)
	        	self.lighton[i]=ConfigSectionMap("lights")[thisOne]
	        	self.lightoff[i]=ConfigSectionMap("lights")[thisToo]

		self.firstrun=1
		self.getLatest=0
		self.sendVals=0
		self.adminBuild=1
		self.light=[ 0,0 ]


    	def updateData(self, dt) :
		t = datetime.now()
		curTime=t.strftime("%I:%M:%S")
		self.labels[0].text = curTime

		if self.firstrun == 1 :
			print "Setup"
			self.ser.write('X')

			for i, val in enumerate(self.pump) :
				thisOne="pump"+str(i)
				self.pump[i]=ConfigSectionMap("pumps")[thisOne]
                		self.sendSerial('P',str(i),str(self.pump[i]))
                		if self.pump[i] == '1' :
                        		print "Pump Is Running"
					self.stopButton('P',i)
					sleep(10)
                		else :
       		                	print "Pump Not Running"
					self.startButton('P',i)

                        for i, val in enumerate(self.other) :
                                thisOne="other"+str(i)
                                self.other[i]=ConfigSectionMap("other")[thisOne]
                                self.sendSerial('O',str(i),str(self.other[i]))
                                if self.other[i] == '1' :
                                        print "Other Is Running"
					self.stopButton('O',i)
					sleep(2)
                                else :
                                        print "Other Not Running"
					self.startButton('O',i)
			self.firstrun=0

		oldLight = [0,0]

		for i, val in enumerate(self.light) :
			oldLight[i]=self.light[i]
			self.light[i]=self.checkTime(self.lighton[i],self.lightoff[i])
        		if oldLight[i] != self.light[i] :
            			self.sendSerial('L',str(i),str(self.light[i]))
				sleep(2)

		self.sendSerial('h','1','1')
           	ph1R = self.ser.readline()
		self.sendSerial('i','1','1')
		ph2R = self.ser.readline()
		self.sendSerial('t','1','1')
		tempR = self.ser.readline()
		tempR = tempR.strip('\n')
		tempR = tempR.strip('\t')
		ph1R = ph1R.strip('\n')
		ph1R = ph1R.strip('\t')
		ph2R = ph2R.strip('\n')
		ph2R = ph2R.strip('\t')
		self.labels[1].text = ph1R
		self.labels[2].text = ph2R
		self.labels[3].text = str(celsiusToFer(float(tempR)))

		self.getLatest = self.getLatest+1
		if self.getLatest == 2 :
			self.getLatest=0
			self.labels[4].text="L"
			for i in range(numLights):
				self.sendSerial('R','L',str(i))
				rdat=self.ser.readline()
				rdat = rdat.strip('\n')
				sdat = int(rdat)
				if sdat == 0:
					self.light[i]=1
					self.labels[4].text=self.labels[4].text+"[color=33ff33]"+str(i)+"[/color]"
				else :
					self.light[i]=0
					self.labels[4].text=self.labels[4].text+"[color=3333ff]"+str(i)+"[/color]"
			self.labels[4].text=self.labels[4].text+' P'
			for i in range(numPumps):
				self.sendSerial('R','P',str(i))
				rdat=self.ser.readline()
				rdat = rdat.strip('\n')
				sdat = int(rdat)
				if sdat == 0:
					self.pump[i]=1
					self.stopButton('P',i)
					self.labels[4].text=self.labels[4].text+"[color=33ff33]"+str(i)+"[/color]"
				else :
					self.pump[i]=0
					self.startButton('P',i)
					self.labels[4].text=self.labels[4].text+"[color=3333ff]"+str(i)+"[/color]"
			self.labels[4].text=self.labels[4].text+' O'
			for i in range(numOther):
				self.sendSerial('R','O',str(i))
				rdat=self.ser.readline()
				rdat = rdat.strip('\n')
				sdat = int(rdat)
				if sdat == 0:
					self.other[i]=1
					self.stopButton('O',i)
					self.labels[4].text=self.labels[4].text+"[color=33ff33]"+str(i)+"[/color]"
				else :
					self.other[i]=0
					self.startButton('O',i)
					self.labels[4].text=self.labels[4].text+"[color=3333ff]"+str(i)+"[/color]"

			self.labels[4].text=self.labels[4].text+' A'
			for i in range(numAlarms):
				self.sendSerial('A','O',str(i))
				rdat=self.ser.readline()
				rdat = rdat.strip('\n')
				sdat = int(rdat)
				if sdat == 0:
					self.alarms[i]=1
					self.labels[4].text=self.labels[4].text+"[color=33ff33]"+str(i)+"[/color]"
				else :
					self.alarms[i]=0
					self.labels[4].text=self.labels[4].text+"[color=3333ff]"+str(i)+"[/color]"

		self.sendVals = self.sendVals+1
		if self.sendVals == 6 :
			self.sendVals=0
			sendToServer(str(self.pump[0]),str(self.pump[1]),str(self.pump[2]),str(self.pump[3]),str(self.other[0]),str(self.other[1]),str(self.other[2]),str(self.other[3]),self.labels[3].text,self.labels[1].text,self.labels[2].text,str(self.alarms[0]),str(self.alarms[1]),str(self.alarms[2]),str(self.alarms[3]),'0')



	def checkTime(self, timeOn, timeOff) :
        	now = datetime.now()
        	now_time = now.time()
        	if time(int(timeOn[0:2]),int(timeOn[2:4])) <= now.time() <= time(int(timeOff[0:2]),int(timeOff[2:4])):
            		return 1
        	else :
            		return 0

	def sendSerial(self,type,device,value) :
		self.ser.write(type)
		self.ser.write(device)
		self.ser.write(value)
		self.ser.write('0')
		self.ser.write('0')
		self.ser.write('0')
		self.ser.write('X')

	def stopButton(self, type, instance) :
		if type == 'P' :
			thisOne="pumpname"+str(instance)
			thisType="pumps"
			self.buttons[instance].text = ConfigSectionMap(thisType)[thisOne]+" Stop"
			self.buttons[instance].background_color=[0,0,255,1]

		if type == 'O' :
			thisOne="othername"+str(instance)
			thisType="other"
			self.others[instance].text = ConfigSectionMap(thisType)[thisOne]+" Stop"
			self.others[instance].background_color=[0,0,255,1]

	def startButton(self, type, instance) :
		if type == 'P' :
			thisOne="pumpname"+str(instance)
			thisType="pumps"
			self.buttons[instance].text = ConfigSectionMap(thisType)[thisOne]+" Start"
			self.buttons[instance].background_color=[0,255,0,1]

		if type == 'O' :
			thisOne="othername"+str(instance)
			thisType="other"
			self.others[instance].text = ConfigSectionMap(thisType)[thisOne]+" Start"
			self.others[instance].background_color=[0,255,0,1]

	def buttonSwap(self, pump, type, instance) :
		if type == 'P' :
			if self.pump[pump]==1:
				self.startButton(type,pump)
				self.pump[pump]=0
			else:
				self.stopButton(type,pump)
				self.pump[pump]=1
			sendme=self.pump[pump]
			if pump==0:
				self.sendSerial('A','R','0')
				sleep(2)

		if type == 'O' :
			if self.other[pump]==1:
				self.startButton(type,pump)
				self.other[pump]=0
			else:
				self.stopButton(type,pump)
				self.other[pump]=1
			sendme=self.other[pump]
		self.sendSerial(type,str(pump),str(sendme))
		return

	def openMaint(self, instance) :
		self.aPopup.open()

	def openPumps(self, instance):
		self.pPopup.open()

	def openOther(self, instance) :
		self.oPopup.open()

	def createAdminPopup(self, instance) :
                myFont='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
                shutdown = Button(text='Shutdown System', font_size=34, size_hint=(1,5), font_name=myFont)
                shutdown.background_color = [255,0,0,1]
                shutdown.bind(on_press=shutdownSystem)
		content = Button(text="Close Me", font_size=34, size_hint=(1,5), font_name=myFont)
		pLayout = BoxLayout()
		pLayout.orientation="vertical"
		pLayout.add_widget(shutdown)
		pLayout.add_widget(content)
		popup = Popup(title="Admin Panel", content=pLayout)
		content.bind(on_press=popup.dismiss)
		popup.open()



class MyApp(App):
	def build(self):
		return MyWidget()

if __name__ == '__main__':
	is_running('fbcp')
	MyApp().run()
