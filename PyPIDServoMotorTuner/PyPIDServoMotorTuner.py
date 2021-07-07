#!/usr/bin/env python

# This Python script is a companion app

'''**********************************************************************************************
 * Python Servo Motor PID Tuner
 * by patlhy
 *
 * This is a companion software for DC Servo Motor controller in
 * https://github.com/patlhy/PID-DC-Motor-Servo-Controller/tree/master/ArduinoTB6612PIDController
 *
 * This PID visualization and tuning Software is used it to change the PID values and see how the
 * DC motor would response to a step response. The software uses the USB COM port to send command
 * to the Arduino Nano or STM32 board and read the motor step response from the controller memory.
 *
 * Detailed instruction on how to use it can be found in:
 * https://hackaday.io/project/178310-stepper-to-dc-motor-conversion
 **********************************************************************************************'''

from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import re
import serial
import time

#Main GUI window initialization
class Window(QtGui.QWidget):
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)

        self.connectstatus=False
        COMlabel = QtGui.QLabel("COM Port")

        try:
          with open("PIDserial.cnf", 'r') as initfile:
              fline = initfile.readline()
              matchCOM = re.search(r'^COM(\d+)', fline)
              if matchCOM:
                cmprt = str(matchCOM.group(1))
              else:
                cmprt = "1"
        except Exception as inst:
              if re.search(r'No such file or directory', str(inst)):
                  with open("PIDserial.cnf", 'w') as initfile:
                    initfile.write("COM1")
                    cmprt = "1"
                          
        self.lineEdit = QtGui.QLineEdit(cmprt)
        COMlabel.setBuddy(self.lineEdit)
        self.connectButton = self.createButton("&Connect", self.connect)
        grplabel1 = QtGui.QLabel("Motor Response")
        grplabel2 = QtGui.QLabel("Driver Output")

        self.motorComboBox = QtGui.QComboBox()
        self.motorComboBox.addItem("Motor A")
        motorComboBoxLabel = QtGui.QLabel("Motor to test:        ")
        motorComboBoxLabel.setBuddy(self.motorComboBox)
        self.motorConnected = QtGui.QLabel("Not connected")

        self.Mssg = QtGui.QLabel("")
        self.Mssg2 = QtGui.QLabel("")
        self.invertbtn = self.createButton("&Invert", self.invert)
        self.invertbtn.setEnabled(False)

        stepSizelabel = QtGui.QLabel("Steps size")
        self.stepSize = QtGui.QLineEdit("20")
        stepSizelabel.setBuddy(self.stepSize)
        self.stepButton = self.createButton("&Test", self.step)
        self.stepButton.setEnabled(False)

        Plabel = QtGui.QLabel("Proportional (P)")
        self.Pterm = QtGui.QLineEdit("")
        Ilabel = QtGui.QLabel("Integral (I)")
        self.Iterm = QtGui.QLineEdit("")
        Dlabel = QtGui.QLabel("Differential (D)")
        self.Dterm = QtGui.QLineEdit("")
        self.setPIDButton = self.createButton("&Set PID", self.setPID)
        self.setPIDButton.setEnabled(False)
        self.saveButton = self.createButton("&Save to Cntrl", self.savePID)
        self.saveButton.setEnabled(False)
        self.stopMotorButton = self.createButton("S&top Motor", self.stopMotor)
        self.stopMotorButton.setEnabled(False)

        self.initgraph = True
        self.plt1 = pg.PlotWidget()
        self.plt1.showGrid(x=True,y=True)
        self.plt1.setLabel('left', 'Motor movement', units='Steps')
        self.plt1.setLabel('bottom', 'Time', units='ms')
        
        self.plt2 = pg.PlotWidget()
        self.plt2.showGrid(x=True,y=True)
        self.plt2.setLabel('left', 'Driver Output', units='PWM Value')
        self.plt2.setLabel('bottom', 'Time', units='ms')

        layout = QtGui.QGridLayout()
        layout.addWidget(COMlabel, 0, 0)
        layout.addWidget(self.lineEdit, 0, 1)
        layout.addWidget(self.connectButton, 0, 2)
        
        layout.addWidget(motorComboBoxLabel, 1, 0)
        layout.addWidget(self.motorComboBox, 1, 1)
        layout.addWidget(self.motorConnected, 1, 2)
        
        layout.addWidget(self.Mssg, 2, 0)
        layout.addWidget(self.Mssg2, 2, 1)
        layout.addWidget(self.invertbtn, 2, 2)
        
        layout.addWidget(stepSizelabel, 3, 0)
        layout.addWidget(self.stepSize, 3, 1)
        layout.addWidget(self.stepButton, 3, 2)

        layout.addWidget(Plabel, 4, 0)
        layout.addWidget(self.Pterm, 4, 1)
        layout.addWidget(self.setPIDButton, 4, 2)
                
        layout.addWidget(Ilabel, 5, 0)
        layout.addWidget(self.Iterm, 5, 1)
        layout.addWidget(self.saveButton, 5, 2)
        
        layout.addWidget(Dlabel, 6, 0)
        layout.addWidget(self.Dterm, 6, 1)
        layout.addWidget(self.stopMotorButton, 6, 2)
        
        layout.addWidget(grplabel1, 7, 0)
        layout.addWidget(self.plt1, 8, 0, 3, 5)  
        layout.addWidget(grplabel2, 13, 0)
        layout.addWidget(self.plt2, 15, 0, 3, 5)  

        self.setLayout(layout)
        self.setWindowTitle("PyPIDServoMotorTuner")


    def createButton(self, text, member):
        button = QtGui.QPushButton(text)
        button.clicked.connect(member)
        return button
    
#Connect to COM port
    def connect(self):
      global serport
      global pidStatus
      global b_init
      
      if self.connectstatus:
        serport.close()
        self.connectstatus =False
        self.motorConnected.setText("Disconnected")
        self.Mssg.setText("")
        self.connectButton.setText("Connect")
        self.invertbtn.setEnabled(False)
        self.stepButton.setEnabled(False)
        self.setPIDButton.setEnabled(False)
        self.saveButton.setEnabled(False)
      
      else:
        pidStatus=[-1,-1,-1,False,-1,-1,-1,False,True,False]

        if not self.connectstatus:
          comport = re.search(r'^(\d+)$',self.lineEdit.text())
          try:
            serport= serial.Serial('COM'+str(comport.group(1)), 115200)
          except:
            self.Mssg.setText("<html><head/><body><p><span style=\"color:#FF0000;\">Error Opening serial port</span></p></body></html>")
            return

          with open("PIDserial.cnf", 'w') as initfile:
                    initfile.write("COM"+str(comport.group(1)))

          self.connectstatus =True
          self.motorConnected.setText("Connected")
          self.Mssg.setText("")
          self.connectButton.setText("Disconnect")

        getStatus(1.8)
        if pidStatus[7] and b_init:
              b_init=False 
              self.motorComboBox.addItem("Motor B",self)
              self.motorComboBox.currentIndexChanged.connect(self.onComboChange)
        checkactive(self)

          
#Send serial command to conduct step test and plot graph.
    def step(self):
      global serport
      global c1, c2,c3
      global motorA
      global pidStatus

      self.stepButton.setEnabled(False)
      self.stopMotorButton.setEnabled(True)

      getStatus(1.8)
      if (pidStatus[3] and motorA) or ((pidStatus[7] and not motorA)):
            self.Mssg.setText("Enable PIN Active")
      else:
            self.Mssg.setText("<html><head/><body><p><span style=\"color:#FF0000;\">Enable PIN not Active</span></p></body></html>")
            self.Mssg2.setText("<html><head/><body><p><span style=\"color:#FF0000;\">Motor not moving.</span></p></body></html>")
            self.stepButton.setEnabled(True)
            self.stopMotorButton.setEnabled(False)
            return    

      try:
          stp =int(self.stepSize.text())
          self.Mssg2.setText("<html><head/><body><p><span style=\"color:#FF0000;\"></span></p></body></html>")
      except:
          self.Mssg2.setText("<html><head/><body><p><span style=\"color:#FF0000;\">Step size values is not numeric. Please correct.</span></p></body></html>")
          self.stepButton.setEnabled(True)
          return #Exit step test if step test input value is not numeric

      if self.setPID():
        self.stepButton.setEnabled(True)
        return

      #Send serial command to start step test
      serport.write("z\n")
      if motorA==True:
          serport.write("T"+str(stp)+"\n")
      else:
          serport.write("U"+str(stp)+"\n")

      steptest =[]
      time.sleep(0.5)

      #Read serial output and plot graph
      while serport.inWaiting():
        x=serport.readline()
        #print(x)

        matchObj = re.search(r'M(\d):PRINT STOPPED!!!', x)
        if matchObj:
            self.Mssg2.setText("<html><head/><body><p><span style=\"color:#FF0000;\">Motor "+ matchObj.group(1) + " Stuck or runaway.</span></p></body></html>")
            serport.write("z\n")
            break

        if motorA:
            matchObj = re.search(r'^(\d+):\t(-?\d+),\t(-?\d+).*\| (\d+):\t(-?\d+),\t(-?\d+)', x)
        else:
            matchObj = re.search(r'^(\d+):.*\| (-?\d+),\t(-?\d+).*\| (\d+):.*\| (-?\d+),\t(-?\d+)', x)
        if matchObj:
                steptest.append([int(matchObj.group(1)),stp+int(matchObj.group(2)),int(matchObj.group(3)),int(matchObj.group(4)),stp+int(matchObj.group(5)),int(matchObj.group(6))])

      graph0 =[]
      graph1 =[]

      grpcount=0
      maxout=0
      minv=0
    
      for rsp in steptest:
         graph0.append(rsp[1])
         graph1.append(rsp[2])
         if grpcount >0 and grpcount <5:
           if rsp[2]< 63800:
             maxout=grpcount
         if grpcount >4 and rsp[2] > graph1[maxout]:
             maxout=grpcount

         if rsp[2] < minv:
             minv= rsp[2]
         grpcount+=1

      for rsp in steptest:
         graph0.append(rsp[4])
         if rsp[5] < minv:
             minv= rsp[5]
         graph1.append(rsp[5])

      y = [stp for i in range(grpcount*2)]
      x = range(0,grpcount*2)

      # check whether there is oscillation. Suggest PID values based on Ziegler–Nichols method if oscillating
      osc= maxpt(graph0, y[0])
      #print osc
      if (osc[0]-osc[2])<10 and (osc[1]-osc[3])>-10 and (osc[0]-osc[1])>8:
          if motorA:
              pterm= pidStatus[0]*0.6
              iterm= 1.2*pidStatus[0]*1000/osc[4]
              dterm= 3*pidStatus[0]*osc[4]/40000
          else:
              pterm= pidStatus[4]*0.6
              iterm= 1.2*pidStatus[4]*1000/osc[4]
              dterm= 3*pidStatus[4]*osc[4]/40000
          msg = "<html><head/><body><p><span style=\"color:#FF0000;\">Oscillating. Suggested PID: P=%.3f ,I=%.3f, D=%.3f </span></p></body></html>"%(pterm,iterm,dterm)
          self.Mssg2.setText(msg)          

      # initialize graph values  
      if self.initgraph:
          self.initgraph=False
          self.plt1.addLegend()
          c1 = self.plt1.plot(pen='r', name='Set point')
          c2 = self.plt1.plot(pen= 'y', name='Motor movement')
          c3 = self.plt2.plot(pen= 'g')

      try:
          self.plt2.setYRange(minv, graph1[maxout]+10 )
      except:
          serport.write("z\n")
          self.Mssg2.setText("<html><head/><body><p><span style=\"color:#FF0000;\">Error!!</span></p></body></html>")
          return 

      #plot graph
      self.plt2.setXRange(0, grpcount*2 )

      c1.setData(x, y)
      c2.setData(x, graph0)
      c3.setData(x, graph1)

      serport.write("z\n")
      self.stepButton.setEnabled(True)
      

#Invert Encoder polarity
    def invert(self):

      if motorA:
          serport.write("x1\n")
      else:
          serport.write("x2\n")          

      time.sleep(1)
          
      while serport.inWaiting():
            x=serport.readline()
            #print(x)
            direction = re.search(r'^Encoder_[1-2] direction: ([A-Z_]+.)', x)
            if direction:
                msg = "<html><head/><body><p><span style=\"color:#0000FF;\">" + direction.group(1) + "</span></p></body></html>"
                self.Mssg2.setText(msg)
     

#Send serial command to set the PID values. Not saved in STM32 or Arduino EEPROM. Values will reset to original at reboot
    def setPID(self):
      global serport
      global pidStatus
      global motorA

      try:
          p=float(self.Pterm.text())
          i=float(self.Iterm.text())
          d=float(self.Dterm.text())
      except:
          self.Mssg2.setText("<html><head/><body><p><span style=\"color:#FF0000;\">PID values are not numeric. Please correct.</span></p></body></html>")
          return True

      if motorA:
          serport.write("p"+str(self.Pterm.text())+"\n")
          serport.write("i"+str(self.Iterm.text())+"\n")
          serport.write("d"+str(self.Dterm.text())+"\n")
      else:
          serport.write("r"+str(self.Pterm.text())+"\n")
          serport.write("l"+str(self.Iterm.text())+"\n")
          serport.write("b"+str(self.Dterm.text())+"\n")
          
      getStatus(0.5)          
      if motorA:
        msg = "<html><head/><body><p><span style=\"color:#0000FF;\">Motor A PID set to: P=%.5f ,I=%.5f, D=%.5f, %s</span></p></body></html>"%(pidStatus[0],pidStatus[1],pidStatus[2], pidStatus[8])
      else:
        msg = "<html><head/><body><p><span style=\"color:#0000FF;\">Motor B PID set to: P=%.5f ,I=%.5f, D=%.5f, %s </span></p></body></html>"%(pidStatus[4],pidStatus[5],pidStatus[6], pidStatus[9])          
      self.Mssg2.setText(msg)
      return False

#Save PID values to controller EEPROM
    def savePID(self):
      if not self.setPID():   
        serport.write("w\n")
        time.sleep(0.2)
        while serport.inWaiting():
            x=serport.readline()
            #print(x)
            isstored = re.search(r'^PID values stored to EEPROM', x)
            if isstored:
                self.Mssg2.setText("<html><head/><body><p><span style=\"color:#0000FF;\">PID values stored to EEPROM</span></p></body></html>")
    

#Stop motor by temporarily setting P term to zero and zeroing the setpoint and motor reported position
    def stopMotor(self):
      global serport

      if self.connectstatus:
        serport.write("p0\n")
        serport.write("r0\n")
        time.sleep(2.5)
        serport.write("z\n")
        serport.write("p"+str(pidStatus[0])+"\n")
        serport.write("r"+str(pidStatus[4])+"\n")

#Select Motor A or B in ComboBox
    def onComboChange(self, ix):
        global motorA
        global pidStatus

        if self.connectstatus:
          if ix:
            getStatus(0.2)
            self.Pterm.setText(str(pidStatus[4]))
            self.Iterm.setText(str(pidStatus[5]))
            self.Dterm.setText(str(pidStatus[6]))
            motorA=False
          else:
            getStatus(0.2)
            self.Pterm.setText(str(pidStatus[0]))
            self.Iterm.setText(str(pidStatus[1]))
            self.Dterm.setText(str(pidStatus[2]))
            motorA=True
          checkactive(self)    

#Close serial port when the program is exited
    def closeEvent(self, *args, **kwargs):
        global serport
        try:
            serport.close()
        except:
            return

#Send an 's' command to serial port to get the PID and Enable PIN status
def getStatus(delay): 
          global serport
          global pidStatus
          
          serport.write("s\n")
          time.sleep(delay)
          
          while serport.inWaiting():
            x=serport.readline()
            #print(x)

            pidObj1 = re.search(r'^kp1: (\d+.\d+)\t\tki1: (\d+.\d+)\tkd1: (\d+.\d+)', x)
            if pidObj1:
              pidStatus[0]= float(pidObj1.group(1))
              pidStatus[1]= float(pidObj1.group(2))
              pidStatus[2]= float(pidObj1.group(3))

            pidObj1 = re.search(r'^kp2: (\d+.\d+)\t\tki2: (\d+.\d+)\tkd2: (\d+.\d+)', x)
            if pidObj1:
              pidStatus[4]= float(pidObj1.group(1))
              pidStatus[5]= float(pidObj1.group(2))
              pidStatus[6]= float(pidObj1.group(3))
            enStatus = re.search(r'^Enable', x, re.IGNORECASE)
            if enStatus:
                if re.search(r'_1', x): 
                  if re.search(r'active', x, re.IGNORECASE):             
                            pidStatus[3]=True
                  else:             
                            pidStatus[3]=False
                elif re.search(r'_2', x):
                  if re.search(r'active', x, re.IGNORECASE):             
                            pidStatus[7]=True
                  else:             
                            pidStatus[7]=False


            pidObj1 = re.search(r'^Encoder_1 direction: ([A-Z_]+.)', x)
            if pidObj1:
                pidStatus[8] = pidObj1.group(1)

            pidObj1 = re.search(r'^Encoder_2 direction: ([A-Z_]+.)', x)
            if pidObj1:
                pidStatus[9] = pidObj1.group(1)


#Check Enable PIN status and activate buttons accordingly
def checkactive(self):
        if motorA and pidStatus[3]:
            self.Mssg.setText("Enable_1 PIN Active")
            self.stepButton.setEnabled(True)
            self.Pterm.setText(str(pidStatus[0]))
            self.Iterm.setText(str(pidStatus[1]))
            self.Dterm.setText(str(pidStatus[2]))
            self.setPIDButton.setEnabled(True)
            self.saveButton.setEnabled(True)
            self.invertbtn.setEnabled(True)
        elif pidStatus[7] and not motorA:
            self.Mssg.setText("Enable_2 PIN Active")
            self.stepButton.setEnabled(True)
            self.Pterm.setText(str(pidStatus[4]))
            self.Iterm.setText(str(pidStatus[5]))
            self.Dterm.setText(str(pidStatus[6]))
            self.setPIDButton.setEnabled(True)
            self.saveButton.setEnabled(True)
            self.invertbtn.setEnabled(True)
        else:
            self.Mssg.setText("<html><head/><body><p><span style=\"color:#FF0000;\">Enable PIN not Active</span></p></body></html>")
            self.Mssg2.setText("")
            self.stepButton.setEnabled(False)
            #self.connectButton.setEnabled(True)
            self.setPIDButton.setEnabled(False)
            self.saveButton.setEnabled(False)
            self.stopMotorButton.setEnabled(False)
            self.invertbtn.setEnabled(False)

#Search for period, max and min point of motor response
def maxpt(rsp, setpt):
  max=[0]
   
  arry= findmax(0,-10000,rsp, setpt)
  max[0]= arry[0]
  time1=arry[2]
    
  arry= findmin(arry[1],max[0],rsp)
  max.append(arry[0])
   
  arry= findmax(arry[1],max[1],rsp, setpt)
  max.append(arry[0])
  time2=arry[2]

  arry= findmin(arry[1],max[2],rsp)
  max.append(arry[0])
  max.append(time2-time1)
  return max

#Find min point of motor response
def findmin(startpt, curmin, data):
  minout = [0,0]
  
  z = startpt
  min1 = curmin
  for i in range(startpt, len(data)):
    if data[i] <= min1:
      min1 = data[i]
      z+=1
    else:
      minout[0]= min1
      break
  minout[1]= z
  return minout

#Find period and max point of motor response
def findmax(startpt, curmax, data, crosspt):
  maxout = [0,0,0.0]
  cross = True

  y = startpt
  max1 = curmax
  arrysize = len(data)
  for i in range(startpt, arrysize):
    if data[i] >= max1:
      max1 = data[i]
      y+=1
      if data[i] >= crosspt and cross:
        if data[i] > crosspt:
          time=float(i-1)+(float(crosspt-data[i-1])/float(data[i]-data[i-1]))
        else:
          time=float(i)
        maxout[2]= time
        cross = False
    else:
      maxout[0]= max1
      break
  maxout[1]= y
  return maxout

#Main Loop
if __name__ == '__main__':
    import sys

    global motorA
    global b_init
    
    motorA=True
    b_init=True

    app = QtGui.QApplication(sys.argv)
    app.setWindowIcon(QtGui.QIcon('motor.png'))

    window = Window()
    window.show()

    sys.exit(app.exec_())
    
