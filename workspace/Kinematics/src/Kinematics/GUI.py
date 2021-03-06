# -*- coding: utf-8 -*-

import sys
import socket
import Kinematics as kin
from PyQt4 import QtGui, QtCore
from sympy.mpmath import *
import math

SERVER_IP = '127.0.0.1'
SERVER_PORT = 54321

class GUI(QtGui.QWidget):
    def __init__(self):     
        super(GUI, self).__init__()
        
        # initialize GUI
        self.initUI()
        
        # get the initial axis values as well as the initial position and orientation of the robot
        self.dataTransfer('GET') # using the prefix get for server-sided parsing
                  
    
    def initUI(self):         
        grid = QtGui.QGridLayout()
        # create the group of widgets at the top left position
        grid.addWidget(self.createAxesGroup(), 0, 0)
        # create the group of widgets at the top right position
        grid.addWidget(self.createPTPGroup(), 0, 1)
        # create the group of widgets at the bottom left position
        grid.addWidget(self.createCartPosGroup(), 1, 0)
        # create the group of widgets at the bottom right position
        grid.addWidget(self.createCartPTPGroup(), 1, 1)
        
        # set window properties
        self.setLayout(grid)
        self.resize(400, 300)
        self.center()
        self.setWindowTitle('Simulation')
        self.show()
        
        
    # create the group of widgets at the top left position
    def createAxesGroup(self):
        # initialize the group
        group_box = QtGui.QGroupBox('Axes')
        # use grid layout
        grid_axes = QtGui.QGridLayout()
        
        # set up labels
        label_axes_a1 = QtGui.QLabel(u'\u03B11', self)    
        label_axes_a2 = QtGui.QLabel(u'\u03B12', self)
        label_axes_a3 = QtGui.QLabel(u'\u03B13', self)    
        label_axes_a4 = QtGui.QLabel(u'\u03B14', self)    
        label_axes_a5 = QtGui.QLabel(u'\u03B15', self)    
        label_axes_a6 = QtGui.QLabel(u'\u03B16', self)
        # set up line edits
        self.lineedit_axes_a1 = QtGui.QLineEdit('0', self)
        self.lineedit_axes_a2 = QtGui.QLineEdit('0', self)
        self.lineedit_axes_a3 = QtGui.QLineEdit('0', self)   
        self.lineedit_axes_a4 = QtGui.QLineEdit('0', self)    
        self.lineedit_axes_a5 = QtGui.QLineEdit('0', self)    
        self.lineedit_axes_a6 = QtGui.QLineEdit('0', self)
        # disable line edits
        self.lineedit_axes_a1.setEnabled(0)
        self.lineedit_axes_a2.setEnabled(0)
        self.lineedit_axes_a3.setEnabled(0)
        self.lineedit_axes_a4.setEnabled(0)
        self.lineedit_axes_a5.setEnabled(0)
        self.lineedit_axes_a6.setEnabled(0)
        
        # add the widgets to the grid layout
        grid_axes.addWidget(label_axes_a1, 0, 0)
        grid_axes.addWidget(self.lineedit_axes_a1, 0, 1)
        grid_axes.addWidget(label_axes_a2, 1, 0)
        grid_axes.addWidget(self.lineedit_axes_a2, 1, 1)
        grid_axes.addWidget(label_axes_a3, 2, 0)
        grid_axes.addWidget(self.lineedit_axes_a3, 2, 1)
        grid_axes.addWidget(label_axes_a4, 3, 0)
        grid_axes.addWidget(self.lineedit_axes_a4, 3, 1)
        grid_axes.addWidget(label_axes_a5, 4, 0)
        grid_axes.addWidget(self.lineedit_axes_a5, 4, 1)
        grid_axes.addWidget(label_axes_a6, 5, 0)
        grid_axes.addWidget(self.lineedit_axes_a6, 5, 1)
        
        # set the grid layout for the group
        group_box.setLayout(grid_axes)
        
        return group_box
    
    
    # create the group of widgets at the bottom left position
    def createCartPosGroup(self):
        # initialize the group
        group_box = QtGui.QGroupBox('Cartesian Position')
        # use grid layout
        grid_cartpos = QtGui.QGridLayout()
        # set up labels
        label_cartpos_x = QtGui.QLabel('X', self)
        label_cartpos_y = QtGui.QLabel('Y', self)
        label_cartpos_z = QtGui.QLabel('Z', self)
        label_cartpos_a = QtGui.QLabel(u'\u03B1', self)
        label_cartpos_b = QtGui.QLabel(u'\u03B2', self)
        label_cartpos_c = QtGui.QLabel(u'\u03B3', self)
        # set up line edits 
        self.lineedit_cartpos_x = QtGui.QLineEdit('0', self)
        self.lineedit_cartpos_y = QtGui.QLineEdit('0', self)  
        self.lineedit_cartpos_z = QtGui.QLineEdit('0', self)
        self.lineedit_cartpos_a = QtGui.QLineEdit('0', self) 
        self.lineedit_cartpos_b = QtGui.QLineEdit('0', self) 
        self.lineedit_cartpos_c = QtGui.QLineEdit('0', self)
        # disable line edits
        self.lineedit_cartpos_x.setEnabled(0)
        self.lineedit_cartpos_y.setEnabled(0)
        self.lineedit_cartpos_z.setEnabled(0)
        self.lineedit_cartpos_a.setEnabled(0)
        self.lineedit_cartpos_b.setEnabled(0)
        self.lineedit_cartpos_c.setEnabled(0)
        
        # add the widgets to the grid layout
        grid_cartpos.addWidget(label_cartpos_x, 0, 0)
        grid_cartpos.addWidget(self.lineedit_cartpos_x, 0, 1)
        grid_cartpos.addWidget(label_cartpos_y, 1, 0)
        grid_cartpos.addWidget(self.lineedit_cartpos_y, 1, 1)
        grid_cartpos.addWidget(label_cartpos_z, 2, 0)
        grid_cartpos.addWidget(self.lineedit_cartpos_z, 2, 1)
        grid_cartpos.addWidget(label_cartpos_a, 3, 0)
        grid_cartpos.addWidget(self.lineedit_cartpos_a, 3, 1)
        grid_cartpos.addWidget(label_cartpos_b, 4, 0)
        grid_cartpos.addWidget(self.lineedit_cartpos_b, 4, 1)
        grid_cartpos.addWidget(label_cartpos_c, 5, 0)
        grid_cartpos.addWidget(self.lineedit_cartpos_c, 5, 1)
        
        # set the grid layout for the group
        group_box.setLayout(grid_cartpos)
        
        return group_box
        
    
    # create the group of widgets at the top right position
    def createPTPGroup(self):
        # initialize the group
        group_box = QtGui.QGroupBox('PTP')
        # use grid layout
        grid_ptp = QtGui.QGridLayout()
        
        # set up labels
        label_ptp_a1 = QtGui.QLabel(u'\u03B11', self)
        label_ptp_a2 = QtGui.QLabel(u'\u03B12', self)
        label_ptp_a3 = QtGui.QLabel(u'\u03B13', self)
        label_ptp_a4 = QtGui.QLabel(u'\u03B14', self)
        label_ptp_a5 = QtGui.QLabel(u'\u03B15', self)
        label_ptp_a6 = QtGui.QLabel(u'\u03B16', self)
        # set up line edits 
        self.lineedit_ptp_a1 = QtGui.QLineEdit('0', self)  
        self.lineedit_ptp_a2 = QtGui.QLineEdit('0', self)  
        self.lineedit_ptp_a3 = QtGui.QLineEdit('0', self) 
        self.lineedit_ptp_a4 = QtGui.QLineEdit('0', self) 
        self.lineedit_ptp_a5 = QtGui.QLineEdit('0', self) 
        self.lineedit_ptp_a6 = QtGui.QLineEdit('0', self)   
        # set up radio buttons
        self.radio_asynch = QtGui.QRadioButton('asynchronous', self)
        self.radio_asynch.setChecked(True)
        self.radio_synch = QtGui.QRadioButton('synchronous', self)
        self.radio_fullsynch = QtGui.QRadioButton('full synchronous', self)
        
        # set up check boxes
        self.check_lin = QtGui.QCheckBox('lin', self)
        self.check_rrt = QtGui.QCheckBox('rrt', self)
        
        # set up button for the calculation
        self.button_move = QtGui.QPushButton('Move', self)
        
        # trigger function on clicked
        QtCore.QObject.connect(self.button_move, QtCore.SIGNAL("clicked()"), self.buttonMoveClicked)
        
        # add the widgets to the grid layout
        grid_ptp.addWidget(label_ptp_a1, 0, 0)
        grid_ptp.addWidget(self.lineedit_ptp_a1, 0, 1)
        grid_ptp.addWidget(self.radio_asynch, 0, 2)
        grid_ptp.addWidget(label_ptp_a2, 1, 0)
        grid_ptp.addWidget(self.lineedit_ptp_a2, 1, 1)
        grid_ptp.addWidget(self.radio_synch, 1, 2)
        grid_ptp.addWidget(label_ptp_a3, 2, 0)
        grid_ptp.addWidget(self.lineedit_ptp_a3, 2, 1)
        grid_ptp.addWidget(self.radio_fullsynch, 2, 2)
        grid_ptp.addWidget(label_ptp_a4, 3, 0)
        grid_ptp.addWidget(self.lineedit_ptp_a4, 3, 1)
        grid_ptp.addWidget(self.check_lin, 3, 2, 2, 1)
        grid_ptp.addWidget(self.check_rrt, 3, 2, 3, 1)
        grid_ptp.addWidget(label_ptp_a5, 4, 0)
        grid_ptp.addWidget(self.lineedit_ptp_a5, 4, 1)
        grid_ptp.addWidget(label_ptp_a6, 5, 0)
        grid_ptp.addWidget(self.lineedit_ptp_a6, 5, 1)
        grid_ptp.addWidget(self.button_move, 5, 2)
        
        # set the grid layout for the group
        group_box.setLayout(grid_ptp)
        
        return group_box
        
        
    # create the group of widgets at the bottom right position
    def createCartPTPGroup(self):
        # initialize the group
        group_box = QtGui.QGroupBox('Cartesian PTP')
        # use grid layout
        grid_cartptp = QtGui.QGridLayout()
        
        # set up labels
        label_cartptp_x = QtGui.QLabel('X', self)
        label_cartptp_y = QtGui.QLabel('Y', self)
        label_cartptp_z = QtGui.QLabel('Z', self)
        label_cartptp_a = QtGui.QLabel(u'\u03B1', self)
        label_cartptp_b = QtGui.QLabel(u'\u03B2', self)
        label_cartptp_c = QtGui.QLabel(u'\u03B3', self)  
        # set up line edits 
        self.lineedit_cartptp_x = QtGui.QLineEdit('0', self)
        self.lineedit_cartptp_y = QtGui.QLineEdit('0', self)  
        self.lineedit_cartptp_z = QtGui.QLineEdit('0', self)
        self.lineedit_cartptp_a = QtGui.QLineEdit('0', self) 
        self.lineedit_cartptp_b = QtGui.QLineEdit('0', self) 
        self.lineedit_cartptp_c = QtGui.QLineEdit('0', self)
        # set up the line edit box for the multiple kinematic solutions

        self.radio_inverse = range(8)
        for i in self.radio_inverse:
            self.radio_inverse[i] = QtGui.QRadioButton('inverse ' + str(i), self)
            QtCore.QObject.connect(self.radio_inverse[i], QtCore.SIGNAL("clicked()"), self.radioClicked)
            self.radio_inverse[i].setVisible(False)
        
        
        # set up button for the calculation
        self.button_calculate = QtGui.QPushButton('Calculate IK', self)
        # trigger function on clicked
        QtCore.QObject.connect(self.button_calculate, QtCore.SIGNAL("clicked()"), self.buttonCalculateClicked)
        
        # add the widgets to the grid layout
        grid_cartptp.addWidget(label_cartptp_x, 0, 0)
        grid_cartptp.addWidget(self.lineedit_cartptp_x, 0, 1)
        grid_cartptp.addWidget(label_cartptp_y, 1, 0)
        grid_cartptp.addWidget(self.lineedit_cartptp_y, 1, 1)
        grid_cartptp.addWidget(label_cartptp_z, 2, 0)
        grid_cartptp.addWidget(self.lineedit_cartptp_z, 2, 1)
        grid_cartptp.addWidget(label_cartptp_a, 3, 0)
        grid_cartptp.addWidget(self.lineedit_cartptp_a, 3, 1)
        grid_cartptp.addWidget(label_cartptp_b, 4, 0)
        grid_cartptp.addWidget(self.lineedit_cartptp_b, 4, 1)
        grid_cartptp.addWidget(label_cartptp_c, 5, 0)
        grid_cartptp.addWidget(self.lineedit_cartptp_c, 5, 1)
        grid_cartptp.addWidget(self.button_calculate, 5, 2)
        
        grid_cartptp.addWidget(self.radio_inverse[0], 0, 2, 1, 1)
        grid_cartptp.addWidget(self.radio_inverse[1], 0, 2, 2, 1)
        grid_cartptp.addWidget(self.radio_inverse[2], 0, 2, 3, 1)
        grid_cartptp.addWidget(self.radio_inverse[3], 0, 2, 4, 1)
        grid_cartptp.addWidget(self.radio_inverse[4], 2, 2, 1, 1)
        grid_cartptp.addWidget(self.radio_inverse[5], 2, 2, 2, 1)
        grid_cartptp.addWidget(self.radio_inverse[6], 2, 2, 3, 1)
        grid_cartptp.addWidget(self.radio_inverse[7], 2, 2, 4, 1)
        
        # set the grid layout for the group
        group_box.setLayout(grid_cartptp)
        
        return group_box


    # setting window properties
    def center(self):
        qr = self.frameGeometry()
        cp = QtGui.QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
        
        
    # function is called when the move button is clicked
    def buttonMoveClicked(self):
        # prefix for parsing
        prefix = "MOV#"   
        # get values and convert QString to string
        msg = str( str( mp.radians( float( self.lineedit_ptp_a1.text()))) + ";" + 
                   str( mp.radians( float( self.lineedit_ptp_a2.text()))) + ";" +
                   str( mp.radians( float( self.lineedit_ptp_a3.text()))) + ";" + 
                   str( mp.radians( float( self.lineedit_ptp_a4.text()))) + ";" + 
                   str( mp.radians( float( self.lineedit_ptp_a5.text()))) + ";" + 
                   str( mp.radians( float( self.lineedit_ptp_a6.text())))) 
        # get motion type
        if self.radio_asynch.isChecked() is True:
            motion_type = "#A"
        elif self.radio_synch.isChecked() is True:
            motion_type = "#S"
        elif self.radio_fullsynch.isChecked() is True:
            motion_type = "#F"
            
        motion_type += ";L" if self.check_lin.isChecked() else "; "
        motion_type += ";R" if self.check_rrt.isChecked() else "; "    

        # send data
        self.dataTransfer(prefix+msg+motion_type)

    # function is called when the clear button is clicked
    def radioClicked(self):
        for i in xrange(0, len(self.radio_inverse)):
            if self.radio_inverse[i].isChecked():
                self.lineedit_ptp_a1.setText(str(self.inverse_configurations[i][0]))
                self.lineedit_ptp_a2.setText(str(self.inverse_configurations[i][1]))
                self.lineedit_ptp_a3.setText(str(self.inverse_configurations[i][2]))
                self.lineedit_ptp_a4.setText(str(self.inverse_configurations[i][3]))
                self.lineedit_ptp_a5.setText(str(self.inverse_configurations[i][4]))
                self.lineedit_ptp_a6.setText(str(self.inverse_configurations[i][5]))
                break
            
    # function is called when the calculate IK button is clicked
    def buttonCalculateClicked(self):
        # prefix for parsing
        prefix = "CAL#"   
        # get values and convert QString to string
        values = str( self.lineedit_cartptp_x.text() + ";" + 
                      self.lineedit_cartptp_y.text() + ";" + 
                      self.lineedit_cartptp_z.text() + ";" + 
                      str( mp.radians( float( self.lineedit_cartptp_a.text()))) + ";" + 
                      str( mp.radians( float( self.lineedit_cartptp_b.text()))) + ";" + 
                      str( mp.radians( float( self.lineedit_cartptp_c.text())))) 
        # send data
        self.dataTransfer(prefix+values)
        
    
    # handles the data transfer between the GUI (client) and openrave (server)
    def dataTransfer(self, msg):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.sendto(msg,(SERVER_IP,SERVER_PORT))
        recv_data, addr = s.recvfrom(2048)
        
        self.handleData(recv_data)
        
        s.close()
      
    
    # handles the data received from openrave  
    def handleData(self, data):
        data_arr = data.split("#")
        
        if data_arr[0] == "VAL":
            self.updateValues(data_arr)
        elif data_arr[0] == "INK":
            self.updateINK(data_arr)
            
            
            
    # update UI with received position, orientation and axis values
    def updateValues(self, data):
        # update axis values
        axis_arr = data[1].split(";")
        
        self.lineedit_axes_a1.setText( str( round( mp.degrees( float( axis_arr[0])), 6)))
        self.lineedit_axes_a2.setText( str( round( mp.degrees( float( axis_arr[1])), 6)))
        self.lineedit_axes_a3.setText( str( round( mp.degrees( float( axis_arr[2])), 6)))
        self.lineedit_axes_a4.setText( str( round( mp.degrees( float( axis_arr[3])), 6)))
        self.lineedit_axes_a5.setText( str( round( mp.degrees( float( axis_arr[4])), 6)))
        self.lineedit_axes_a6.setText( str( round( mp.degrees( float( axis_arr[5])), 6)))
        
        # update position and orientaion values
        cart_arr = data[2].split(";")
        self.lineedit_cartpos_x.setText( str( round( float( cart_arr[0]), 6)))
        self.lineedit_cartpos_y.setText( str( round( float( cart_arr[1]), 6)))
        self.lineedit_cartpos_z.setText( str( round( float( cart_arr[2]), 6)))
        self.lineedit_cartpos_a.setText( str( round( mp.degrees( float( cart_arr[3])), 6)))
        self.lineedit_cartpos_b.setText( str( round( mp.degrees( float( cart_arr[4])), 6)))
        self.lineedit_cartpos_c.setText( str( round( mp.degrees( float( cart_arr[5])), 6)))
        
    
    # update UI with multiple inverse kinematic solutions
    def updateINK(self, data):
        
        values = data[1].split(';')
        
        self.inverse_configurations = []
        for i in xrange(0, len(self.radio_inverse)):
            self.radio_inverse[i].setVisible(False)

        if 0 < len(values[-1]): 
            for i in xrange(0, min(len(values), len(self.radio_inverse))):              
                config = []
                for angle in values[i].split(' '):
                    config.append(round(mp.degrees(float(angle)),6))
                self.inverse_configurations.append(config)
                self.radio_inverse[i].setVisible(True)    
    
def main():
    app = QtGui.QApplication(sys.argv)
    
    gui = GUI()
    
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    main()