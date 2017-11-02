# Frindo Robot Controller

#import time
from time import sleep
import serial



class Robot(object):
    """Defines the Frindo robot API""" 
    def __init__(self):
        self.port = '/dev/ttyACM0'

        self.serialRead = serial.Serial(self.port,9600, timeout=1)

        # Wait if serial port is not open yet
        while not self.serialRead.isOpen():
            sleep(1)

        print("Waiting for serial port connection ...")
        sleep(2)

        print("Running ...")
        
        
    def send_command(self, cmd):
        """Sends a command to the Arduino robot controller"""
        self.serialRead.write(cmd.encode('ascii'))
        str_val=self.serialRead.readline()
        return str_val
        
    def go(self):
        """Send a go command for continouos forward driving"""
        cmd='g\n'
        return self.send_command(cmd)
        
    def backward(self):
        """Send a backward command for continouos reverse driving"""
        cmd='v\n'
        return self.send_command(cmd)
        
    def stop(self):
        """Send a stop command to stop motors"""
        cmd='s\n'
        return self.send_command(cmd)

    def left(self):
        """Send a rotate left command for continouos rotating left"""
        cmd='n\n'
        return self.send_command(cmd)

    def right(self):
        """Send a rotate right command for continouos rotating right"""
        cmd='m\n'
        return self.send_command(cmd)
        
    def step_forward(self):
        """Send a step forward command for driving forward for a predefined amount of time"""
        cmd='f\n'
        return self.send_command(cmd)

    def step_backward(self):
        """Send a step backward command for driving backward for a predefined amount of time"""
        cmd='b\n'
        return self.send_command(cmd)

    def step_rotate_left(self):
        """Send a step rotate left command for rotating left for a predefined amount of time"""
        cmd='l\n'
        return self.send_command(cmd)
        
    def step_rotate_right(self):
        """Send a step rotate right command for rotating right for a predefined amount of time"""
        cmd='r\n'
        return self.send_command(cmd)
        
    def go_diff(self, speedLeft, speedRight, dirLeft, dirRight):
        """Start left motor with speed speedLeft (in [0;255]) and direction dirLeft (0=reverse, 1=forward)
           and right motor with speed speedRight (in [0;255]) and direction dirRight (0=reverse, 1=forward)"""
        cmd = 'd' + str(speedLeft) + ',' + str(speedRight) + ',' + str(dirLeft) + ',' + str(dirRight) + '\n'
        return self.send_command(cmd)
    
    
    def read_ir_sensor(self, sensorid):
        """Send a read sensor command with sensorid and return sensor value. Will return -1, if error occurs."""
        cmd=str(sensorid) + '\n'
        str_val=self.send_command(cmd)
        if len(str_val) > 0:
            return int(str_val)
        else:
            return -1
            
    def read_front_ir_sensor(self):
        """Read the front IR sensor and return the measured voltage in the range [0;1023] (from 0 to 5 volts)"""
        return self.read_ir_sensor(0)
        
    def read_right_ir_sensor(self):
        """Read the right IR sensor and return the measured voltage in the range [0;1023] (from 0 to 5 volts)"""
        return self.read_ir_sensor(1)
        
    def read_left_ir_sensor(self):
        """Read the left IR sensor and return the measured voltage in the range [0;1023] (from 0 to 5 volts)"""
        return self.read_ir_sensor(2)
        
    def set_speed(self, speed):
        """Speed must be a value in the range [0; 255]"""
        cmd='z' + str(speed) + '\n'
        return self.send_command(cmd)
        
    def set_turnspeed(self, speed):
        """Speed must be a value in the range [0; 255]"""
        cmd='x' + str(speed) + '\n'
        return self.send_command(cmd)

    def set_step_time(self, steptime):
        """steptime is the amount of miliseconds used in the step_forward and step_backwards commands."""
        cmd='t' + str(steptime) + '\n'
        return self.send_command(cmd)
        
    def set_turn_time(self, turntime):
        """turntime is the amount of miliseconds used in the step_rotate_left and step_rotate_right commands."""
        cmd='y' + str(turntime) + '\n'
        return self.send_command(cmd)
