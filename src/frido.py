# Our frido extension

import robot as robotics

class Robot(object):
    """Defines the Frindo robot API""" 
    def __init__(self):
        # Initialize the robot
        self.robot = robotics.Robot()

        # Set speed
        self.robot.set_speed(70)
        self.robot.set_turnspeed(20)
        self.robot.set_step_time(1200)
        self.robot.set_turn_time(1000)

    # Sleep(4) for 100cm
    def go(self, cm):
        direction = 1

        if cm < 0:
            direction = 0

        leftSpeed = 129
        rightSpeed = int(round(1.305 * leftSpeed))
        amount = round(cm/25)

        self.robot.go_diff(180, 235, direction, direction)
        sleep(0.01)
        self.robot.go_diff(leftSpeed, rightSpeed, direction, direction)   
        sleep(amount)

        self.stop()

    def turn(self, degrees): 
        if degrees < 0:
            turn = round((1.8/360)*(-degrees))
            self.robot.set_turnspeed(170)
            self.robot.left()
            sleep(0.01)
            self.robot.set_turnspeed(90)
            self.robot.left()
            sleep(turn)
        else:
            turn = round((1.8/360)*degrees)
            self.robot.set_turnspeed(170)
            self.robot.right()
            sleep(0.01)
            self.robot.set_turnspeed(90)
            self.robot.right()
            sleep(turn)

        self.robot.stop()
