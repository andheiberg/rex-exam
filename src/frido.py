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

    def canGo(self, cm):
        # The thinking is that it will go mostly straight.
        # But we should account for some potential drift.
        # Plus the robot does need some clearance (5 cm in this case)
        sideThreshold = (5 + cm * 0.2)

        if self._frontDistToCM(self.robot.read_front_ir_sensor()) < (cm * 2):
            return False
        elif self._rightDistToCM(self.robot.read_right_ir_sensor()) < sideThreshold:
            return False
        elif self._leftDistToCM(self.robot.read_left_ir_sensor()) < sideThreshold:
            return False

        return True

    def _frontDistToCM(self, distance):
        return int(round((79.943427 * (0.9953660709 ** distance))))

    def _rightDistToCM(self, distance):
        return int(round((85.60176169 * (0.995149692 ** distance))))

    def _leftDistToCM(self, distance):
        return int(round((90.5118426 * (0.9951333544 ** distance))))
