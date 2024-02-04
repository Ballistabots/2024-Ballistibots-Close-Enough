import wpilib
import shooting
import robot as robot

class Constnants:

   def __init__(self):
      self.driver2 = wpilib.Joystick(1)
      print("init of constants")
      self.Gravity = 9.8 #m/s
      self.MaxVeloctiy = 620 #max volatage per second m/s
      self.InitialHeight = 0.01838706
      self.HoodAngle = robot.MyRobot.getHoodPosition()





