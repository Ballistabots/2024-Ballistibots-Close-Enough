import math
import numpy
import wpilib
import robot
class Shooting:
   def __init__(self):
      self.velocity = 600
      self.robot = robot.MyRobot()

      pass

   def getInitalLaunchAngle(self, distance: float, velocity: float) -> float:

      LA = 1 / 2 * numpy.arcsin(((9.8 * distance) / (math.sqrt(velocity))))  # formula to return the inital launch angle
      return LA

   def CheckIfLegal(self, distance: float, velocity: float) -> bool:

      LA = 1 / 2 * numpy.arcsin(((9.8 * distance) / (math.sqrt(velocity))))  # formula to return the inital launch angle

      if LA < 40:  # if the angle is less than the minimum or more than the maximum the functions returns false
         return False
      else:
         return True

   def CalculateDistanceR(self, velocity: float,
                          launchAngle: float) -> float:  # calculates the distance traveled and returns it as a float

      R = ((math.sqrt(velocity) * math.sin(2 * launchAngle)) / 9.8)  # 9.8 is gravity
      return R

   def CalculatePeakHeight(self, velocity: float, LaunchAngle: float) -> float:
      H = ((math.sqrt(velocity) * math.sin(2 * LaunchAngle)) / (2 * 9.8))  # 9.8 is gravity
      return H

   def Fire(self, distance: float, velocity: float):
      legal = self.CheckIfLegal(distance, velocity)
      if legal:
         print("UPDATING SERVO")
         ServoPos = self.getInitalLaunchAngle(distance, velocity)
         self.UpdateServo(ServoPos)
         distanceTele = self.CalculateDistanceR(velocity, ServoPos)
         print(f"Distance: {distanceTele}")
         print(f"Peak height at {distanceTele / 2}:  {self.CalculatePeakHeight(ServoPos, velocity)}")
         print("Ready for firing starting firing process")
         print(f"Flywheel velocity is at {self.velocity}")
         return True
      else:
         print("ILLEGAL DISTANCE NOT SHOOTING CHANGE DISTANCE")
         return False

   def scale_number(self, unscaled, to_min, to_max, from_min, from_max):
      return (to_max - to_min) * (unscaled - from_min) / (from_max - from_min) + to_min

   def AngleToServoPosition(self, angle: float) -> float:  # converts an angle to a servo position
      MIN_ANGLE = 220  # 1.0 for servo pos
      MAX_ANGLE = 260  # -1.0 for servo pos
      add = 0
      return self.scale_number(angle, -1, 1, MIN_ANGLE, MAX_ANGLE) + add  # for tuning

   def ServoPosToAngle(self, Servo) -> float:
      MIN_ANGLE = 40  # 1.0 for servo pos
      MAX_ANGLE = 80  # -1.0 for servo pos
      return self.scale_number(Servo, -MAX_ANGLE, MIN_ANGLE, -1, 1)

   def UpdateServo(self, NewServoPos) -> None:
      # updates servo pos to match the angle given by the LA servo pos function
      NewPos = self.AngleToServoPosition(NewServoPos)
      NewPos -= 0.01
      NewPos = float(NewPos)
      if NewPos > 1 or NewPos < -1:
         print(f"range no good {NewPos}")
      else:
         print(f"range no good {NewPos}")


   def FlyWheelPrep(self):
      robot.MyRobot.setFlyWheel(self.robot, 600)
