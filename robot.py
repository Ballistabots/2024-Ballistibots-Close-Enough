import math

import phoenix5
import rev as Rev
import wpilib
import wpilib.drive
import wpimath
from phoenix5 import ControlMode
from wpilib import event
from wpimath import filter
import navx

from networktables import NetworkTables

import physics_d.shooting as shooter


class MyRobot(wpilib.TimedRobot):

   def getFlyWheel(self):
      return ("outake1")

   def setFlyWheel(self, velocity):
      self.outake.set(ControlMode.Velocity, velocity)

   def robotPeriodic(self):

      pass

   def disabledInit(self):
      self.logger.info("Entered disabled mode")

      self.timer.reset()
      self.timer.start()

   def robotInit(self) -> None:
      self.EventLoop = event.EventLoop()
      self.shooter = shooter.Shooting()
      # Drivetrain motors
      self.frontLeft = Rev.CANSparkMax(7, Rev.CANSparkMax.MotorType.kBrushless)
      self.frontRight = Rev.CANSparkMax(5, Rev.CANSparkMax.MotorType.kBrushless)
      self.backLeft = Rev.CANSparkMax(1, Rev.CANSparkMax.MotorType.kBrushless)
      self.backRight = Rev.CANSparkMax(6, Rev.CANSparkMax.MotorType.kBrushless)

      self.sd = NetworkTables.getTable("SmartDashboard")

      self.timer = wpilib.Timer()

      self.navx = navx.AHRS.create_i2c()

      # self.leftHood = wpilib.Servo(1)
      # self.rightHood = wpilib.Servo(0)

      # scoring mech
      self.outake = phoenix5.TalonFX(9, "rio")

      self.intake = Rev.CANSparkMax(2, Rev.CANSparkMax.MotorType.kBrushless)
      self.belt = Rev.CANSparkMax(3, Rev.CANSparkMax.MotorType.kBrushless)

      self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
      self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
      self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

      self.maxVel = 600

      self.frontRight.setInverted(True)
      self.backRight.setInverted(True)

      self.outake.setInverted(True)

      self.leftHood = wpilib.Servo(1)
      self.rightHood = wpilib.Servo(0)

      self.robotDrive = wpilib.drive.MecanumDrive(self.frontLeft, self.backLeft, self.frontRight, self.backRight)

      self.driver = wpilib.Joystick(0)
      self.driver2 = wpilib.Joystick(1)
      self.slow = -(self.driver.getThrottle())

      self.ShootingReady = False

   def teleopPeriodic(self):
      self.slow = math.sin(self.driver.getThrottle()) + math.sin(self.driver.getThrottle() + -0.2)


      self.robotDrive.driveCartesian(
         self.driver.getY()  * self.slow,
         -self.driver.getX() * self.slow,
         -self.driver.getZ() * self.slow,
      )



      if self.driver2.getPOV(0):
         self.intake.setVoltage(0)
         self.belt.setVoltage(0)
      else:
         self.intake.setVoltage(-50)
         self.belt.setVoltage(-100)

         # turns the motors for intaking off when there is no action needed
      if self.driver2.getRawButtonPressed(2):
         angle = self.shooter.ServoPosToAngle(self.driver.getThrottle())
         distance = self.shooter.CalculateDistanceR(600, angle)
         peakHeight = self.shooter.CalculatePeakHeight(600,angle)
         print(f"Distance: {distance}")
         print(f"Peak height: {peakHeight}")

      self.leftHood.setPosition(-self.driver2.getThrottle())
      self.rightHood.setPosition(self.driver2.getThrottle())

      if self.driver2.getTrigger():
         self.outake.set(ControlMode.PercentOutput, 0.8)
      else:
         self.outake.set(ControlMode.PercentOutput, 0)

      if self.timer.hasPeriodPassed(0.5):
         self.sd.putNumber("Displacement X", self.navx.getDisplacementX())
         self.sd.putNumber("Displacement Y", self.navx.getDisplacementY())
         self.sd.putBoolean("IsCalibrating", self.navx.isCalibrating())
         self.sd.putBoolean("IsConnected", self.navx.isConnected())
         self.sd.putNumber("Angle", self.navx.getAngle())
         self.sd.putNumber("Pitch", self.navx.getPitch())
         self.sd.putNumber("Yaw", self.navx.getYaw())
         print("YAW", self.navx.getYaw())
         self.sd.putNumber("Roll", self.navx.getRoll())
         # self.sd.putNumber("Analog", self.analog.getVoltage())
         self.sd.putNumber("Timestamp", self.navx.getLastSensorTimestamp())

   def TestPeriodic(self) -> None:
      pass


if __name__ == "__main__":
   wpilib.run(MyRobot)
