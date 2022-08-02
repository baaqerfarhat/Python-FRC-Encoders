from ast import While
from time import sleep, time
#from typing_extensions import Self
from webbrowser import get
from hal import getEncoder, getEncoderDirection, getEncoderDistance
from numpy import True_
import wpilib
from wpilib.drive import MecanumDrive
import ctre
from networktables import NetworkTables
import navx
import wpimath.controller
from navx import AHRS
import math
import wpilib.drive
from wpilib import Encoder, Ultrasonic
#from rev import ColorSensorV3




def run():
    raise ValueError()


class MyRobot(wpilib.TimedRobot):

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0


    if wpilib.RobotBase.isSimulation():
        # These PID parameters are used in simulation
        kP = 0.06
        kI = 0.00
        kD = 0.00
    else:
        # These PID parameters are used on a real robot
        kP = 0.6
        kI = 2
        kD = 0.125

    kToleranceDegrees = 2.0




    def robotInit(self):
        """Robot initialization function"""

        self.Ultrasonic1 = Ultrasonic(1,2)

       

        
       # self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)


        self.frontLeftMotor = ctre.WPI_TalonSRX(4)
        self.rearLeftMotor = ctre.WPI_TalonSRX(3)
        self.frontRightMotor = ctre.WPI_TalonSRX(1)
        self.rearRightMotor = ctre.WPI_TalonSRX(2)

        self.timer = wpilib.Timer()

        turnController = wpimath.controller.PIDController(
            self.kP,
            self.kI,
            self.kD,
        )
        turnController.enableContinuousInput(-180.0, 180.0)
        turnController.setTolerance(self.kToleranceDegrees)

        self.turnController = turnController

        self.sd = NetworkTables.getTable("SmartDashboard")

        self.navx = navx.AHRS.create_spi()
        self.ahrs = AHRS.create_spi()

        # invert the left side motors
        self.frontLeftMotor.setInverted(True)
        self.rearLeftMotor.setInverted(True)

        self.drive = MecanumDrive(
            self.frontLeftMotor,
            self.rearLeftMotor,
            self.frontRightMotor,
            self.rearRightMotor,
        )

        self.drive.setExpiration(0.1)

        self.stick = wpilib.Joystick(self.joystickChannel)

   
    def disabledInit(self):
        self.logger.info("Entered disabled mode")

        self.timer.reset()
        self.timer.start()

    def disabledPeriodic(self):
        if self.timer.hasPeriodPassed(1):
            self.sd.putNumber("Displacement X", self.navx.getDisplacementX())
            self.sd.putNumber("Displacement Y", self.navx.getDisplacementY())
            self.sd.putBoolean("IsCalibrating", self.navx.isCalibrating())
            self.sd.putBoolean("IsConnected", self.navx.isConnected())
            self.sd.putNumber("Angle", self.navx.getAngle())
            self.sd.putNumber("Pitch", self.navx.getPitch())
            self.sd.putNumber("Yaw", self.navx.getYaw())
            self.sd.putNumber("Velocity", self.navx.getVelocityY())
            print("YAW", self.navx.getYaw())
            print("Angle", self.navx.getAngle())
            print("Displacemnt X", self.navx.getDisplacementX())
            print("Displacemnt Y", self.navx.getDisplacementY())
            #print("Displacemnt Z", self.navx.getDisplacementZ())
            
            self.sd.putNumber("Roll", self.navx.getRoll())
            # self.sd.putNumber("Analog", self.analog.getVoltage())
            self.sd.putNumber("Timestamp", self.navx.getLastSensorTimestamp())

    def teleopInit(self):
        self.drive.setSafetyEnabled(True)
        self.tm = wpilib.Timer()
        self.tm.start()

    
    def teleopPeriodic(self):
        #detectedColor = self.colorSensor.getColor()

        #ir = self.colorSensor.getIR()

     #   wpilib.SmartDashboard.putNumber("Red", detectedColor.red)
      #  wpilib.SmartDashboard.putNumber("Green", detectedColor.green)
      #  wpilib.SmartDashboard.putNumber("Blue", detectedColor.blue)
      #  wpilib.SmartDashboard.putNumber("IR", ir)

     #   proximity = self.colorSensor.getProximity()

     #   wpilib.SmartDashboard.putNumber("Proximity", proximity)

    #    rawDetectedColor = self.colorSensor.getRawColor()
#
      #  wpilib.SmartDashboard.putNumber("Raw Red", rawDetectedColor.red)
       # wpilib.SmartDashboard.putNumber("Raw Green", rawDetectedColor.green)
      #  wpilib.SmartDashboard.putNumber("Raw Blue", rawDetectedColor.blue)
      #  wpilib.SmartDashboard.putNumber("Raw IR", rawDetectedColor.ir)

        
        Encoder1 = Encoder(0, 1)
        Encoder2 = Encoder(2, 3)
        Encoder3 = Encoder(4, 5)
        Encoder4 = Encoder(6, 7)

        print("Encoder 1 ", Encoder1.getDistance())
        print("Encoder 2 ", Encoder2.getDistance())
        print("Encoder 3 ",  Encoder3.getDistance())
        print("Encoder 4 ",  Encoder4.getDistance())


        """Runs the motors with Mecanum drive."""
        # Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        # This sample does not use field-oriented drive, so the gyro input is set to zero.

        if self.tm.hasPeriodPassed(2.0):
            print("NavX Gyro", self.ahrs.getYaw(), self.ahrs.getAngle())

        rotateToAngle = False

        wpilib.SmartDashboard.putNumber("Yaw", self.ahrs.getYaw())
        wpilib.SmartDashboard.putNumber("Angle Adj", self.ahrs.getAngleAdjustment())
        wpilib.SmartDashboard.putNumber("Displacement", self.ahrs.getDisplacementY())
        wpilib.SmartDashboard.putNumber("Velocity m/s", self.ahrs.getVelocityY())
        wpilib.SmartDashboard.putNumber("KP", self.kP)
        wpilib.SmartDashboard.putNumber("KI", self.kI)
        wpilib.SmartDashboard.putNumber("KD", self.kD)

        #wpilib.SmartDashboard.putData("Encoder 1", Encoder1)
        

        if self.stick.getRawButton(1):  
            self.ahrs.reset()

        if self.stick.getRawButton(2):
            setpoint = 0.0
            rotateToAngle = True
        elif self.stick.getRawButton(3):
            print("Yo mama says its pressed yeah")
            #TurnAng = self.ahrs.getAngleAdjustment() + 90
            while self.ahrs.getYaw() < 81.5 :
                self.frontLeftMotor.set(1)
                self.frontRightMotor.set(-1)
                self.rearLeftMotor.set(1)
                self.rearRightMotor.set(-1)

                if self.ahrs.getYaw() >= 90:
                    self.ahrs.reset()
                    break
            
        elif self.stick.getRawButton(4):
            setpoint = 179.9
            rotateToAngle = True
        elif self.stick.getRawButton(5):
            setpoint = -90.0
            rotateToAngle = True

        if rotateToAngle:
            currentRotationRate = self.turnController.calculate(
                self.ahrs.getYaw(), setpoint
            )
        else:
            self.turnController.reset()
            currentRotationRate = self.stick.getTwist()


        self.drive.driveCartesian(
            self.stick.getY(), self.stick.getX(), self.stick.getZ(),
        )



    def autonomousInit(self):
        self.timer.reset()
        self.timer.start()
    
    
    def autonomousPeriodic(self): 
#Uses strafing to autonomously form a square
            #wpilib.Encoder()
        wpilib.SmartDashboard.putNumber("Velocity Y m/s", self.ahrs.getVelocityY())
        wpilib.SmartDashboard.putNumber("Displacement Y ", self.ahrs.getDisplacementY())
        wpilib.SmartDashboard.putNumber("Acceleration Y", self.ahrs.getRawAccelY())
        wpilib.SmartDashboard.putNumber("Ultrasonic", self.Ultrasonic1.getRange())

        
       # if -0.1 <= round(self.ahrs.getVelocityY(), 2) >= 0.1 and self.timer.get() > 0.3:
        #    self.frontLeftMotor.set(0)
         #   self.frontRightMotor.set(0)
          #  self.rearLeftMotor.set(0)
           # self.rearRightMotor.set(0)
            
       # else:
        #    self.frontLeftMotor.set(-1)
         #   self.frontRightMotor.set(-1)
          #  self.rearLeftMotor.set(-1)
           # self.rearRightMotor.set(-1)



        self.Ultrasonic1.setAutomaticMode(True)


        if self.Ultrasonic1.getRange() < 1:
            print("Detected")

            self.frontLeftMotor.set(1)
            self.frontRightMotor.set(-1)
            self.rearLeftMotor.set(1)
            self.rearRightMotor.set(-1)

        else:
            print("Not Read")
            self.frontLeftMotor.set(-1)
            self.frontRightMotor.set(-1)
            self.rearLeftMotor.set(-1)
            self.rearRightMotor.set(-1)

        def meterTime(meters):
            timeToTravel = meters * 1.7 #for meters multiply by 1.7
            while  self.timer.get() < timeToTravel:
                self.frontLeftMotor.set(-1)
                self.frontRightMotor.set(-1)
                self.rearLeftMotor.set(-1)
                self.rearRightMotor.set(-1)

        
        def m_forward_timer() :
            while self.timer.get() <= 3.0:
                    self.frontLeftMotor.set(-1)
                    self.frontRightMotor.set(-1)
                    self.rearLeftMotor.set(-1)
                    self.rearRightMotor.set(-1)
                    
            self.timer.reset()
        def m_left_turn_inplace():
            while self.timer.get() <= 1.05:
                    self.frontLeftMotor.set(-1)
                    self.frontRightMotor.set(1)
                    self.rearLeftMotor.set(-1)
                    self.rearRightMotor.set(1)
                    
            self.timer.reset()
        def m_right_turn_inplace():
            while self.timer.get() <= 1:
                    self.frontLeftMotor.set(1)
                    self.frontRightMotor.set(-1)
                    self.rearLeftMotor.set(1)
                    self.rearRightMotor.set(-1)
                    
            self.timer.reset()
        def m_reverse_timer():
            while self.timer.get() <= 3.0:
                    self.frontLeftMotor.set(1)
                    self.frontRightMotor.set(1)
                    self.rearLeftMotor.set(1)
                    self.rearRightMotor.set(1)
                    
            self.timer.reset()
           
        def m_stop():
                self.timer.set(5)
                self.frontLeftMotor.set(0)
                self.frontRightMotor.set(0)
                self.rearLeftMotor.set(0)
                self.rearRightMotor.set(0)
    

        def driveForwardTest():
            self.timer.reset()
            self.timer.start()

        #meterTime(2)

                    

       # m_forward_timer()
       # m_left_turn_inplace()
       # m_forward_timer()
       # m_left_turn_inplace()
       # m_forward_timer()
       # m_left_turn_inplace()
       # m_forward_timer()
       # m_left_turn_inplace()
       # m_stop()
    

            
if __name__ == "__main__":
    wpilib.run(MyRobot)

    








