

from time import sleep, time
import wpilib
from wpilib.drive import MecanumDrive
import ctre
import math


class MyRobot(wpilib.TimedRobot):

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0

    def robotInit(self):
        """Robot initialization function"""

        self.frontLeftMotor = ctre.WPI_TalonSRX(4)
        self.rearLeftMotor = ctre.WPI_TalonSRX(3)
        self.frontRightMotor = ctre.WPI_TalonSRX(1)
        self.rearRightMotor = ctre.WPI_TalonSRX(2)

        self.timer = wpilib.Timer()
        

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

    def teleopInit(self):
        self.drive.setSafetyEnabled(True)

    

    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""
        # Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        # This sample does not use field-oriented drive, so the gyro input is set to zero.
        self.drive.driveCartesian(
            self.stick.getY(), self.stick.getX(), self.stick.getZ(),
        )

    
    
    def autonomousInit(self):
        self.timer.reset()
        self.timer.start()
    
    
    def autonomousPeriodic(self): 
#Uses strafing to autonomously form a square
            #wpilib.Encoder()

                while self.timer.get() <= 3.0:
                    self.frontLeftMotor.set(1)
                    self.frontRightMotor.set(1)
                    self.rearLeftMotor.set(1)
                    self.rearRightMotor.set(1)
                    
                while 4.0 <= self.timer.get()<= 8.0:
                    self.frontLeftMotor.set(-1)
                    self.frontRightMotor.set(1)
                    self.rearLeftMotor.set(1)
                    self.rearRightMotor.set(-1)
                    
                while 9.0 <= self.timer.get() <= 12.0:
                 self.frontLeftMotor.set(-1)
                 self.frontRightMotor.set(-1)
                 self.rearLeftMotor.set(-1)
                 self.rearRightMotor.set(-1)
                 
                while 14.0 <= self.timer.get() <= 18.0: 
                     self.frontLeftMotor.set(1)
                     self.frontRightMotor.set(-1)
                     self.rearLeftMotor.set(-1)
                     self.rearRightMotor.set(1)
                     


            
if __name__ == "__main__":
    wpilib.run(MyRobot)

    








