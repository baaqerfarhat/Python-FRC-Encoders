
from ctre import TalonSRX
import wpilib
from wpilib.drive import MecanumDrive

class MyRobot(wpilib.TimedRobot):

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0

    def robotInit(self):
        """Robot initialization function"""

        self.frontLeftMotor = TalonSRX(4)
        self.rearLeftMotor = TalonSRX(3)
        self.frontRightMotor = TalonSRX(1)
        self.rearRightMotor = TalonSRX(2)

        # invert the left side motors
        self.frontLeftMotor.setInverted(True)

        # you may need to change or remove this to match your robot
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
            self.stick.getX(), self.stick.getY(), self.stick.getZ(), 0
        )


if __name__ == "__main__":
    wpilib.run(MyRobot)