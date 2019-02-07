package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;
import frc.robot.subsystems.MotorController;
import frc.robot.subsystems.DriveUnit.UnitSide;
import frc.robot.subsystems.DriveUnit;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LinearSlide;

public class Hardware {
    public static MotorController frontLeftDriveMotor,
                                backLeftDriveMotor,
                                frontRightDriveMotor,
                                backRightDriveMotor;

    public static DriveUnit leftDriveUnit,
                        rightDriveUnit;

    public static DigitalInput stopIntakeSystem,
                        stopMovingUp,
                        stopMovingDown,
                        stopLinearSlideUp,
                        stopLinearSlideDown;

    public static DriveTrain driveTrain;

    public static LinearSlide linearSlide;
    //Init all the hardware
    public Hardware() {
        //Init limit switches
        stopIntakeSystem = new DigitalInput(RobotMap.stopIntake);
        stopMovingDown = new DigitalInput(RobotMap.collectorDown);
        stopMovingUp = new DigitalInput (RobotMap.collectorUp);
        stopLinearSlideUp = new DigitalInput(RobotMap.stopLinearSlideDown);
        stopLinearSlideDown = new DigitalInput(RobotMap.stopLinearSlideUp);


        //Init drivetrain motors
        frontLeftDriveMotor = new MotorController(RobotMap.frontLeftMotor);
        backLeftDriveMotor = new MotorController(RobotMap.backLeftMotor);
        frontRightDriveMotor = new MotorController(RobotMap.frontRightMotor);
        backLeftDriveMotor = new MotorController(RobotMap.backLeftMotor);
        
        //Init drive unit objects
        leftDriveUnit = new DriveUnit(frontLeftDriveMotor, backLeftDriveMotor, UnitSide.LEFT);
        rightDriveUnit = new DriveUnit(frontRightDriveMotor, backRightDriveMotor, UnitSide.RIGHT);
        driveTrain = new DriveTrain(rightDriveUnit, leftDriveUnit);

        //Init subsystems
        linearSlide = new LinearSlide(RobotMap.linearSlideMotor);
    }
}