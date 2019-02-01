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
                        stopMovingDown;

    public static DriveTrain driveTrain;

    public static LinearSlide linearSlide;
    //Init all the hardware
    public Hardware() {
        //Init limit switches
        this.stopIntakeSystem = new DigitalInput(RobotMap.stopIntake);
        this.stopMovingDown = new DigitalInput(RobotMap.collectorDown);
        this.stopMovingUp = new DigitalInput (RobotMap.collectorUp);
        //Init drivetrain motors
        this.frontLeftDriveMotor = new MotorController(RobotMap.frontLeftMotor);
        this.backLeftDriveMotor = new MotorController(RobotMap.backLeftMotor);
        this.frontRightDriveMotor = new MotorController(RobotMap.frontRightMotor);
        this.backLeftDriveMotor = new MotorController(RobotMap.backLeftMotor);
        
        //Init drive unit objects
        this.leftDriveUnit = new DriveUnit(frontLeftDriveMotor, backLeftDriveMotor, UnitSide.LEFT);
        this.rightDriveUnit = new DriveUnit(frontRightDriveMotor, backRightDriveMotor, UnitSide.RIGHT);
        this.driveTrain = new DriveTrain(rightDriveUnit, leftDriveUnit);

        //Init subsystems
        this.linearSlide = new LinearSlide(RobotMap.linearSlideMotor);
    }
}