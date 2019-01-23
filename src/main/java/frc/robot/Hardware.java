package frc.robot;

import frc.robot.RobotMap;
import frc.robot.subsystems.MotorController;
import frc.robot.subsystems.DriveUnit.UnitSide;
import frc.robot.subsystems.DriveUnit;

class Hardware {
    public static MotorController frontLeftDriveMotor,
                                backLeftDriveMotor,
                                frontRightDriveMotor,
                                backRightDriveMotor;

    public static DriveUnit leftDriveUnit,
                        rightDriveUnit;

    //Init all the hardware
    public Hardware() {
        //Init drivetrain motors
        this.frontLeftDriveMotor = new MotorController(RobotMap.frontLeftMotor);
        this.backLeftDriveMotor = new MotorController(RobotMap.backLeftMotor);
        this.frontRightDriveMotor = new MotorController(RobotMap.frontRightMotor);
        this.backLeftDriveMotor = new MotorController(RobotMap.backLeftMotor);

        //Init drive unit objects
        this.leftDriveUnit = new DriveUnit(frontLeftDriveMotor, backLeftDriveMotor, UnitSide.LEFT);
        this.rightDriveUnit = new DriveUnit(frontRightDriveMotor, backRightDriveMotor, UnitSide.RIGHT);
    }
}