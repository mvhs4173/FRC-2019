package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;
import frc.robot.subsystems.MotorController;
import frc.robot.subsystems.UltrasonicSensor;
import frc.robot.subsystems.DriveUnit.UnitSide;
import frc.robot.subsystems.DriveUnit;
import frc.robot.subsystems.LimitSwitch;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LinearSlide;

public class Hardware {
    public static MotorController frontLeftDriveMotor,
                                backLeftDriveMotor,
                                frontRightDriveMotor,
                                backRightDriveMotor,
                                clawGripMotor,
                                clawAngleMotor,
                                clawLeftIntake,
                                clawRightIntake,
                                linearSlideLeftLift,
                                linearSlideRightLift;

    public static DriveUnit leftDriveUnit,
                        rightDriveUnit;

    public static DigitalInput stopIntakeSystem,
                        stopMovingUp,
                        stopMovingDown,
                        stopLinearSlideUp,
                        stopLinearSlideDown;

    public static UltrasonicSensor frontDistanceSensor;

    public static DriveTrain driveTrain;

    public static LinearSlide linearSlide;
    public static ClawSubsystem claw;

    public static LimitSwitch clawUpLimitSwitch,
                        clawDownLimitSwitch;

    //Init all the hardware
    public Hardware() {

        //Init drivetrain motors
        frontLeftDriveMotor = new MotorController(RobotMap.frontLeftMotor);
        backLeftDriveMotor = new MotorController(RobotMap.backLeftMotor);
        frontRightDriveMotor = new MotorController(RobotMap.frontRightMotor);
        backRightDriveMotor = new MotorController(RobotMap.backRightMotor);
        
        //Init drive unit objects
        leftDriveUnit = new DriveUnit(frontLeftDriveMotor, backLeftDriveMotor, UnitSide.LEFT);
        rightDriveUnit = new DriveUnit(frontRightDriveMotor, backRightDriveMotor, UnitSide.RIGHT);
        driveTrain = new DriveTrain(rightDriveUnit, leftDriveUnit);

        ///////////LINEAR SLIDE/////////
        linearSlideLeftLift = new MotorController(RobotMap.linearSlideLeftLift);
        linearSlideRightLift = new MotorController(RobotMap.linearSlideRightLift);
        linearSlideRightLift.setFollower(linearSlideRightLift);//Make the right lifter follow the left lifter so that they will always do the same thing
        linearSlide = new LinearSlide();

        frontDistanceSensor = new UltrasonicSensor(RobotMap.ultrasonicTriggerChannel, RobotMap.ultrasonicEchoChannel);
        //Claw//
        clawGripMotor = new MotorController(RobotMap.clawGripMotor);
        clawAngleMotor = new MotorController(RobotMap.clawAngleMotor);
        clawLeftIntake = new MotorController(RobotMap.clawLeftIntake);
        clawRightIntake = new MotorController(RobotMap.clawRightIntake);

        clawUpLimitSwitch = new LimitSwitch(RobotMap.clawUpLimitChannel);
        clawDownLimitSwitch = new LimitSwitch(RobotMap.clawDownLimitChannel);

        claw = new ClawSubsystem();
    }
}