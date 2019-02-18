package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;
import frc.robot.subsystems.MotorController;
import frc.robot.subsystems.UltrasonicSensor;
import frc.robot.subsystems.DriveUnit.UnitSide;
import frc.robot.subsystems.DriveUnit;
import frc.robot.subsystems.IntakeSystem;
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
                                linearSlideRightLift,
                                linearSlideBreak;

    public static DriveUnit leftDriveUnit,
                        rightDriveUnit;

    public static UltrasonicSensor frontDistanceSensor;

    public static DriveTrain driveTrain;

    public static LinearSlide linearSlide;
    public static ClawSubsystem claw;

    public static LimitSwitch clawUpLimitSwitch,
                        clawDownLimitSwitch;

    public static IntakeSystem intake;

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
        linearSlideBreak = new MotorController(RobotMap.linearSlideBreak);
        linearSlideLeftLift = new MotorController(RobotMap.linearSlideLeftLift);
        linearSlideLeftLift.setDirection(InvertType.InvertMotorOutput);
        int ticksPerRevolution = 1024;
        linearSlideLeftLift.configQuadEncoder(ticksPerRevolution);//Set up the quadrature encoder linked to the linear slide
        linearSlideLeftLift.setBrakeMode(true);
        linearSlideRightLift = new MotorController(RobotMap.linearSlideRightLift);
        linearSlideRightLift.setFollower(linearSlideLeftLift);//Make the right lifter follow the left lifter so that they will always do the same thing
        linearSlideRightLift.setDirection(InvertType.OpposeMaster);
        
        linearSlide = new LinearSlide();

        frontDistanceSensor = new UltrasonicSensor(RobotMap.ultrasonicTriggerChannel, RobotMap.ultrasonicEchoChannel);
        //Claw//
        clawGripMotor = new MotorController(RobotMap.clawGripMotor);
        int gripperTicksPerRevolution = 1680;
        clawGripMotor.configQuadEncoder(gripperTicksPerRevolution);
        clawAngleMotor = new MotorController(RobotMap.clawAngleMotor);
        clawLeftIntake = new MotorController(RobotMap.clawLeftIntake);
        clawRightIntake = new MotorController(RobotMap.clawRightIntake);
        
        //Enable limit switches on the angle motor so that the motor will stop when either of them are triggered
        clawAngleMotor.enableForwardLimitSwitch();
        clawAngleMotor.enableReverseLimitSwitch();

        intake = new IntakeSystem();

        claw = new ClawSubsystem();
    }
}