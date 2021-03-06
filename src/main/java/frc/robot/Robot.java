/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ReleaseLinearBrake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SmoothedJoystick;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static boolean switchToHatch;
  public static OI oi;
  public static Hardware hardware;
  public static SmoothedJoystick joystick;
  public static DriveTrain driveTrain;
  public static Preferences prefs;
  

  ReleaseLinearBrake releaseBrake; 

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    hardware = new Hardware();//Init all the robot hardware
    switchToHatch = oi.switchToHatch.get();
    oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    prefs = Preferences.getInstance();
    joystick=oi.joy;
    driveTrain=hardware.driveTrain;
    releaseBrake = new ReleaseLinearBrake();
  }
 
  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
  
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }

    releaseBrake.start();
    //moveToMid.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    double throttle = joystick.getThrottle();
    double x = joystick.getXValue();
    double y = joystick.getYValue();

    driveTrain.setReverseMode(false);
    driveTrain.driveWithJoystick(x, y, throttle);
    
    Scheduler.getInstance().run();

    //Put the HSV threshold values on the smart dashboard so that the vision system can retrieve them
    SmartDashboard.putNumber("HLOW", prefs.getInt("HLOW", 0));
    SmartDashboard.putNumber("SLOW", prefs.getInt("SLOW", 0));
    SmartDashboard.putNumber("VLOW", prefs.getInt("VLOW", 0));

    SmartDashboard.putNumber("HHIGH", prefs.getInt("HHIGH", 180));
    SmartDashboard.putNumber("SHIGH", prefs.getInt("SHIGH", 255));
    SmartDashboard.putNumber("VHIGH", prefs.getInt("VHIGH", 255));

    SmartDashboard.putBoolean("QuadPinState", Hardware.linearSlide.getQuadPinState());
    SmartDashboard.putNumber("Linearslide Position", Hardware.linearSlide.getPosition());
    SmartDashboard.putNumber("Linearslide Amperage", Hardware.linearSlide.getAmpUsage());


  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double throttle = joystick.getThrottle();
    double x = joystick.getXValue();
    double y = joystick.getYValue();

    switchToHatch = OI.switchToHatch.get();

    driveTrain.setReverseMode(false);
    driveTrain.driveWithJoystick(x, y, throttle);
    
    SmartDashboard.putBoolean("swith for hatch", OI.switchToHatch.get());
    
    SmartDashboard.putNumber("getYV", OI.joy.getYValue());
    SmartDashboard.putNumber("getXV", OI.joy.getXValue());
    SmartDashboard.putNumber("getY", OI.joy.getY());
    SmartDashboard.putNumber("getX", OI.joy.getX());

    SmartDashboard.putNumber("Linearslide  Position", Hardware.linearSlide.getPosition());
    SmartDashboard.putBoolean("BreakLimitStatus", Hardware.linearSlide.checkBrake());
    SmartDashboard.putNumber("LinearAmprege", Hardware.linearSlide.getAmpUsage());

    SmartDashboard.putString("Claw Position", Hardware.claw.getClawPosition().toString());
    SmartDashboard.putBoolean("Linear brake Triggered", Hardware.linearSlide.checkBrake());

    SmartDashboard.putNumber("Gripper Position", Hardware.claw.getGripperEncoderPosition());
    SmartDashboard.putBoolean("CargoSwitch", Hardware.intake.cargoSwitchPressed());

    SmartDashboard.putBoolean("ReverseLimAngle", Hardware.clawAngleMotor.getReverseLimitSwitchTriggered());

    Scheduler.getInstance().run();
  }

  /**
   * Test mode will run color picker code for vision processing.
   */
  @Override
  public void testPeriodic() {
    /*
    if(OI.switchToHatch.get()){
      Hardware.clawRightIntake.setPercentSpeed(1);
    } else {
      Hardware.clawRightIntake.setPercentSpeed(-1);
    }
    */
  }
}