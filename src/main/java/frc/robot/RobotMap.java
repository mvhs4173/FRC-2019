/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  public static int frontLeftMotor = 1;
  public static int backLeftMotor = 2;

  public static int frontRightMotor = 3;
  public static int backRightMotor = 4;
  
  public static int ultrasonicTriggerChannel = 0;
  public static int ultrasonicEchoChannel = 1;


  //////////CLAW SYSTEM IDs////////////
  public static int clawGripMotor = 7;
  public static int clawAngleMotor = 8;
  public static int clawLeftIntake = 6;
  public static int clawRightIntake = 42;

  public static int clawUpLimitChannel = 2;
  public static int clawDownLimitChannel = 3;

  ///////////Linear Slide IDs/////////////
  public static int linearSlideLeftLift = 9;
  public static int linearSlideRightLift = 10;
  public static int linearSlideBreak = 11;


  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
