/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Hardware;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class AutoAssist extends Command {
  private boolean driverHasControl = false;//Indicates if the driver has manual control over the robot
  private DriveTrain driveTrain;
  private Joystick joystick;

  public enum TargetType {
    HATCH,
    CARGO;
  }

  public enum TargetLevel {
    LOW,
    MEDIUM,
    HIGH;
  }

  public AutoAssist(Joystick joystick) {
    // Use requires() here to declare subsystem dependencies
    requires(Hardware.driveTrain);
    driveTrain = Hardware.driveTrain;
    this.joystick = joystick;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    driverHasControl = true;//Give the driver control until we detect that we need to take over
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //If the driver currently has control over the robot
    if (driverHasControl) {
      driveTrain.driveWithJoystick(joystick.getX(), joystick.getY(), joystick.getThrottle());
    }else {
      driveTrain.driveUnitAtRPM(0.0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
