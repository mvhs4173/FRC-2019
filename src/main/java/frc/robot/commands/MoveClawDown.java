/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClawSubsystem.ClawPosition;

/**
 * An example command.  You can replace me with your own command.
 */
 public class MoveClawDown extends Command {
  private boolean isFinished = false;

  private ClawSubsystem claw;
  // Limit switches for clawMotor and angleMotor

  public MoveClawDown() {
    // Use requires() here to declare subsystem dependencies
    requires(Hardware.claw);
    this.claw = Hardware.claw;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Make sure the claw isn't already in the position we want it to be in
    if (claw.getClawPosition() == ClawPosition.CLAW_DOWN) {
      isFinished = true;//Stop the command it's already at the target position
    }else {
      isFinished = false;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!isFinished) {
      claw.lowerClawSystem();

      //Check if the claw is in the desired position
      if (claw.getClawPosition() == ClawPosition.CLAW_DOWN) {
        SmartDashboard.putBoolean("Clawdown Finished", true);
        isFinished = true;//Stop the command
        claw.stopClawSystem();
      }else {
        isFinished = false;
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    claw.stopClawSystem();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    claw.stopClawSystem();
  }
}