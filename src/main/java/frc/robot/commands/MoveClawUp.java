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
 public class MoveClawUp extends Command {
  private boolean isFinished = false;

  private ClawSubsystem claw;
  // Limit switches for clawMotor and angleMotor

  public MoveClawUp() {
    // Use requires() here to declare subsystem dependencies
    requires(Hardware.claw);
    this.claw = Hardware.claw;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    claw.enableUpperLimit();
    SmartDashboard.putString("Claw Position", claw.getClawPosition().toString());
    //Make sure the claw isn't already in the position we want it to be in
    if (claw.getClawPosition() == ClawPosition.CLAW_UP) {
      isFinished = true;//Stop the command it's already at the target position
    }else {
      isFinished = false;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putString("Claw Position", claw.getClawPosition().toString());
    
    if (!isFinished) {
      claw.raiseClawSystem();

      //Check if the claw is in the desired position
      if (claw.getClawPosition() == ClawPosition.CLAW_UP) {
        isFinished = true;//Stop the command
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