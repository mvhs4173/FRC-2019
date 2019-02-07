/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ClawSubsystem;

/**
 * An example command.  You can replace me with your own command.
 */
 public class MoveClawDown extends Command {
<<<<<<< HEAD
  private boolean isFinished = false;

  private ClawSubsystem claw;
=======
  private boolean finished = false;
  private ClawSubsystem clawSubsystem;
  // Positive position of the claw
  private int clawNegativePosition = 10;
  private int clawEncoderPosition;
  private int clawTargetPosition;
>>>>>>> 91e2ba8f2a7e57f29af680fdfbd9d04eab8b2844
  // Limit switches for clawMotor and angleMotor

  public MoveClawDown(ClawSubsystem clawSubsystem) {
    // Use requires() here to declare subsystem dependencies
    requires(clawSubsystem);
    this.claw = clawSubsystem;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Make sure the claw isn't already in the position we want it to be in
    if (claw.isClawDown()) {
      isFinished = true;//Stop the command it's already at the target position
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
<<<<<<< HEAD
    if (!isFinished) {
      claw.lowerClawSystem();

      //Check if the claw is in the desired position
      if (claw.isClawDown()) {
        isFinished = true;//Stop the command
      }
    }
=======
    clawEncoderPosition = clawSubsystem.getAngleMotorPositionRaw();
    clawSubsystem.clawOpen();
    clawSubsystem.clawDown();
>>>>>>> 91e2ba8f2a7e57f29af680fdfbd9d04eab8b2844
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