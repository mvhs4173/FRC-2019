/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import frc.robot.Hardware;

public class AutoPlaceGameObject extends Command {
  Vision vision;
  DriveTrain driveTrain;

  public AutoPlaceGameObject() {
    // Use requires() here to declare subsystem dependencies
    requires(Hardware.driveTrain);
    vision = Hardware.vision;
    driveTrain = Hardware.driveTrain;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double horizontalOffset = vision.getHorizontalOffset();
    String direction = (horizontalOffset < 0) ? "left" : "right";
    if(Math.abs(horizontalOffset) < 0.8){
        driveTrain.driveLeftUnitAtRPM(0.5*horizontalOffset);
        driveTrain.driveRightUnitAtRPM(0.4*horizontalOffset);
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
