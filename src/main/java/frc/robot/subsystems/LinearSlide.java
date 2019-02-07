/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Hardware;

/**
 * Add your docs here.
 */
public class LinearSlide extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private MotorController leftLift;
  private MotorController rightLift; 

  public LinearSlide(){
    this.leftLift = Hardware.linearSlideLeftLift;
    this.rightLift = Hardware.linearSlideRightLift;
  }

public int getPosition() {
  return leftLift.getEncoderPosition(); 
}

public void moveSlideUp() {
  leftLift.setVelocityRPM(50);
}

public void moveSlideDown() {
  leftLift.setVelocityRPM(-50);
}

public void stopSlideMovement(){
  leftLift.setVelocityRPM(0);
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  
  }
}
