/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.*;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class ClawSubsystem extends Subsystem {
  private MotorController clawMotor;
  private MotorController angleMotor;
  private MotorController cimMotorOne;
  private MotorController cimMotorTwo;
  private MagneticLimitSwitch clawOpenLimitSwitch;
  private MagneticLimitSwitch clawCloseLimitSwitch;
  private double clawSpeed = 1;

  public ClawSubsystem(MotorController clawMotor,
                       MotorController angleMotor,
                       MotorController cimMotorOne,
                       MotorController cimMotorTwo,
                       MagneticLimitSwitch clawOpenLimitSwitch,
                       MagneticLimitSwitch clawCloseLimitSwitch) {
    this.clawMotor = clawMotor;
    this.clawOpenLimitSwitch = clawOpenLimitSwitch;
    this.clawCloseLimitSwitch = clawCloseLimitSwitch;
    this.angleMotor = angleMotor;
    this.cimMotorOne = cimMotorOne;
    this.cimMotorTwo = cimMotorTwo;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void clawOpen() {
    clawMotor.setVelocityRPM(clawSpeed);
  }

  public void clawClose() {
    clawMotor.setVelocityRPM(-clawSpeed);
  }

  public boolean isClawOpen() {
    return clawOpenLimitSwitch.isMagnetClose();
  }

  public boolean isClawClose() {
    return !clawCloseLimitSwitch.isMagnetClose();
  }
}