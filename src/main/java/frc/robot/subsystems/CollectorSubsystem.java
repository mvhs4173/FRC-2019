/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class CollectorSubsystem extends Subsystem {
  private MotorController collectorMotor;
  private MagneticLimitSwitch clawOpenLimitSwitch;
  private MagneticLimitSwitch clawCloseLimitSwitch;
  private double clawSpeed = 1;
  private DigitalInput stopIntakeSystem;
  private DigitalInput moveCollectorDown;
  private DigitalInput moveCollectorUp;
  public CollectorSubsystem(MotorController clawMotor, MagneticLimitSwitch clawOpenLimitSwitch, MagneticLimitSwitch clawCloseLimitSwitch) {
    this.collectorMotor = clawMotor;
    this.clawOpenLimitSwitch = clawOpenLimitSwitch;
    this.clawCloseLimitSwitch = clawCloseLimitSwitch;
    this.moveCollectorDown = new DigitalInput(RobotMap.collectorDown);
    this.moveCollectorUp = new DigitalInput (RobotMap.collectorUp);
    this.stopIntakeSystem = new DigitalInput(RobotMap.stopIntake);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void clawOpen() {
    collectorMotor.setVelocityRPM(clawSpeed);
  }

  public void clawClose() {
    collectorMotor.setVelocityRPM(-clawSpeed);
  }

  public boolean isClawOpen() {
    return clawOpenLimitSwitch.isMagnetClose();
  }

  public boolean isClawClose() {
    return !clawCloseLimitSwitch.isMagnetClose();
  }
}