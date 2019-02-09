/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.*;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class ClawSubsystem extends Subsystem {
  private MotorController gripMotor;
  private MotorController angleMotor;
  private MotorController leftIntakeMotor;
  private MotorController rightIntakeMotor;
  private LimitSwitch clawUpLimit,
                  clawDownLimit;

  private double clawSpeed = MotorController.getPowerFromRPM(1);

  public enum ClawPosition {
    CLAW_UP,
    CLAW_DOWN,
    IN_BETWEEN,
    INVALID;
  }

  public ClawSubsystem() {
    this.gripMotor = Hardware.clawGripMotor;
    this.angleMotor = Hardware.clawAngleMotor;
    this.leftIntakeMotor =  Hardware.clawLeftIntake;
    this.rightIntakeMotor = Hardware.clawRightIntake;
    rightIntakeMotor.setFollower(leftIntakeMotor);
    rightIntakeMotor.setFollowerDirection(InvertType.OpposeMaster);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
   * Stops the intake motors
   */
  public void stopIntake() {
    leftIntakeMotor.setVelocityRPM(0);
    //rightIntakeMotor.setVelocityRPM(0);
  }

  public void intakeCargo(){
    leftIntakeMotor.setPercentSpeed(1);
  }

  public void expelCargo(){
    leftIntakeMotor.setPercentSpeed(-1);
  }

  /*
  public boolean cargoSwitchPressed(){
    return cargoLimit.isTriggered();
  }
  */

  /**
   * Raises the system
   */
  public void raiseClawSystem() {
    angleMotor.setPercentSpeed(clawSpeed);
  }

  /**
   * Lowers the system
   */
  public void lowerClawSystem() {
    angleMotor.setPercentSpeed(-clawSpeed);
  }

  /**
   * Stops the system from moving
   */
  public void stopClawSystem() {
    angleMotor.setPercentSpeed(0.0);
  }

  /**
   * Gets the position of the claw
   * @return A ClawPosition Enum describing the position of the claw
   */
  public ClawPosition getClawPosition() {
    ClawPosition position = ClawPosition.IN_BETWEEN;

    //When the top limit switch is enabled
    if (angleMotor.getForwardLimitSwitchTriggered()) {
      position = ClawPosition.CLAW_UP;
    
    //When the bottom limit switch is triggered
    }else if(angleMotor.getRevereseLimitSwitchTriggered()) {
      position = ClawPosition.CLAW_DOWN;

    //If neither limit switch is triggered then it is in between the max and minumum height
    }else if(!angleMotor.getForwardLimitSwitchTriggered() && !angleMotor.getRevereseLimitSwitchTriggered()){
      position = ClawPosition.IN_BETWEEN;

    //If both switches are triggered then there is something wrong, this is an invalid position
    }else {
      position = ClawPosition.INVALID;
    }

    return position;
  }
}