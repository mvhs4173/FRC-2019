/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.*;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class ClawSubsystem extends Subsystem {
  private MotorController gripMotor;
  private MotorController angleMotor;
  private LimitSwitch clawUpLimit,
                  clawDownLimit;

  private double clawSpeed = MotorController.getPowerFromRPM(1);

  private enum Direction {
    UP,
    DOWN;
  }

  private ClawPosition lastClawPosition;
  private Direction lastClawDirection;



  public enum ClawPosition {
    CLAW_UP,
    CLAW_DOWN,
    IN_BETWEEN,
    INVALID;
  }

  public ClawSubsystem() {
    this.gripMotor = Hardware.clawGripMotor;
    this.angleMotor = Hardware.clawAngleMotor;
    gripMotor.resetEncoder();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
   * Raises the system
   */
  public void raiseClawSystem() {
    ClawPosition currentPosition = getClawPosition();
    //Figure out which direction to turn the motor to get to the top position
    if (lastClawPosition == ClawPosition.CLAW_UP && lastClawDirection == Direction.UP && currentPosition == ClawPosition.IN_BETWEEN) {
      angleMotor.setPercentSpeed(0.75);
      
      //When to set the position and direction as the "last" position/direction
      if (currentPosition == ClawPosition.CLAW_UP || currentPosition == ClawPosition.CLAW_DOWN) {
        lastClawPosition = currentPosition;
        lastClawDirection = Direction.DOWN;
      }
    }else {
      angleMotor.setPercentSpeed(-1);
      
      if (currentPosition == ClawPosition.CLAW_UP || currentPosition == ClawPosition.CLAW_DOWN) {
        lastClawPosition = currentPosition;
        lastClawDirection = Direction.UP;
      }
    }
  }

  public void raiseClawSlow() {
    angleMotor.setPercentSpeed(-0.75);
    lastClawDirection = Direction.UP;
    ClawPosition clawPosition = getClawPosition();

    if (clawPosition == ClawPosition.CLAW_UP || clawPosition == ClawPosition.CLAW_DOWN) {
      lastClawPosition = clawPosition;
    }
  }

  /**
   * Lowers the system
   */
  public void lowerClawSystem() {
    angleMotor.setPercentSpeed(0.75);
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
    SmartDashboard.putBoolean("ReverseTriggered", angleMotor.getReverseLimitSwitchTriggered());
    SmartDashboard.putBoolean("ForwardTriggered", angleMotor.getForwardLimitSwitchTriggered());
    //When the top limit switch is enabled
    if (angleMotor.getForwardLimitSwitchTriggered()) {
      position = ClawPosition.CLAW_UP;
    
    //When the bottom limit switch is triggered
    }else if(angleMotor.getReverseLimitSwitchTriggered()) {
      position = ClawPosition.CLAW_DOWN;

    //If neither limit switch is triggered then it is in between the max and minumum height
    }else if(!angleMotor.getForwardLimitSwitchTriggered() && !angleMotor.getReverseLimitSwitchTriggered()){
      position = ClawPosition.IN_BETWEEN;

    //If both switches are triggered then there is something wrong, this is an invalid position
    }else {
      position = ClawPosition.INVALID;
    }
    return position;
  }


  public void disableUpperLimit() {
    angleMotor.disableForwardLimitSwitch();
  }

  public void enableUpperLimit() {
    angleMotor.enableForwardLimitSwitch();
  }

  public int getGripperEncoderPosition() {
    return gripMotor.getEncoderPosition();
  }

  public void setGripperSpeed(double speed) {
    gripMotor.setVelocityRPS(speed);
  }
}