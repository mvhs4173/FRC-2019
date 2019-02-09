/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.*;
import frc.robot.*;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public static Joystick joy = new Joystick(0);
	public static Joystick launchpad = new Joystick(1);
  public Button slideUp = new JoystickButton(joy, 1),
        slideDown = new JoystickButton(joy, 2),
        intakeCargo = new JoystickButton(joy,3),
        expelCargo = new JoystickButton(joy,4),
        angleCollectorDown = new JoystickButton(joy,5),
        angleCollectorUp = new JoystickButton(joy, 6),
        releseHatch = new JoystickButton(joy, 7),
        gripHatch = new JoystickButton(joy, 8);
  public OI (){
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
    slideUp.whenPressed(new MoveLinearSlideUp());
    slideDown.whenPressed(new MoveLinearSlideDown());
    intakeCargo.whenPressed(new IntakeCargo());
    expelCargo.whenPressed(new ExpelCargo());
    angleCollectorDown.whenPressed(new MoveClawDown());
    angleCollectorUp.whenPressed(new MoveClawUp());
    releseHatch.whenPressed(new ReleseHatch());
    gripHatch.whenPressed(new GripHatch());
    intakeCargo.whenReleased(new StopIntake());
    expelCargo.whenReleased(new StopIntake());

  }
}
