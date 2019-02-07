package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch {
	private DigitalInput magSwitch;
	
	public LimitSwitch(int portNumber) {
		this.magSwitch = new DigitalInput(portNumber);
		
	}
	public boolean isTriggered() {
		return !magSwitch.get();
	}
}
