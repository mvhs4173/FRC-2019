package org.usfirst.frc.team4173.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

public class MagneticLimitSwitch {
	private DigitalInput magSwitch;
	public MagneticLimitSwitch(int portNumber) {
		this.magSwitch = new DigitalInput(portNumber);
		
	}
	public boolean isMagnetClose() {
		return !magSwitch.get();
	}
}
