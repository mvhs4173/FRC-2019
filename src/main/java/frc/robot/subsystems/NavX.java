package org.usfirst.frc.team4173.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class NavX{
	
	SPI.Port navxPort = SPI.Port.kMXP;

	AHRS navX = null;
	
	double originAngle;
	
	//Initialize the navX object
	
	
	public NavX() {
		navX = new AHRS(navxPort);
	}
	
	double getRelativeAngle(double angle) {
		return angle - originAngle;//The angle of the robot based on its starting angle
	}
	
	
	//Returns the given angle in an angle that can range from -180 to +180
	   
	/** 
	 * @param angle The angle to convert
	 * @return Returns an angle that has been converted to a range of -180 to +180 degrees
	 */ 
	public double convertTo180Angles(double angle) {
		double relativeAngle = getRelativeAngle(angle);
		
		double angle360 = relativeAngle - Math.floor(relativeAngle/360) * 360;
		double angle180 = angle360;
		
		if (angle360 >= 180) {
			angle180 = angle360 - 360;
		}
		return angle180;
	}   
	    
	//Returns the heading in the range of -180 to +180
	    
	/**
	 * @return Returns the heading of the robot. Its heading can be anything from -180 to +180 degrees, Positive angles are equivalent to a clockwise turn 
	 */
	public double getHeading() {
		double rawHeading = navX.getAngle();
		double finalAngle = convertTo180Angles(rawHeading);
		return finalAngle;
	}
	
	//Gives the raw heading that we get from the sensor
	
	/**
	 * @return Returns the raw un-altered angle from the NavX
	 */
	public double getRawHeading() {
		return navX.getAngle();
	}
	
	//Gives the roll of the robot from -180 to +180
	/**
	 * @return Returns the current roll value (in degrees, from -180 to 180) reported by the sensor. Roll is a measure of rotation around the X Axis. A positive angle indicates an upward rotation
	 * Returns:The current roll value in degrees (-180 to 180).
	 */
	public double getRoll() {
		double rawRoll = navX.getRoll();
		return -rawRoll;//Negate it because the sensor makes it so going down is positive and up is negative
	}
	/**
	 * 
	 * @return Returns the current altitude, based upon calibrated readings from a barometric pressure sensor, and the currently-configured sea-level barometric pressure [navX Aero only]. This value is in units of meters. 
	 * NOTE: This value is only valid sensors including a pressure sensor. To determine whether this value is valid, see isAltitudeValid(). 

	 * Returns:Returns current altitude in meters (as long as the sensor includes an installed on-board pressure sensor).
	 */
	//Gives the altitude
	public double getAltitude() {
		return navX.getAltitude();
	}
	
	/**
	 * 
	 * @return Returns true if the sensor is currently automatically calibrating the gyro and accelerometer sensors.
	 */
	public boolean getIsCalibrating() {
		return navX.isCalibrating();
	}
	
	/**
	 * @return Returns which absolute angle is considered the origin
	 */
	public double getOrigin() {
		return originAngle;
	}
	
	/**
	 * Sets the current angle of the robot as its origin point
	 */
	public void setOrigin() {
		originAngle = navX.getAngle();
	}

}
