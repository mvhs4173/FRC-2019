package org.usfirst.frc.team4173.robot.subsystems;

public class CoordinateSystem{
	private double currentPosition[];
	
	public CoordinateSystem(double startingCoordinate[]) {
			this.currentPosition = startingCoordinate;
	}
	
	/**
	 * Calculates how much the robot needs to rotate and how far it needs to travel for it to reach the given coordinate
	 * @param coordinate The new coordinate in (X, Y) format
	 * @return A double array where index 0 holds the angle in degrees the robot needs to rotate to and where index 1 holds the distance the robot needs to travel
	 */
	public double[] getCoordinateChangeInfo(double coordinate[]) {
		//Get the distance between the two coordinates on both axis
		double deltaX = currentPosition[0] - coordinate[0];
		double deltaY = currentPosition[1] - coordinate[1];
		
		//Find distance between points using pythagorean theorem, distance is in feet
		double distanceFeet = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
		
		//Determine the angle you will need to face to get to the point
		double angle = -Degrees.subtract(Degrees.radiansToDegrees(Math.atan2(deltaY,  deltaX)), 180.0);
		
		double infoTable[] = {angle, distanceFeet};//What we give to the caller
		return infoTable;
	}
	
	/**
	 * Sets the coordinate that the system considers the robot to be located at
	 * @param position The coordinate the robot is at in (X, Y) format
	 */
	public void setCurrentPosition(double position[]) {
		currentPosition = position;
	}
	
	/**
	 * Returns what the system has stored for the robot's current position
	 * @return The coordinate of the robot's position in (X, Y) format
	 */
	public double[] getCurrentPosition() {
		return currentPosition;
	}
}
