/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;

public class UltrasonicSensor {

  private Ultrasonic sensor;

  /**
   * @param triggerDIOChannel The Digital IO Channel number on the RIO that the trigger pin of the Ultrasonic sensor is connected to
   * @param echoDIOChannel The Digital IO Channel number on the RIO that the echo pin of the Ultrasonic sensor is connected to
   */
  public UltrasonicSensor(int triggerDIOChannel, int echoDIOChannel) {
    this.sensor = new Ultrasonic(triggerDIOChannel, echoDIOChannel);
    sensor.setAutomaticMode(true);//Enable "round robin" mode so that we can ping on command
  }

  /**
   * Sends out and Ultrasonic Wave
   */
  public void ping() {
    sensor.ping();
  }

  /**
   * If the sensor was able to get a valid range measurement since ping() was last called
   * @return A boolean, True if a valid measurment was made, False is there was no valid measurment 
   */
  public boolean gotRange() {
    return sensor.isRangeValid();
  }

  /**
   * Gets the distance in Inches that the sensor measured
   * @return A double
   */
  public double getDistanceInches() {
    return sensor.getRangeInches();
  }

  public double getDistanceMM() {
    return sensor.getRangeMM();
  }
}