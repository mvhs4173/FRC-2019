/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class handles the vision processing for the camera.
 * using this class you can get information about vision targets on the field that the camera sees
 */
public class Vision {
    String horizontalOffsetKey = "HorizontalOffset";
    String horizontalTiltKey = "HorizontalTilt";
    String isAlignedKey = "isAligned";


    public Vision() {

    }

    /**
     * Detects if the robot is aligned with the vision target.
     * If there is no vision target that the camera can detect this function will return false
     * @return A boolean indicating if the robot is aligned with vision target
     */
    public boolean isAlignedWithTarget() {
        return SmartDashboard.getBoolean(isAlignedKey, false);
    }

    /**
     * Gets how much the robot is tilted when facing the vision target
     * If the robot is perpendicular to the target then it will return 0
     * @return A number indicating how much the robot is tilted
     */
    public double getHorizontalTilt() {
        return SmartDashboard.getNumber(horizontalTiltKey, -9999);
    }

    public double getHorizontalOffset() {
        return SmartDashboard.getNumber(horizontalOffsetKey, -9999);
    }
}
