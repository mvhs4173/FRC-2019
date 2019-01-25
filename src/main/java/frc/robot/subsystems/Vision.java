package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;


/**
 * Vision processing class used for getting information about what the robot is currently seeing
 */
public class Vision {
    /**
     * An Enum that indicates what the Camera is targeting 
     */
    public enum VisionTargetType {
        CARGO,
        REFLECTIVE_TAPE,
        NONE;
    }

    private NetworkTableInstance networkTables;//Network tables service
    private NetworkTable visionTable;//The network table that holds all the information we need
    private NetworkTableEntry visionTargetPosition;
    private NetworkTableEntry visionTargetSize;
    private NetworkTableEntry visionTargetHorizontalOffset;
    private NetworkTableEntry visionTargetType;

    //Keys for data in the Network Tables
    private String targetPositionKey = "TargetPosition";
    private String targetSizeKey = "TargetSize";
    private String targetHorizontalOffsetKey = "HorizontalOffset";
    private String targetTypeKey = "TargetType";

    Number[] defaultVector = {-1, -1};

    /**
     * Initializes the Vision Processing by connecting to camera
     */
    public Vision() {
        networkTables = NetworkTableInstance.getDefault();//Startup Network tables instance
        visionTable = networkTables.getTable("SmartDashboard");//Get the table that the informatio will be transfered across

        //Set up the Entries in the table
        visionTargetPosition = visionTable.getEntry(targetPositionKey);
        visionTargetSize = visionTable.getEntry(targetSizeKey);
        visionTargetHorizontalOffset = visionTable.getEntry(targetHorizontalOffsetKey);
        visionTargetType = visionTable.getEntry(targetTypeKey);
    }

    /**
     * @return Return the X and Y coordinate of the center of the vision target the camera currently sees as a Integer Array If the camera does not have a vision target in frame then it will return [-1, -1]
     * 
     */
    public int[] getVisionTargetPosition() {
        Number[] targetPosition = visionTargetPosition.getNumberArray(defaultVector);
        int[] position = {(int)targetPosition[0], (int)targetPosition[1]};

        return position;
    }

    /**
     * Get the size of the current vision target
     * @return An Integer Array holding the X and Y lengths of the bounding box of the vision target, if there is no vision target in frame then the X and Y values will be -1
     */
    public int[] getVisionTargetSize() {
        Number[] targetSize = visionTargetSize.getNumberArray(defaultVector);
        int[] size = {(int)targetSize[0], (int)targetSize[1]};//Get the size into the correct format we can return

        return size;
    }

    /**
     * Get the horizontal offset of the camera to the vision target
     * @return An integer representing the horizontal offset
     */
    public int getVisionTargetHorizontalOffset() {
        return (int)visionTargetHorizontalOffset.getNumber(0);
    }

    /**
     * Get the type of the target that the camera sees
     * @return Returns an Enum indicating which type
     */
    public VisionTargetType getTargetType() {
        int targetTypeNumber = (int)visionTargetType.getNumber(0);
        VisionTargetType targetType = VisionTargetType.NONE;

        //Determine the target type
        switch(targetTypeNumber) {
            case 0:
                targetType = VisionTargetType.NONE;
                break;
            case 1:
                targetType = VisionTargetType.CARGO;
                break;
            case 2:
                targetType = VisionTargetType.REFLECTIVE_TAPE;
                break;
        }

        return targetType;
    }
}