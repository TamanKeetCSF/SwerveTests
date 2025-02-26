package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;

    public VisionSubsystem() {
        // "limelight" is the default table name; adjust if your Limelight has a different name
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /** @return whether the Limelight has any valid target (AprilTag) in view */
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    /** @return horizontal offset (tx in degrees) to the AprilTag (0 if no target) */
    public double getTagXOffset() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    /** @return vertical offset (ty in degrees) to the AprilTag (0 if no target) */
    public double getTagYOffset() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    // (Optional) If using 3D pose: get camera translation relative to tag (tx, ty, tz in meters)
    public double[] getCameraPoseTargetSpace() {
        return limelightTable.getEntry("camerapose_targetspace")
                             .getDoubleArray(new double[6]);
    }
}
