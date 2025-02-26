package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveSubsystem extends SubsystemBase {
    // Use the Pigeon IMU on port 0
    private final PigeonIMU gyro = new PigeonIMU(0);
    private boolean fieldOriented = true;  // Field-oriented drive mode toggle

    // Limelight vision NetworkTable entries (assuming table name "limelight")
    private final NetworkTable limelightTable;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry taEntry;
    private final NetworkTableEntry tvEntry;

    // PID controllers for alignment (forward/back and lateral)
    private final PIDController forwardController;
    private final PIDController strafeController;

    // Alignment tolerance (in inches)
    private static final double ALIGN_TOLERANCE = 1.0;

    // Stored measurements for logging
    private double lastDistance = 0.0;
    private double lastOffset = 0.0;
    private double lastTx = 0.0;
    private double lastForwardOutput = 0.0;
    private double lastStrafeOutput = 0.0;

    public SwerveDriveSubsystem() {
        // Initialize Limelight NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        txEntry = limelightTable.getEntry("tx");
        taEntry = limelightTable.getEntry("ta");
        tvEntry = limelightTable.getEntry("tv");

        // Initialize PID controllers (tune these values as needed)
        forwardController = new PIDController(0.1, 0.0, 0.0);
        strafeController = new PIDController(0.1, 0.0, 0.0);

        forwardController.setTolerance(ALIGN_TOLERANCE);
        strafeController.setTolerance(ALIGN_TOLERANCE);
    }

    /**
     * Align the robot to an AprilTag using vision.
     * @param targetDistance Desired distance from the tag (in inches).
     * @param targetOffset Desired lateral offset from tag center (in inches, positive = robot left of tag).
     */
    public void alignToAprilTag(double targetDistance, double targetOffset) {
        double tv = tvEntry.getDouble(0.0);
        if (tv < 1.0) {
            // No target visible; stop and log.
            drive(0.0, 0.0, 0.0);
            System.out.println("alignToAprilTag: No target visible");
            SmartDashboard.putString("Vision/Status", "No target");
            return;
        }

        // Read vision data
        double tx = txEntry.getDouble(0.0);
        double ta = taEntry.getDouble(0.0);

        // Estimate current distance using 'ta' as a rough proxy (calibration required)
        double currentDistance;
        if (ta > 0) {
            // For simplicity, assume an inverse square relationship.
            double k = targetDistance * Math.sqrt(ta);
            currentDistance = k / Math.sqrt(ta);
        } else {
            currentDistance = 0.0;
        }

        // Estimate lateral offset using tx (angle) and current distance
        double currentOffset = currentDistance * Math.tan(Math.toRadians(tx));

        lastTx = tx;
        lastDistance = currentDistance;
        lastOffset = currentOffset;

        // Compute PID outputs
        forwardController.setSetpoint(targetDistance);
        strafeController.setSetpoint(targetOffset);
        double forwardOutput = forwardController.calculate(currentDistance);
        double strafeOutput = strafeController.calculate(currentOffset);

        // Clamp outputs
        forwardOutput = MathUtil.clamp(forwardOutput, -1.0, 1.0);
        strafeOutput = MathUtil.clamp(strafeOutput, -1.0, 1.0);

        lastForwardOutput = forwardOutput;
        lastStrafeOutput = strafeOutput;

        // Command the drivetrain: forward, lateral (strafe) and zero rotation (alignment uses vision only)
        drive(forwardOutput, strafeOutput, 0.0, false);

        // Publish values to SmartDashboard and log to console
        SmartDashboard.putNumber("Vision/tx", tx);
        SmartDashboard.putNumber("Vision/ta", ta);
        SmartDashboard.putNumber("Vision/Distance", currentDistance);
        SmartDashboard.putNumber("Vision/Offset", currentOffset);
        SmartDashboard.putNumber("Align/ForwardOutput", forwardOutput);
        SmartDashboard.putNumber("Align/StrafeOutput", strafeOutput);

        System.out.printf("alignToAprilTag: tx=%.2f deg, dist=%.2f in, offset=%.2f in, fwdOut=%.2f, strafeOut=%.2f%n",
                          tx, currentDistance, currentOffset, forwardOutput, strafeOutput);
    }

    /** Drive method that accepts forward, strafe, and rotation commands with field-relative option. */
    public void drive(double forward, double strafe, double rotation, boolean fieldRelative) {
        double fwd = forward;
        double str = strafe;
        if (fieldRelative) {
            // Use the Pigeon IMU yaw for field-oriented control
            double angleRad = Math.toRadians(getYaw());
            double temp = fwd * Math.cos(angleRad) + str * Math.sin(angleRad);
            str = -fwd * Math.sin(angleRad) + str * Math.cos(angleRad);
            fwd = temp;
        }
        // Here you would convert (fwd, str, rotation) into module states and command your motors.
        // For demonstration, we simply log the drive values.
        SmartDashboard.putNumber("Drive/ForwardCmd", fwd);
        SmartDashboard.putNumber("Drive/StrafeCmd", str);
        SmartDashboard.putNumber("Drive/RotationCmd", rotation);
    }

    /** Overloaded drive method using current field-oriented mode. */
    public void drive(double forward, double strafe, double rotation) {
        drive(forward, strafe, rotation, fieldOriented);
    }

    /** Stop the drivetrain. */
    public void stop() {
        // Stop movement
        drive(0.0, 0.0, 0.0);
    }

    /**
     * Toggle between field-oriented and robot-oriented driving.
     */
    public void toggleFieldOriented() {
        fieldOriented = !fieldOriented;
        SmartDashboard.putBoolean("Drive/FieldOriented", fieldOriented);
        System.out.println("Field-oriented mode: " + (fieldOriented ? "ON" : "OFF"));
    }

    /** Returns the current yaw angle from the Pigeon IMU (in degrees). */
    public double getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return ypr[0];
    }

    /** Get the last measured horizontal angle (tx) from vision (in degrees). */
    public double getTx() {
        return lastTx;
    }

    /** Get the last estimated distance (inches) to the target. */
    public double getCurrentDistance() {
        return lastDistance;
    }

    /** Get the last estimated lateral offset (inches) from target center. */
    public double getCurrentOffset() {
        return lastOffset;
    }

    /** Get the last forward PID output. */
    public double getLastForwardOutput() {
        return lastForwardOutput;
    }

    /** Get the last strafe PID output. */
    public double getLastStrafeOutput() {
        return lastStrafeOutput;
    }

    /** Check if alignment is within tolerance (both distance and lateral offset). */
    public boolean isAlignedToTarget() {
        double tv = tvEntry.getDouble(0.0);
        if (tv < 1.0) {
            return false;
        }
        return forwardController.atSetpoint() && strafeController.atSetpoint();
    }
}
