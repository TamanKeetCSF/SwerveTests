package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveRightOfAprilTagCommand extends Command{
    private final SwerveDriveSubsystem swerve;
    private final double targetDistance = 1.0; // inches
    private final double targetOffset = -5.0;  // 5 inches right of tag

    public MoveRightOfAprilTagCommand(SwerveDriveSubsystem subsystem) {
        this.swerve = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("MoveRightOfAprilTagCommand: Initializing (Aligning 5 inches right of tag at 1 inch distance)");
    }

    @Override
    public void execute() {
        swerve.alignToAprilTag(targetDistance, targetOffset);
        System.out.printf("MoveRightOfAprilTagCommand: tx=%.2f, currentDist=%.2f, currentOffset=%.2f, fwdOut=%.2f, strafeOut=%.2f%n",
                swerve.getTx(), swerve.getCurrentDistance(), swerve.getCurrentOffset(),
                swerve.getLastForwardOutput(), swerve.getLastStrafeOutput());
    }

    @Override
    public boolean isFinished() {
        boolean aligned = swerve.isAlignedToTarget();
        if (aligned) {
            System.out.println("MoveRightOfAprilTagCommand: Alignment reached (within tolerance).");
        }
        return aligned;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0.0, 0.0, 0.0);
        if (interrupted) {
            System.out.println("MoveRightOfAprilTagCommand: Interrupted, stopping alignment.");
        } else {
            System.out.println("MoveRightOfAprilTagCommand: Completed successfully.");
        }
    }
}

