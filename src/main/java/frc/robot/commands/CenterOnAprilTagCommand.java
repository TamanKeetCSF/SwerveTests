package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class CenterOnAprilTagCommand extends Command {
    private final SwerveDriveSubsystem swerve;
    private final double targetDistance = 1.0; // inches
    private final double targetOffset = 0.0;   // center

    public CenterOnAprilTagCommand(SwerveDriveSubsystem subsystem) {
        this.swerve = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("CenterOnAprilTagCommand: Initializing (Aligning to tag center at 1 inch distance)");
    }

    @Override
    public void execute() {
        swerve.alignToAprilTag(targetDistance, targetOffset);
        System.out.printf("CenterOnAprilTagCommand: tx=%.2f, currentDist=%.2f, currentOffset=%.2f, fwdOut=%.2f, strafeOut=%.2f%n",
                swerve.getTx(), swerve.getCurrentDistance(), swerve.getCurrentOffset(),
                swerve.getLastForwardOutput(), swerve.getLastStrafeOutput());
    }

    @Override
    public boolean isFinished() {
        boolean aligned = swerve.isAlignedToTarget();
        if (aligned) {
            System.out.println("CenterOnAprilTagCommand: Alignment complete (within tolerance).");
        }
        return aligned;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0.0, 0.0, 0.0);
        if (interrupted) {
            System.out.println("CenterOnAprilTagCommand: Interrupted, stopping alignment.");
        } else {
            System.out.println("CenterOnAprilTagCommand: Completed successfully.");
        }
    }
}

