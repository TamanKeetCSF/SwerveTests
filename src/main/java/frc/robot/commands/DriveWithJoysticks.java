package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveWithJoysticks extends Command {
    private final SwerveDriveSubsystem swerve;
    private final XboxController controller;
    private static final double DEADBAND = 0.05;

    public DriveWithJoysticks(SwerveDriveSubsystem subsystem, XboxController controller) {
        this.swerve = subsystem;
        this.controller = controller;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double forward = -controller.getLeftY();
        double strafe = -controller.getLeftX();
        double rotation = -controller.getRightX();

        forward = Math.abs(forward) > DEADBAND ? forward : 0.0;
        strafe = Math.abs(strafe) > DEADBAND ? strafe : 0.0;
        rotation = Math.abs(rotation) > DEADBAND ? rotation : 0.0;

        swerve.drive(forward, strafe, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
