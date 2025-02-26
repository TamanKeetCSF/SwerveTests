package frc.robot;

import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.commands.CenterOnAprilTagCommand;
import frc.robot.commands.MoveLeftOfAprilTagCommand;
import frc.robot.commands.MoveRightOfAprilTagCommand;
import frc.robot.commands.DriveWithJoysticks;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    private final XboxController driverController = new XboxController(0);

    private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(swerveDriveSubsystem, driverController);
    private final CenterOnAprilTagCommand centerOnTag = new CenterOnAprilTagCommand(swerveDriveSubsystem);
    private final MoveLeftOfAprilTagCommand moveLeftOfTag = new MoveLeftOfAprilTagCommand(swerveDriveSubsystem);
    private final MoveRightOfAprilTagCommand moveRightOfTag = new MoveRightOfAprilTagCommand(swerveDriveSubsystem);

    public RobotContainer() {
        swerveDriveSubsystem.setDefaultCommand(driveWithJoysticks);
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(driverController, XboxController.Button.kX.value)
            .onTrue(centerOnTag);
        new JoystickButton(driverController, XboxController.Button.kY.value)
            .onTrue(moveLeftOfTag);
        new JoystickButton(driverController, XboxController.Button.kB.value)
            .onTrue(moveRightOfTag);
        new JoystickButton(driverController, XboxController.Button.kStart.value)
            .onTrue(new InstantCommand(() -> swerveDriveSubsystem.toggleFieldOriented()));
    }
}



