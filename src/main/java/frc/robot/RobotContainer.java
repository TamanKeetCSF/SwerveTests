package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
    
    private final XboxController driverController = new XboxController(0);
    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    
    public RobotContainer() {
        swerveDriveSubsystem.setDefaultCommand(new DriveWithJoysticks(swerveDriveSubsystem, driverController));
        
        configureButtonBindings();
    }
    
    public Command getAutonomousCommand() {
        return null;
    }
    
    private void configureButtonBindings() {

      final JoystickButton buttonA = new JoystickButton(driverController, 1);

        // Cambia entre field oriented y robot oriented usando el botón A 

        buttonA.onTrue(new InstantCommand(() -> swerveDriveSubsystem.toggleFieldOriented()));
        
    }
}


