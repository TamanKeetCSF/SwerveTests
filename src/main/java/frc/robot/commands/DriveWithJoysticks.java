package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class DriveWithJoysticks extends Command {
    
    private final SwerveDriveSubsystem swerveDrive;
    private final XboxController controller;
    
    public DriveWithJoysticks(SwerveDriveSubsystem swerveDrive, XboxController controller) {
        this.swerveDrive = swerveDrive;
        this.controller = controller;
        addRequirements(swerveDrive);
    }
    
    @Override
    public void execute() {
        double xSpeed = -controller.getLeftY(); 
        double ySpeed = -controller.getLeftX(); 
        double rotation = -controller.getRightX(); 

        double leftTrigger = controller.getLeftTriggerAxis(); 
        if (leftTrigger > 0.1) {
            double slowModeFactor = 0.5; 
            xSpeed *= slowModeFactor;
            ySpeed *= slowModeFactor;
            rotation *= slowModeFactor;
        }
        
        swerveDrive.drive(xSpeed, ySpeed, rotation);
    }
    
    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}