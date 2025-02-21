package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDriveSubsystem extends SubsystemBase {
    
    private final SwerveModule frontLeft = new SwerveModule(
        SwerveConstants.FRONT_LEFT_DRIVE_MOTOR, 
        SwerveConstants.FRONT_LEFT_STEER_MOTOR, 
        SwerveConstants.FRONT_LEFT_CANCODER);
    
    private final SwerveModule frontRight = new SwerveModule(
        SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR, 
        SwerveConstants.FRONT_RIGHT_STEER_MOTOR, 
        SwerveConstants.FRONT_RIGHT_CANCODER);
    
    private final SwerveModule backLeft = new SwerveModule(
        SwerveConstants.BACK_LEFT_DRIVE_MOTOR, 
        SwerveConstants.BACK_LEFT_STEER_MOTOR, 
        SwerveConstants.BACK_LEFT_CANCODER);
    
    private final SwerveModule backRight = new SwerveModule(
        SwerveConstants.BACK_RIGHT_DRIVE_MOTOR, 
        SwerveConstants.BACK_RIGHT_STEER_MOTOR, 
        SwerveConstants.BACK_RIGHT_CANCODER);
    
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        SwerveConstants.FRONT_LEFT_POSITION,
        SwerveConstants.FRONT_RIGHT_POSITION,
        SwerveConstants.BACK_LEFT_POSITION,
        SwerveConstants.BACK_RIGHT_POSITION
    );
    
    public SwerveDriveSubsystem() {}
    
    public void drive(double xSpeed, double ySpeed, double rotation) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
            new edu.wpi.first.math.kinematics.ChassisSpeeds(xSpeed, ySpeed, rotation)
        );
        
        frontLeft.setDesiredState(states[0].speedMetersPerSecond, states[0].angle);
        frontRight.setDesiredState(states[1].speedMetersPerSecond, states[1].angle);
        backLeft.setDesiredState(states[2].speedMetersPerSecond, states[2].angle);
        backRight.setDesiredState(states[3].speedMetersPerSecond, states[3].angle);
    }
    
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}
