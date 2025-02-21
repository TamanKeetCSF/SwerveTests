package frc.robot.subsystems;

import com.revrobotics.SparkMax;
import com.revrobotics.MotorType;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;

public class SwerveModule extends SubsystemBase {
    
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final CANCoder encoder;
    private final PIDController steerPID;
    
    public SwerveModule(int driveMotorID, int steerMotorID, int encoderID) {
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);
        encoder = new CANCoder(encoderID);
        
      
        steerPID = new PIDController(SwerveConstants.STEER_P, SwerveConstants.STEER_I, SwerveConstants.STEER_D);
        steerPID.enableContinuousInput(-180, 180); 
    }
    
    public void setDesiredState(double speed, Rotation2d angle) {
        double currentAngle = encoder.getAbsolutePosition();
        double targetAngle = angle.getDegrees();
        double steerOutput = steerPID.calculate(currentAngle, targetAngle);
        
        driveMotor.set(speed / SwerveConstants.MAX_WHEEL_SPEED_MPS); 
        steerMotor.set(steerOutput);
    }
    
    public Rotation2d getModuleAngle() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }
    
    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
   }
}

