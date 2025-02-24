package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;


public class SwerveModule extends SubsystemBase {
    
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final CANcoder encoder;
    private final PIDController steerPID;
    
    public SwerveModule(int driveMotorID, int steerMotorID, int encoderID) {
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);
        encoder = new CANcoder(encoderID);
        
        steerPID = new PIDController(SwerveConstants.STEER_P, SwerveConstants.STEER_I, SwerveConstants.STEER_D);
        SmartDashboard.putData("SwerveSteerPID", steerPID);
        steerPID.enableContinuousInput(-180, 180); 
    }
    
    public void setDesiredState(SwerveModuleState desiredState) {
        var encoderRotation = new Rotation2d(encoder.getPosition().getValueAsDouble());
        desiredState.optimize(encoderRotation);

        desiredState.cosineScale(encoderRotation);

        double currentAngle = encoder.getAbsolutePosition().getValueAsDouble()*360; 
        System.out.println("posisición del encoder" + currentAngle);
        SmartDashboard.putNumber("CurrentAngle", currentAngle);
        double targetAngle = desiredState.angle.getDegrees();
        System.out.println("posisición objetivo" + targetAngle);
        SmartDashboard.putNumber("CurrentAngle", currentAngle);
        double error= currentAngle - targetAngle;
        double steerOutput = steerPID.calculate(currentAngle, targetAngle);

        System.out.println("PIDangulo" + steerOutput);
        
        
        driveMotor.set(desiredState.speedMetersPerSecond * Math.cos(error*Math.PI/180)*SwerveConstants.MAX_WHEEL_SPEED_MPS/10); 
        steerMotor.set(steerOutput*10);
    }
    
    public Rotation2d getModuleAngle() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    }
    
    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
   }
}

