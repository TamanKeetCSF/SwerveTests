package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix.sensors.PigeonIMU;

public class SwerveModuleTesting extends SubsystemBase {
    
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
    
    private final PigeonIMU gyro = new PigeonIMU(0);
    
    private boolean fieldOriented = false;
    
    public SwerveModuleTesting() {}

    /**
     * Habilitar o dehabilitar field.oriented
     * @param fieldOriented verdadero en field-oriented, falso robot-oriented
     */
    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }
    
    public void toggleFieldOriented() {
        fieldOriented = !fieldOriented;
    }
    
    public boolean isFieldOriented() {
        return fieldOriented;
    }
    
    public void drive(double xSpeed, double ySpeed, double rotation) {
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented) {
            double yaw = gyro.getYaw();
            Rotation2d gyroAngle = Rotation2d.fromDegrees(yaw);
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, gyroAngle);
        } else {
            chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, rotation);
        }
//AQUÍ SE ELIGE EL MÓDULO QUE SE DESEA PROBAR
//LOS ESTADOS VAN DE ACUERDO AL MÓDULO
//FrontLeft 0
//FrontRight 1
//BackLeft  2
//BackRight 3


        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        backLeft.setDesiredState(states[2]);
    }
    
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}

