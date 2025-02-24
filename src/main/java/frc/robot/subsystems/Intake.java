// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.SwerveConstants;


public class Intake extends SubsystemBase {
    /** Creates a new Elevator. */
    private final SparkMax m_arm;
    private final SparkMax m_intake;
    // Add encoders
    public Intake(int armMotorID, int intakeMotorID) {
        m_arm = new SparkMax(armMotorID, MotorType.kBrushless);
        m_intake = new SparkMax(intakeMotorID, MotorType.kBrushed); 
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    public void armSube() {
      m_arm.set(0.15);  
    }
  
    public void armBaja() {
        m_arm.set(-0.15);  
    }
  
    public void armStop() {
        m_arm.set(0.0);
    }

    public void intakeComer() {
        m_intake.set(0.15);  
    }

    public void intakeSacar() {
        m_intake.set(-0.15);  
    }

    public void intakeStop() {
        m_intake.set(0.0);  
    }
  
    // Method to get average motor speed
    public double getArmSpeed() {
        RelativeEncoder encoderArm = m_arm.getEncoder();
        return (encoderArm.getVelocity());
    }
}