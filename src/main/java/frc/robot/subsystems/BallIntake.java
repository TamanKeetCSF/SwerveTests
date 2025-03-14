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


public class BallIntake extends SubsystemBase {
    /** Creates a new Elevator. */
    private final SparkMax m_marco;
    private final SparkMax m_ballIntake;
    // Add encoders
    public BallIntake(int marcoMotorID, int ballIntakeMotorID) {
        m_marco = new SparkMax(marcoMotorID, MotorType.kBrushless);
        m_ballIntake = new SparkMax(ballIntakeMotorID, MotorType.kBrushed);
    }
  
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
  
    public void marcoSube() {
        m_marco.set(0.15);  
    }
  
    public void marcoBaja() {
        m_marco.set(-0.15);  
    }
  
    public void marcoStop() {
        m_marco.set(0.0);
    }

    public void ballIntakeComer() {
        m_ballIntake.set(0.4);  
    }

    public void ballIntakeSacar() {
        m_ballIntake.set(-0.4);  
    }

    public void ballIntakeStop() {
        m_ballIntake.set(0.0);  
    }
    // Method to get average motor speed
    public double getMarcoSpeed() {
        RelativeEncoder encoderMarco = m_marco.getEncoder();
        return (encoderMarco.getVelocity());
    }
}