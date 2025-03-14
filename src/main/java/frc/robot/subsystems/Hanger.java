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


public class Hanger extends SubsystemBase {
  /** Creates a new Elevator. */
  private final SparkMax m_hanger;
  // Add encoders
  public Hanger(int hangerMotorID) {
      m_hanger = new SparkMax(hangerMotorID, MotorType.kBrushless);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void hangerSube() {
    m_hanger.set(0.1);  
  }
  
  public void hangerBaja() {
    m_hanger.set(-0.1);
  }
  
  public void hangerStop() {
    m_hanger.set(0.0);
  }
  
  // Method to get average motor speed
  public double getHangerSpeed() {
    RelativeEncoder encoderHanger = m_hanger.getEncoder();
    return (encoderHanger.getVelocity());
  }
}