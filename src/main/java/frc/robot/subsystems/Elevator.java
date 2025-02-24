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


public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final SparkMax m_elevator;
  // Add encoders
  public Elevator(int elevatorMotorID) {
    m_elevator = new SparkMax(elevatorMotorID, MotorType.kBrushless);   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void elevatorSube() {
    m_elevator.set(0.15);  
  }

  public void elevatorBaja() {
    m_elevator.set(-0.15);  
  }

  public void elevatorStop() {
    m_elevator.set(0.0);  
  }

  // Method to get average motor speed
  public double getElevatorSpeed() {
      RelativeEncoder encoderElevador = m_elevator.getEncoder();
      return (encoderElevador.getVelocity());
  }
}