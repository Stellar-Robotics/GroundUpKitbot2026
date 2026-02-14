// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelSubsystem extends SubsystemBase {

  SparkMax dualFuelMotor = new SparkMax(12, MotorType.kBrushless);

  /** Creates a new FuelSubsystem. */
  public FuelSubsystem() {
    SparkMaxConfig dualFuelMotorConfig = new SparkMaxConfig();

    dualFuelMotorConfig.smartCurrentLimit(60);
    dualFuelMotor.configure(dualFuelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  
  public void intake() {
    dualFuelMotor.setVoltage(8.0);
  }

  public void StopIntake() {
    dualFuelMotor.setVoltage(.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
