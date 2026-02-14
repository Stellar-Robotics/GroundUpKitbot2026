// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Drive extends SubsystemBase {
  
  SparkMax tankFLMotor = new SparkMax(MotorConstants.fLCanID, MotorType.kBrushless);
  SparkMax tankFRMotor = new SparkMax(MotorConstants.fRCanID, MotorType.kBrushless);
  SparkMax tankBLMotor = new SparkMax(MotorConstants.bLCanID, MotorType.kBrushless);
  SparkMax tankBRMotor = new SparkMax(MotorConstants.bRCanID, MotorType.kBrushless);


  public Drive() {
    
  SparkMaxConfig tankFLMotorConfig = new SparkMaxConfig();
  SparkMaxConfig tankFRMotorConfig = new SparkMaxConfig();
  SparkMaxConfig tankBLMotorConfig = new SparkMaxConfig();
  SparkMaxConfig tankBRMotorConfig = new SparkMaxConfig();

  tankFLMotorConfig.smartCurrentLimit(MotorConstants.currentLimit)
    .inverted(true);
  tankFRMotorConfig.smartCurrentLimit(MotorConstants.currentLimit)
    .inverted(false);
  tankBLMotorConfig.smartCurrentLimit(MotorConstants.currentLimit)
    .inverted(true)
    .follow(MotorConstants.fLCanID);
  tankBRMotorConfig.smartCurrentLimit(MotorConstants.currentLimit)
    .inverted(false)
    .follow(MotorConstants.fRCanID);

  tankBLMotor.configure(tankBLMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  tankBRMotor.configure(tankBRMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  tankFLMotor.configure(tankFLMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  tankFRMotor.configure(tankFRMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }



  public Command driveTank(Supplier<Double> leftSpeed, Supplier<Double> rightSpeed) {


    Command driveCommand = run(() -> {
      tankFRMotor.set(rightSpeed.get());
      tankFLMotor.set(leftSpeed.get());
    });

    return driveCommand;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
