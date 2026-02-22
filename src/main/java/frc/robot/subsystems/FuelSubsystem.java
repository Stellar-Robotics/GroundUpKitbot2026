// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MotorConstants;

public class FuelSubsystem extends SubsystemBase {

  SparkMax dualFuelMotor = new SparkMax(12, MotorType.kBrushless);
  SparkMax kickerMotor = new SparkMax(13, MotorType.kBrushless);

  /** Creates a new FuelSubsystem. */
  public FuelSubsystem() {
    SparkMaxConfig dualFuelMotorConfig = new SparkMaxConfig();
    SparkMaxConfig kickerMotorConfig = new SparkMaxConfig();

    dualFuelMotorConfig.smartCurrentLimit(MotorConstants.currentLimit).inverted(false);
    kickerMotorConfig.smartCurrentLimit(MotorConstants.currentLimit).inverted(false);


    kickerMotor.configure(kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    dualFuelMotor.configure(dualFuelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  

  public Command intakeStuff() {

    Command intakeStuff = runEnd(()->{
    dualFuelMotor.setVoltage(5);
    kickerMotor.setVoltage(5);
    },()->{
    dualFuelMotor.setVoltage(0);
    kickerMotor.setVoltage(0);

    }
    );
    return intakeStuff;
  }


  public Command dropStuff() {

    Command dropStuff = runEnd(()->{
    dualFuelMotor.setVoltage(-5);
    kickerMotor.setVoltage(-5);
    },()->{
    dualFuelMotor.setVoltage(0);
    kickerMotor.setVoltage(0);

    }
    );
    return dropStuff;
  }

    public Command shootStuff() {

      Command SpinUpShooter = runOnce(() -> {
        dualFuelMotor.setVoltage(8);
      });

      Command runWeeeMotor = run(() -> {
        kickerMotor.setVoltage(-8);
      });


      Command rnWeeeMotor = runOnce(()->{
        kickerMotor.setVoltage(8);
      }
      );

      Command commandSeq = new SequentialCommandGroup(
        SpinUpShooter,
        rnWeeeMotor,
        new WaitCommand(1),
        runWeeeMotor
      ).handleInterrupt(() -> {
        dualFuelMotor.setVoltage(0);
        kickerMotor.setVoltage(0);
      });

    return commandSeq;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
