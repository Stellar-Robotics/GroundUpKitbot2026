// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.MotorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.MechanismConstants;

public class FuelSubsystemV2 extends SubsystemBase {

  /* ------------------------------------------------------
   * Initialization (Setup code)
   * ------------------------------------------------------ */

  // Create motor objects (UPDATE CAN IDs!)
  private SparkMax intakeSpark = new SparkMax(34, MotorType.kBrushless);
  private SparkMax feederSpark = new SparkMax(36, MotorType.kBrushless);
  private SparkFlex launcherFlex = new SparkFlex(35, MotorType.kBrushless);

  // Get launcher motor CLC for percision speed control
  private SparkClosedLoopController launcherCLC = launcherFlex.getClosedLoopController();

  // Set intake speed
  // private double intakeSpeedPercentage = 0.5;
  // private double feederSpeedPercentage = 0.5;
  // private int shooterSpeedRPM = 2700;
  // private double spinUpTimeSeconds = 1.5;


  public FuelSubsystemV2() {

    // Create motor configs
    var intakeSparkConf = new SparkMaxConfig();
    var feederSparkConf = new SparkMaxConfig();
    var launcherFlexConf = new SparkFlexConfig();

    // Configure motor configs
    intakeSparkConf
      .smartCurrentLimit(MotorConstants.weakCurrentLimit)
      .inverted(false);
    feederSparkConf
      .smartCurrentLimit(MotorConstants.strongCurrentLimit)
      .inverted(true);
    launcherFlexConf
      .smartCurrentLimit(MotorConstants.strongCurrentLimit)
      .inverted(true)
      .closedLoop.pid(0.0025, 0, 0.0025);

    // Apply motor configs to motors
    intakeSpark.configure(
      intakeSparkConf, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters);
    feederSpark.configure(
      feederSparkConf, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters);
    launcherFlex.configure(
      launcherFlexConf, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters);
  }


  /* ------------------------------------------------------
   * Standard methods for operating subsystem
   * ------------------------------------------------------ */

  // Set intake and feeder motors at desired speed percentage
  public void setIntake(double speedPercentage) {
    intakeSpark.set(speedPercentage);
    feederSpark.set(-speedPercentage);
  }


  // Spin's shooter up to desired speed
  public void setShooterRPM(int speedRPM) {
    launcherCLC.setSetpoint(
      MathUtil.clamp(speedRPM, 0, 4000),
      ControlType.kVelocity
    );
  }


  // Set feeder at desired speed
  public void setFeeder( double speedPercentage) { feederSpark.set(speedPercentage); }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  /* ------------------------------------------------------
   * Command methods used by command based callers
   * ------------------------------------------------------ */

  // Intake and expel commands at preset speed
  public Command intakeCommand() {
    return runEnd(
      () -> setIntake(MechanismConstants.intakeSpeedPercentage), 
      () -> setIntake(0)
    ).withName("IntakeFuel");
  }

  public Command expelCommand() {
    return runEnd(
      () -> setIntake(-MechanismConstants.intakeSpeedPercentage),
      () -> setIntake(0)
    ).withName("ExpelFuel");
  }


  // Primary command for spinning up and launching fuel
  public Command spinUpAndLaunchCommand() {
    // Build a command composition
    Command com = new SequentialCommandGroup(
      runOnce(() -> setShooterRPM(MechanismConstants.shootSpeedRPM)), // Spin up
      runOnce(() -> setFeeder(-MechanismConstants.feedSpeedPercentage)), // Run feeder backwards initally
      runOnce(() -> setIntake(MechanismConstants.feedSpeedPercentage)), // Run intake to prevent fuel from leaving
      new WaitCommand(MechanismConstants.spinUpTimeSeconds), // Wait for certin time
      runOnce(() -> setFeeder(MechanismConstants.feedSpeedPercentage)), // Run feeder forwards
      new WaitUntilCommand(() -> false)
    )
    .handleInterrupt(() -> {
      // Stop shooter and feeder when inturrupted
      setShooterRPM(0);
      setFeeder(0);
      setIntake(0);
    })
    .withName("SpinUpAndLaunch");

    return com;
  }
}
