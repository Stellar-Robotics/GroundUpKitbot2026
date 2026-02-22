// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.tankConstants;

public class TankSubsystem extends SubsystemBase {

  SparkMax tankFLMotor = new SparkMax(MotorConstants.fLCanID, MotorType.kBrushless);
  SparkMax tankFRMotor = new SparkMax(MotorConstants.fRCanID, MotorType.kBrushless);
  SparkMax tankBLMotor = new SparkMax(MotorConstants.bLCanID, MotorType.kBrushless);
  SparkMax tankBRMotor = new SparkMax(MotorConstants.bRCanID, MotorType.kBrushless);
  
  ADIS16470_IMU gyro = new ADIS16470_IMU();

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(25));

  DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
    kinematics, 
    new Rotation2d(gyro.getAngle()), 
    tankFLMotor.getEncoder().getVelocity(), 
    tankFRMotor.getEncoder().getVelocity(), 
    new Pose2d()
  );

  SparkClosedLoopController frontLeftCLC = tankFLMotor.getClosedLoopController();
  SparkClosedLoopController frontRightCLC = tankFRMotor.getClosedLoopController();

  public TankSubsystem() {
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

    tankFLMotorConfig.encoder.positionConversionFactor(tankConstants.RotationsInAMeter);
    tankFLMotorConfig.encoder.positionConversionFactor(tankConstants.RotationsInAMeter);
    tankFLMotorConfig.closedLoop.pid(0.01, 0, 0);
    tankFRMotorConfig.closedLoop.pid(0.01, 0, 0);

    tankBLMotor.configure(tankBLMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    tankBRMotor.configure(tankBRMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    tankFLMotor.configure(tankFLMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    tankFRMotor.configure(tankFRMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void chassisDrive(Supplier<ChassisSpeeds> chassisSpeedSupplier) {

    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeedSupplier.get());

    frontLeftCLC.setSetpoint(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity);
    frontRightCLC.setSetpoint(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity);
  }

  public ChassisSpeeds convertToChassisSpeeds(double leftSpeed, double rightSpeed) {
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
    return chassisSpeeds;
  }

  public void configurePathPlanner() {

    RobotConfig config;

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // AutoBuilder.configure(
    //   () -> poseEstimator.getEstimatedPosition(), 
    //   (poseReset) -> poseEstimator.resetPose(poseReset), 
    //   null, 
    //   null, 
    //   null, 
    //   config, 
    //   null, 
    //   null
    // );

  }


  @Override
  public void periodic() {
    poseEstimator.update(new Rotation2d(
      gyro.getAngle()), 
      tankFLMotor.getEncoder().getVelocity(), 
      tankFRMotor.getEncoder().getVelocity()
    );
  }
}
