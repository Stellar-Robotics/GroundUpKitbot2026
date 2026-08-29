// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.tankConstants;

public class TankSubsystem extends SubsystemBase {

  // Motor definition
  SparkMax tankFLMotor = new SparkMax(MotorConstants.fLCanID, MotorType.kBrushless);
  SparkMax tankFRMotor = new SparkMax(MotorConstants.fRCanID, MotorType.kBrushless);
  SparkMax tankBLMotor = new SparkMax(MotorConstants.bLCanID, MotorType.kBrushless);
  SparkMax tankBRMotor = new SparkMax(MotorConstants.bRCanID, MotorType.kBrushless);

  // Create closed loop controller refrences
  SparkClosedLoopController frontLeftCLC = tankFLMotor.getClosedLoopController();
  SparkClosedLoopController frontRightCLC = tankFRMotor.getClosedLoopController();

  // Robot sensing object definition
  ADIS16470_IMU gyro = new ADIS16470_IMU();
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(25));
  DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
    kinematics, 
    Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle() + 90, 360)), 
    -tankFLMotor.getEncoder().getPosition(), 
    -tankFRMotor.getEncoder().getPosition(), 
    new Pose2d(0, 5, Rotation2d.fromDegrees(0))
  );

  // Dashboard and visual feedback object definition
  Field2d field = new Field2d();
  public SendableChooser<Double> robotSpeed;

  // Drivetrain reversed
  public Boolean isReversed = false;


  public TankSubsystem() {

    // Setup and configure motors
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

    tankFLMotorConfig.encoder.positionConversionFactor(1 / tankConstants.RotationsInAMeter);
    tankFRMotorConfig.encoder.positionConversionFactor(1 / tankConstants.RotationsInAMeter);
    tankFLMotorConfig.closedLoop.pid(0.0002, 0.000001, 0.015);
    tankFRMotorConfig.closedLoop.pid(0.0002, 0.000001, 0.015);
    tankFRMotorConfig.closedLoop.feedForward.kV(0.002);
    tankFLMotorConfig.closedLoop.feedForward.kV(0.002);

    tankBLMotor.configure(tankBLMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    tankBRMotor.configure(tankBRMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    tankFLMotor.configure(tankFLMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    tankFRMotor.configure(tankFRMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Setup pathplanner and reset the gyro
    configurePathPlanner();
    gyro.reset();
    gyro.calibrate();

    // Setup and publish speeds presets to the dashboard
    robotSpeed = new SendableChooser<>();
    robotSpeed.addOption("normal speed", 1.0);
    robotSpeed.addOption("safer speed", 0.3);
    robotSpeed.addOption("super duper slow mode", 0.1);

    robotSpeed.setDefaultOption("normal speed", 1.0);

    SmartDashboard.putBoolean("Safer Speed", true);
    SmartDashboard.putData("robot speed", robotSpeed);
    SmartDashboard.putNumber("setPoint", 20);
  }


  // Drive the robot using a chassisspeeds object (x and rot value)
  public void chassisDrive(ChassisSpeeds chassisSpeedSupplier) {

    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeedSupplier);

    frontLeftCLC.setSetpoint(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity);
    frontRightCLC.setSetpoint(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity);
  }


  // Utility to convert direct wheel speeds to a chassis speeds object
  public ChassisSpeeds convertToChassisSpeeds(double leftSpeed, double rightSpeed) {
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
    return chassisSpeeds;
  }


  // Configure PathPlanner for auto configuration
  public void configurePathPlanner() {
    
    RobotConfig config;

    try{
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
        () -> poseEstimator.getEstimatedPosition(), 
        (poseReset) -> poseEstimator.resetPose(poseReset), 
        () -> convertToChassisSpeeds(tankFLMotor.getEncoder().getVelocity(), tankFRMotor.getEncoder().getVelocity()), 
        (chassisSpeeds, feedForwards) -> chassisDrive(chassisSpeeds),
        new PPLTVController(0), 
        config, 
        () -> {
          //checks to see if we are on the red alliance, if so the path will be flipped
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          else {
          return false;
          }
        }, 
          this
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

  }


  // Traditional tank drive method
  public Command driveTank(Supplier<Double> leftSpeed, Supplier<Double> rightSpeed, boolean squareInputs, boolean slowerRobot) {

    Command driveCommand = run(() -> {

      //double speedMultiplier = slowerRobot ? 0.3 : 1;

      double speedMultipliers = robotSpeed.getSelected();

      // boolean dashSpeedOverride = SmartDashboard.getBoolean("Safer Speed",true);
      // if (dashSpeedOverride == true) {
      //   speedMultiplier = 0.3;
      // } else{
      //   speedMultiplier = 1;
      // }

      if (squareInputs) { // Optionally square inputs (finer control at lower speeds)
        tankFRMotor.set(isReversed ? 
          MathUtil.copyDirectionPow(-rightSpeed.get(), 2) * speedMultipliers :
          MathUtil.copyDirectionPow(rightSpeed.get(), 2) * speedMultipliers);
        tankFLMotor.set(isReversed ? 
          MathUtil.copyDirectionPow(-leftSpeed.get(), 2) * speedMultipliers :
          MathUtil.copyDirectionPow(leftSpeed.get(), 2) * speedMultipliers);
      } else {
        tankFRMotor.set(isReversed ? -rightSpeed.get() * speedMultipliers : rightSpeed.get() * speedMultipliers);
        tankFLMotor.set(isReversed ? -leftSpeed.get() * speedMultipliers : leftSpeed.get() * speedMultipliers);
      }
    });

    return driveCommand;
  }


  public Command flipsDriveCommand() {return run(() -> isReversed = !isReversed);}


  
  public Command testOdometry() {
    Command test = runEnd(() -> {
      double setPoint = SmartDashboard.getNumber("setPoint", 20);
      poseEstimator.resetPose(new Pose2d(3.664, 4.067, Rotation2d.fromDegrees(180)));
      if(poseEstimator.getEstimatedPosition().getX() <= 7.0168) {
        chassisDrive(new ChassisSpeeds(setPoint, 0, 0));
      }
    }, () -> {
      chassisDrive(new ChassisSpeeds(0, 0, 0));
    }
    );
    return test;
  }



  // Constructs supplier condition to tell if robot is outside of restricted area
  public BooleanSupplier outOfBounds(double xRestriction, double yRestriction) {

    Supplier<List<Double>> robotPose = () -> Arrays.asList(poseEstimator.getEstimatedPosition().getX(), poseEstimator.getEstimatedPosition().getY());
    
    
    BooleanSupplier outOfBounds = () -> {
      if(robotPose.get().get(0) >= xRestriction 
      || robotPose.get().get(1) >= yRestriction 
      || robotPose.get().get(0) <= 0 
      || robotPose.get().get(1) <= 0) {    //returns true if the robot leaves the boundries
        return true;
      }
      else {
        return false;
      }
    };
    return outOfBounds;
  }


  // Reposition the rovot automatically when robot goes
  // outside of defined area.

  public Command boundries(double xRestriction, double yRestriction) {

    boolean outOfBounds = outOfBounds(xRestriction, yRestriction).getAsBoolean();
    //Pose2d centerPose2d = new Pose2d(xRestriction/2, yRestriction/2, Rotation2d.fromDegrees(0));
    double[] center = {xRestriction/2, yRestriction/2};
    Supplier<List<Double>> robotPose = () -> Arrays.asList(poseEstimator.getEstimatedPosition().getX(), poseEstimator.getEstimatedPosition().getY());

    Command boundries = run(() ->{
      if(outOfBounds) {
        Supplier<Double> robotAngle = () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        double targetAngle = 180-Math.atan(robotPose.get().get(0)-center[0]/robotPose.get().get(1)-center[1]);
        while(robotAngle.get() != targetAngle +- (tankConstants.speedOfTurn * 5)); {
          frontLeftCLC.setSetpoint(tankFLMotor.getEncoder().getPosition()+tankConstants.speedOfTurn, ControlType.kPosition);  //tune this
        }
        while(robotPose.get().get(0) >= xRestriction-2 
          || robotPose.get().get(1) >= yRestriction-2 
          || robotPose.get().get(0) <= 2 
          || robotPose.get().get(1) <= 2) {
            frontLeftCLC.setSetpoint(tankFLMotor.getEncoder().getPosition()+tankConstants.speed, ControlType.kPosition);
            frontRightCLC.setSetpoint(tankFLMotor.getEncoder().getPosition()+tankConstants.speed, ControlType.kPosition);
        }
      }
    }
    );
    return boundries;
  }


  @Override
  public void periodic() {
    poseEstimator.update(
      Rotation2d.fromDegrees(gyro.getAngle()), 
      -tankFLMotor.getEncoder().getPosition(), 
      -tankFRMotor.getEncoder().getPosition()
    );


    // Update pose estimation each loop and
    // ensure periodic feedback is updated.
    field.setRobotPose(poseEstimator.getEstimatedPosition());
    SmartDashboard.putData("Field", field);
    SmartDashboard.putNumber("GyroAngle", gyro.getAngle());
  }
}
