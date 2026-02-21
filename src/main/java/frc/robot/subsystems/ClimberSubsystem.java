// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorConstants;


public class ClimberSubsystem extends SubsystemBase {

  PneumaticHub revHub = new PneumaticHub(18);
  Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  Solenoid extensionSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  SparkMax climberMotor = new SparkMax(MotorConstants.climberCanID, MotorType.kBrushless);
  SparkClosedLoopController ClimberCLC = climberMotor.getClosedLoopController();
  Solenoid lockSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);

  //use set to turn on with bolean
  //off is locked for lock
  //off is retracted for extension

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
    
    climberMotorConfig.smartCurrentLimit(MotorConstants.currentLimit)
    .inverted(false)
    .closedLoop.pid(0, 0, 0);

    climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters );
  }

  
  
  public Command lock() {
    Command lockCommand = runOnce(()->{
      lockSolenoid.toggle();
    }
    );
    return lockCommand;
  }

  boolean extend = true;
  
  public Command extend() {
    Command extendCommand = runOnce(()->{
      extensionSolenoid.toggle();
    }
    );
    return extendCommand;
  }

  public Command activateClimbingMotor(double climberSetPoint) {
    double clampedClimberSetPoint = MathUtil.clamp(
      climberSetPoint,
      0,
      5
    );

    Command activateClimbingMotor = runOnce(()->{
    ClimberCLC.setSetpoint(clampedClimberSetPoint, ControlType.kPosition);
    }
    );
    return activateClimbingMotor;
  }

  public Command oneClimberSequence(double iSuckAtNamingStuff){
    Command oneClimberSequence = runOnce(()->{
      activateClimbingMotor(iSuckAtNamingStuff);
      lock();
      activateClimbingMotor(-iSuckAtNamingStuff);
    }
    );
    return oneClimberSequence;
  }

  public Command finalClimberingSequence(){

    Command finalClimberingSequence = runOnce(()->{
      lockSolenoid.set(ClimberConstants.unlocked);
      oneClimberSequence(ClimberConstants.firstClimb);
      oneClimberSequence(ClimberConstants.secondClimb);
      oneClimberSequence(ClimberConstants.thirdClimb);
    }
    );


    return finalClimberingSequence;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
