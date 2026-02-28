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


  //this extends the climbing device
  //off/false is retracted
  Solenoid extensionSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

  //this motor is the motor that does the climbing
  SparkMax climberMotor = new SparkMax(MotorConstants.climberCanID, MotorType.kBrushless);
  SparkClosedLoopController ClimberCLC = climberMotor.getClosedLoopController();

  //this motor locks the climber in place after compleating the climb to each rung
  //off/false is locked
  Solenoid lockSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);


  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
    
    climberMotorConfig.smartCurrentLimit(MotorConstants.currentLimit)
    .inverted(false)
    .closedLoop.pid(0, 0, 0);                                        //change these

    climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters );
  }

  
  //creates a command to toggle the locking motor
  public Command lock() {
    Command lockCommand = runOnce(()->{
      lockSolenoid.toggle();
    }
    );
    return lockCommand;
  }

  //creates a command to toggle the extendsion motor
  public Command extend() {
    Command extendCommand = runOnce(()->{
      extensionSolenoid.toggle();
    }
    );
    return extendCommand;
  }

  //creates a command to set the position of the climbing motor
  public Command climbCommand(double climberSetPoint) {
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

  //creates a command to climb one rung of the latter
  public Command oneClimberSequence(double iSuckAtNamingStuff){
    Command oneClimberSequence = runOnce(()->{
      climbCommand(iSuckAtNamingStuff);
      lock();
      climbCommand(-iSuckAtNamingStuff);
    }
    );
    return oneClimberSequence;
  }

  //creates a command to do the full climb at the end of teleop
  public Command finalClimberingSequence(){

    Command finalClimberingSequence = runOnce(()->{
      lockSolenoid.set(ClimberConstants.unlocked);
      oneClimberSequence(ClimberConstants.firstClimb);
      lockSolenoid.set(ClimberConstants.unlocked);
      oneClimberSequence(ClimberConstants.secondClimb);
      lockSolenoid.set(ClimberConstants.unlocked);
      oneClimberSequence(ClimberConstants.thirdClimb);
    }
    //this is a safe gaurd so this command won't run unless the extension is extended
    ).onlyIf(()->extensionSolenoid.get() == true);


    return finalClimberingSequence;
  }

  public Command autoClimbingSeqCommand() {
    Command autoClimbingSeqCommand = runOnce(()->{
      lockSolenoid.set(true);          //unlocks locking thingy
      extend();
      climbCommand(ClimberConstants.autoClimb);
      lockSolenoid.set(false);

    }
    );
    return autoClimbingSeqCommand;
  }

  public Command endOfAutoClimb() {
    Command endOfAutoClimb = runOnce(()->{
      climbCommand(ClimberConstants.autoClimb);
      lockSolenoid.set(ClimberConstants.unlocked);
      extensionSolenoid.set(false);   
    }
    );
    return endOfAutoClimb;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
