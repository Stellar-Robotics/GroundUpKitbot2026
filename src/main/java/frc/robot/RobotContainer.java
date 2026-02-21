// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.FuelSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  FuelSubsystem ultimateDodgeBallMachine = new FuelSubsystem();
  ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  Drive zoomZoom = new Drive();
   // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandXboxController operatorController = new CommandXboxController(2);
  Joystick leftJoystick = new Joystick(0);
  Joystick righJoystick = new Joystick(1);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true

    //this shoots
    operatorController.rightTrigger().whileTrue(
      ultimateDodgeBallMachine.shootStuff()
    );

    //this intakes
    operatorController.leftBumper().whileTrue(
      ultimateDodgeBallMachine.intakeStuff()
    );

    //this drops
    operatorController.rightBumper().whileTrue(
      ultimateDodgeBallMachine.dropStuff()
    );
    /*
    operatorController.back().onTrue(
      climberSubsystem.finalClimberingSequence()
    );
    

    operatorController.start().onTrue(
      climberSubsystem.extend()
    );
    */

    Supplier<Double> leftJoystickInputFilter = () -> MathUtil.applyDeadband(leftJoystick.getY(), .15);
    Supplier<Double> rightJoystickInputFilter = () -> MathUtil.applyDeadband(righJoystick.getY(), .15);

    //this goes zoom zoom
    zoomZoom.setDefaultCommand( zoomZoom.driveTank(leftJoystickInputFilter, rightJoystickInputFilter) );


  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {};
  }
}
