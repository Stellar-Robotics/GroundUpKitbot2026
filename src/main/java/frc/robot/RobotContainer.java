// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.TankSubsystem;
import frc.robot.subsystems.FuelSubsystemV2;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems are defined here...
  FuelSubsystemV2 fuelSubsystem = new FuelSubsystemV2();
  TankSubsystem driveSubsystem = new TankSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick leftJoystick = new CommandJoystick(0);
  CommandJoystick righJoystick = new CommandJoystick(1);
  CommandXboxController operatorController = new CommandXboxController(2);

  // Selector for autonomous
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // and subsystem command defaults
    configureBindings();

    // Configure auto chooser (Add pathplanner autos as options here)
    autoChooser.setDefaultOption("Sitting Duck", Commands.runOnce(() -> {}));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Select Auto", autoChooser);
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

    // Spin up and launch
    operatorController.rightTrigger().whileTrue(fuelSubsystem.spinUpAndLaunchCommand());
    // Intake fuel
    operatorController.pov(0).whileTrue(fuelSubsystem.intakeCommand());
    // Expel fuel
    operatorController.pov(180).whileTrue(fuelSubsystem.expelCommand());

    // Invert drivetrain
    leftJoystick.button(9).onTrue(driveSubsystem.flipsDriveCommand());
      
    SmartDashboard.putBoolean("robot reversed", driveSubsystem.isReversed);

    // this goes zoom zoom
    driveSubsystem.setDefaultCommand(
      driveSubsystem.driveTank(
        () -> MathUtil.applyDeadband(leftJoystick.getY(), .15), 
        () -> MathUtil.applyDeadband(righJoystick.getY(), .15), 
        true, 
        false
      )
    );


    // Robot barrier restriction (NULL POINTER CRASH 8/24/26)
    // double xRestriction = SmartDashboard.getNumber("WidthRestrictionsInMeters", 2.7432);
    // double yRestriction = SmartDashboard.getNumber("lengthRestrictionsInMeters", 2.7432);
    //   if(SmartDashboard.getBoolean("EnableBoundries", false)) {
    //     //zoomZoom.boundries(xRestriction, yRestriction);
    //   }
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
