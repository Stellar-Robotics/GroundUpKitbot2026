// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  Spark light = new Spark(0);

  public void blinker(double color, double time) {
    light.set(0.85);
    new WaitCommand(4);
  }

  public Command blueAndYellow() {
    Command lights = runOnce(() -> {
      blinker(0.85, 4);
      blinker(0.69, 4);
    }
    );
    return lights;
  }

  public Command fasterBlueAndYellow() {
    Command lights = runOnce(() -> {
      blinker(0.85, 1);
      blinker(0.69, 1);
    });
    return lights;
  }

  public Command mixedBlueAndYellow() {
    Command lights = runOnce(() -> {
      light.set(.85);
      new WaitCommand(.5);
      light.set(.69);
      new WaitCommand(.5);
      light.set(.85);
      new WaitCommand(.5);
      light.set(.69);
      new WaitCommand(.5);
      light.set(.85);
      new WaitCommand(5);
      light.set(.69);
      new WaitCommand(5);
    });
    return lights;
  }

  public Command rainbow() {
    Command lights = run(() -> {
      blinker(0.57, 2);//redish
      blinker(0.63, 2);//orange
      blinker(0.69, 2);//yellow
      blinker(0.73, 2);//lime green
      blinker(0.77, 2);//green
      blinker(0.81, 2);//aqua
      blinker(0.87, 2);//blue
      blinker(0.91, 2);//violet
    }
    );
    return lights;
  }

  //all of the Commands below are just solid colors
  public Command red() {
    return runOnce(() -> light.set(0.61));
  }

  public Command orange() {
    return runOnce(() -> light.set(0.63));
  }

  public Command yellow() {
    return runOnce(() -> light.set(0.69));
  }

  public Command green() {
    return runOnce(() -> light.set(0.77));
  }

  public Command blue() {
    return runOnce(() -> light.set(0.87));
  }

  public Command purple() {
    return runOnce(() -> light.set(0.91));
  }

  public Command white() {
    return runOnce(() -> light.set(0.93));
  }

  public Command black() {
    return runOnce(() -> light.set(0.99));
  }



  public Command USAAAAAA() {
    Command lights = runOnce(() -> {
      blinker(0.93, 3);//white
      blinker(0.61, 3);//red
      blinker(0.87, 3);//blue
    }
    );
    return lights;
  }

  public Command monochromatic() {
    Command lights = runOnce(() -> {
      blinker(0.93, 2);
      blinker(0.95, 1);
      blinker(0.97, 1);
      blinker(0.99, 2);
      blinker(0.97, 1);
      blinker(0.95, 1);
    }
    );
    return lights;
  }

  




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
