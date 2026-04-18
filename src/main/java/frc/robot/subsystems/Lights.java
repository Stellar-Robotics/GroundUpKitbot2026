// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */

  public SendableChooser<Double> lightChooser;

  Spark light = new Spark(0);

  public Lights() {
    lightChooser = new SendableChooser<>();

    //solid colors
    lightChooser.addOption("red", 0.57);
    lightChooser.addOption("orange", 0.63);
    lightChooser.addOption("yellow", 0.69);
    lightChooser.addOption("lime green", 0.73);
    lightChooser.addOption("green", 0.77);
    lightChooser.addOption("aqua", 0.81);
    lightChooser.addOption("blue", 0.87);
    lightChooser.addOption("purple", 0.91);
    lightChooser.addOption("white", 0.93);
    lightChooser.addOption("black", 0.99);

    // //designs
    // lights.lightChooser.addOption("blue and yellow blinking", lights.blueAndYellow());
    // lights.lightChooser.addOption("faster blue and yellow blinking", lights.fasterBlueAndYellow());
    // lights.lightChooser.addOption("mixed blue and yellow blinking", lights.mixedBlueAndYellow());
    // lights.lightChooser.addOption("rainbow", lights.rainbow());
    // lights.lightChooser.addOption("red white and blue", lights.USAAAAAA());
    // lights.lightChooser.addOption("monochromatic", lights.monochromatic());
    // lights.lightChooser.addOption("gradiant", lights.gradiant());
    // lights.lightChooser.addOption("sparkle", lights.sparkle());

    SmartDashboard.putData("select lights", lightChooser);
  }

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


  /*the next ones im not exactly sure what they will do but
    im pretty sure they will include the 2 colors that are the primary and secondary*/
  
  public Command gradiant() {
    return runOnce(() -> light.set(0.41));
  }

  public Command sparkle() {
    return runOnce(() -> light.set(0.39));
  }


  public Command noLights() {
    return runOnce(() -> light.set(0));
  }
  
  // public Command lightCommand() {
  //   Command lightCommand;
  //   try {
  //     lightCommand = lightChooser.getSelected();
  //     lightCommand.setName("LightSelect");
  //   } catch (Exception e) {
  //     lightCommand = new Command() {};
  //     Logger.getGlobal().info("No Light Command!!");
  //   }
  //   return lightCommand;
  // }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
