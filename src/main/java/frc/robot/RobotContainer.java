// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private CANBus canivore1 = new CANBus("mechCAN");
  private CANBus canivore2 = new CANBus("swerveCAN");

  private final SendableChooser<Color> colorChooser = new SendableChooser<>();

  public enum Color {
    NONE,
    RED,
    GREEN,
    BLUE
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    colorChooser.setDefaultOption("OFF", Color.NONE);
    colorChooser.addOption("Red", Color.RED);
    colorChooser.addOption("Green", Color.GREEN);
    colorChooser.addOption("Blue", Color.BLUE);

    SmartDashboard.putData("Color Selector", colorChooser);
  }


}

