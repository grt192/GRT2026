// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// frc imports
import frc.robot.controllers.PS5DriveController;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.commands.shooter.hood.hoodCommand;
// Subsystems
import frc.robot.subsystems.swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
// WPILib imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.ctre.phoenix6.CANBus;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private boolean isCompetition = true;

  private PS5DriveController driveController;
  // private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  CANBus c = new CANBus("mechCAN");
  private flywheel wheel = new flywheel(c);
  private hood hooded = new hood(c);
  private CommandPS5Controller gamer = new CommandPS5Controller(1);
  boolean manualModeShooter = true;
   private final HopperSubsystem hopp = new HopperSubsystem(c);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // constructDriveController(); 
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named f`actories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  
  private void configureBindings() {
    System.out.println("Configuring");

      //gun.run();
      /* Driving -- One joystick controls translation, the other rotation. If the robot-relative button is held down,
      * the robot is controlled along its own axes, otherwise controls apply to the field axes by default. If the
      * swerve aim button is held down, the robot will rotate automatically to always face a target, and only
      * translation will be manually controllable. */

      /* 
    swerveSubsystem.setDefaultCommand(
      new RunCommand(() -> {
        swerveSubsystem.setDrivePowers(
          driveController.getForwardPower(),
          driveController.getLeftPower(),
          driveController.getRotatePower()
        );
        }, 
        swerveSubsystem
      )
    );
    */
    Trigger exist = new Trigger(() -> 1==1);
    exist.whileTrue(new InstantCommand(() -> {
      Logger.recordOutput("DriverMode/", manualModeShooter);}));
      

    //Switch Mode
    gamer.circle().onTrue(new InstantCommand(() ->{ 
      manualModeShooter = !manualModeShooter; 
      wheel.dontShoot(); 
      hooded.hoodSpeed(0.0); 
    }));

    //Auto
    Trigger shooty = new Trigger(() -> gamer.getR2Axis() > -0.7 && manualModeShooter == false && DriverStation.isJoystickConnected(1));
    Trigger shootn = new Trigger(() -> gamer.getR2Axis() <- 0.7  && manualModeShooter == false);
    Trigger hoodAuto = new Trigger(() -> manualModeShooter == false);

    shooty.whileTrue(new RunCommand(() -> wheel.shoot(), wheel));
    shootn.onTrue(new InstantCommand(() -> wheel.dontShoot(), wheel));
    
    hoodAuto.whileTrue(new hoodCommand(hooded, wheel));

    Trigger hop = new Trigger(()-> gamer.R1().getAsBoolean());

    hop.onTrue(new InstantCommand(() -> hopp.spinAtTargetRPM(), hopp));
    
    hop.onFalse(new InstantCommand(() -> hopp.spinAtRPM(0), hopp));

    //Manual
    
    
    wheel.setDefaultCommand(Commands.run(() -> {
      if(manualModeShooter){
        if (DriverStation.isJoystickConnected(1)) {
          double speed = (gamer.getR2Axis() + 1) / 2;
          wheel.flySpeed(speed);
        } else{
          wheel.flySpeed(0);
        }
      }
    } , wheel));

    hooded.setDefaultCommand(Commands.run(() -> {
      if(manualModeShooter){
        System.out.println("manual");
        if (gamer.L3().getAsBoolean()) {
          hooded.hoodSpeed(0.1);
        }else if(gamer.R3().getAsBoolean()){
          hooded.hoodSpeed(-0.1);
        } else{
          hooded.hoodSpeed(0);
        }
      }
    }, hooded));
    
    /*driveController.getRelativeMode().whileTrue(
      new RunCommand(
        () -> {
          swerveSubsystem.setRobotRelativeDrivePowers(
            driveController.getForwardPower(),
            driveController.getLeftPower(),
            driveController.getRotatePower()
          );
          driveController.getRotatePower();
          }, swerveSubsystem)
    );
    */


    /* Pressing the button resets the field axes to the current robot axes. */
    /* 
    driveController.bindDriverHeadingReset(
      () ->{
        swerveSubsystem.resetDriverHeading();
      },
      swerveSubsystem
    );
    */
  }
    

  /**
   * Constructs the drive controller based on the name of the controller at port
   * 0
   */
  private void constructDriveController(){
    driveController = new PS5DriveController();
    driveController.setDeadZone(0.05);
  }


}
