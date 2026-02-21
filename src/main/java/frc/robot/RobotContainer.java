// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.VisionConstants;
// frc imports
import frc.robot.controllers.PS5DriveController;
<<<<<<< HEAD
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.commands.shooter.hood.hoodCommand;
// Subsystems
import frc.robot.subsystems.swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
=======


// Subsystems
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Vision.CameraConfig;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
// import frc.robot.Constants.IntakeConstants;

// Commands
import frc.robot.commands.intake.ManualIntakePivotCommand;

import com.ctre.phoenix6.CANBus;

// import frc.robot.commands.intake.SetIntakePivotCommand;
// import frc.robot.commands.hopper.HopperSetRPMCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;



>>>>>>> main
// WPILib imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.Commands;
=======
>>>>>>> main
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
  private PS5DriveController driveController;
  // private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  CANBus c = new CANBus("mechCAN");
  private flywheel wheel = new flywheel(c);
  private hood hooded = new hood(c);
  private CommandPS5Controller gamer = new CommandPS5Controller(1);
  boolean manualModeShooter = true;
   private final HopperSubsystem hopp = new HopperSubsystem(c);

  private final CANBus canivore = new CANBus("can");
  private final RollerIntakeSubsystem intakeSubsystem = new RollerIntakeSubsystem(canivore);
  private final PivotIntakeSubsystem pivotIntake = new PivotIntakeSubsystem();
  private final HopperSubsystem HopperSubsystem = new HopperSubsystem(canivore);
  private final Field2d m_field = new Field2d();

  private final VisionSubsystem visionSubsystem1 = new VisionSubsystem(
    VisionConstants.cameraConfigs[0]
  );
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
<<<<<<< HEAD
    // constructDriveController(); 
=======
    visionSubsystem1.setInterface(swerveSubsystem::addVisionMeasurements);

    constructDriveController();
    constructMechController();
>>>>>>> main
    configureBindings();
    configureAutoChooser();

    CameraServer.startAutomaticCapture(); // start driver cam
    SmartDashboard.putData("Field", m_field);
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

    Trigger hop = new Trigger(()-> gamer.getR2Axis() > -0.7);

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
<<<<<<< HEAD
    */
=======

    // --- Intake pivot set-position controls (commented out for now) ---
    // mechController.square().onTrue(
    //   new SetIntakePivotCommand(pivotIntake, IntakeConstants.STOWED_POS)
    // );
    // mechController.cross().onTrue(
    //   new SetIntakePivotCommand(pivotIntake, IntakeConstants.EXTENDED_POS)
    // );

  // circle for the manual hopper
    mechController.circle().whileTrue(
      new RunCommand(
        () -> HopperSubsystem.setManualControl(1.0),
        HopperSubsystem
      )
    ).onFalse(
      new InstantCommand(
        () -> HopperSubsystem.stop(),
        HopperSubsystem
      )
    );

    // --- Hopper RPM control (commented out for now) ---
    // mechController.triangle().onTrue(
    //   new HopperSetRPMCommand(HopperSubsystem)
    // );

    /* Intake Controls - Hold button to run rollers */
    // R1 - intake in
    mechController.R1().whileTrue(
      new RunCommand(
        () -> intakeSubsystem.setDutyCycle(Constants.IntakeConstants.ROLLER_IN_SPEED),
        intakeSubsystem
      )
    ).onFalse(
      new InstantCommand(
        () -> intakeSubsystem.stop(),
        intakeSubsystem
      )
    );

    // L1 - intake out
    mechController.L1().whileTrue(
      new RunCommand(
        () -> intakeSubsystem.setDutyCycle(Constants.IntakeConstants.ROLLER_OUT_SPEED),
        intakeSubsystem
      )
    ).onFalse(
      new InstantCommand(
        () -> intakeSubsystem.stop(),
        intakeSubsystem
      )
    );

     // Pivot Configs: R2 for pivot up and L2 for pivot down
        pivotIntake.setDefaultCommand(
    new ManualIntakePivotCommand(pivotIntake, () -> mechController.getR2Axis() - mechController.getL2Axis()
     )
   );



  }


  public void updateDashboard() {
    // Robot position
    Pose2d robotPose = swerveSubsystem.getRobotPosition();
    m_field.setRobotPose(robotPose);

    // Match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Intake
    // SmartDashboard.putString("Status/Intake State", getIntakeState());
    // SmartDashboard.putNumber("Status/Intake Angle", pivotIntake.getAngleDegrees());
    SmartDashboard.putBoolean("Status/Roller Active", isRollerActive());
    SmartDashboard.putBoolean("Status/At Top Limit", pivotIntake.isAtTopLimit());
    SmartDashboard.putBoolean("Status/At Bottom Limit", pivotIntake.isAtBottomLimit());

    // Vision + Autoalign 
    // VisionSubsystem is commented out rn because it's outdated
    SmartDashboard.putBoolean("Status/Target Detected", hasTarget());
    SmartDashboard.putBoolean("Status/Target Locked", isTargetLocked());
    SmartDashboard.putNumber("Status/Target Distance", getTargetDistance());
    SmartDashboard.putBoolean("Status/Auto Align Active", isAutoAlignActive());
  }

  // --- Intake state detection (commented out for now) ---
  // private String getIntakeState() {
  //   double angle = pivotIntake.getAngleDegrees();
  //   double tolerance = 0.05;
  //   if (Math.abs(angle - IntakeConstants.STOWED_POS) < tolerance) {
  //     return "STOWED";
  //   } else if (Math.abs(angle - IntakeConstants.EXTENDED_POS) < tolerance) {
  //     return "EXTENDED";
  //   } else {
  //     return "MOVING";
  //   }
  // }


  private boolean isRollerActive() {
    return intakeSubsystem.isRunning();
  }

  // TODO: Integrate w vision subsystem when its setup / enabled

  /**
   * Returns true if a target is detected by vision
   */
  private boolean hasTarget() {
    // TODO: check if AprilTag/target is detected
    // return visionSubsystem.hasTarget();
    return false;
  }

  /**
   * Returns true if target is locked (centered and stable)
   */
  private boolean isTargetLocked() {
    // TODO: Implement target lock 
    // Check if target is within tolerance and robot is aligned
    // return visionSubsystem.isTargetLocked();
    return false;
  }


  /**
   * Returns distance to target in meters
   */
  private double getTargetDistance() {
    // TODO: Get distance from vision subsystem
    // return visionSubsystem.getTargetDistance();
    return 0.0;
  }


  private boolean isAutoAlignActive() {
    // TODO: Check if auto-align command is running
    return false;
>>>>>>> main
  }
    

  /**
   * Constructs the drive controller based on the name of the controller at port
   * 0
   */
  private void constructDriveController(){
    driveController = new PS5DriveController();
    driveController.setDeadZone(0.05);
  }

<<<<<<< HEAD
=======
  /**
   * Constructs mech controller
   */
  private void constructMechController(){
    mechController = new CommandPS5Controller(1);
  }

  /**
   * Config the autonomous command chooser
   */
  private void configureAutoChooser() {
    // Add auton here
    autoChooser.setDefaultOption("Do Nothing", null);

    SmartDashboard.putData("Auto Selector", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
>>>>>>> main

}
 