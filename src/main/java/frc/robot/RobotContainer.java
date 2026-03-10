// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Vision.VisionConstants;
// import frc.robot.Constants.VisionConstants;
import frc.robot.commands.allign.RotateToFieldAngleCommand;
// frc imports
import frc.robot.controllers.PS5DriveController;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
// Subsystems
import frc.robot.subsystems.swerve.SwerveSubsystem;
// import frc.robot.subsystems.Vision.VisionSubsystem;
// import frc.robot.subsystems.Vision.CameraConfig;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.shooter.towerRollers;

// import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
// import frc.robot.Constants.IntakeConstants;

// Commands
import frc.robot.commands.intake.ManualIntakePivotCommand;
import frc.robot.commands.shooter.rampDownFlywheel;
import frc.robot.commands.vision.GetCameraDisplacement;
import frc.robot.Constants.TowerConstants.TOWER_INTAKE;
import frc.robot.commands.ShooterSequence;

import com.ctre.phoenix6.CANBus;

// import frc.robot.commands.intake.SetIntakePivotCommand;
// import frc.robot.commands.hopper.HopperSetRPMCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.*;

// WPILib imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private PS5DriveController driveController;
    private CommandPS5Controller mechController;
    private final CANBus swerveCAN = new CANBus(Constants.Swerve_CAN_BUS);
    private final CANBus mechCAN = new CANBus(Constants.Mech_CAN_BUS);

    private SwerveSubsystem swerveSubsystem = Constants.SWERVE_ENABLED ? new SwerveSubsystem(swerveCAN) : null;
    private final FieldManagementSubsystem fmsSubsystem = new FieldManagementSubsystem();
    private towerRollers tower = new towerRollers(mechCAN);

    private final RollerIntakeSubsystem intakeSubsystem = new RollerIntakeSubsystem(mechCAN);
    // private final PivotIntakeSubsystem pivotIntake = new
    // PivotIntakeSubsystem(mechCAN);
    private final HopperSubsystem HopperSubsystem = new HopperSubsystem(mechCAN);
    private final Field2d m_field = new Field2d();
    private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem(mechCAN);
    private final flywheel flywheelSubsystem = new flywheel(mechCAN);
    private final hood hoodSubsystem = new hood(mechCAN);
    private boolean shootSeq = false;

    int rps = 0;
    int hoodAng = 0;
    int towerSpeed = 0;

    private final VisionSubsystem visionSubsystem1 = new VisionSubsystem(
        VisionConstants.cameraConfig11);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        SmartDashboard.putNumber("Shooter RPS", rps);
        SmartDashboard.putNumber("Hood Angle", hoodAng);
        SmartDashboard.putNumber("Tower Speed", towerSpeed);

        visionStuff();
        constructController();
        configureBindings();
        configureAutoChooser();

        CameraServer.startAutomaticCapture(); // start driver cam
        SmartDashboard.putData("Field", m_field);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named f`actories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private boolean mechEnabled = false;

    private void configureBindings() {
        /*
         * Driving -- One joystick controls translation, the other rotation. If the
         * robot-relative button is held down,
         * the robot is controlled along its own axes, otherwise controls apply to the
         * field axes by default. If the
         * swerve aim button is held down, the robot will rotate automatically to always
         * face a target, and only
         * translation will be manually controllable.
         */
        if (Constants.SWERVE_ENABLED && swerveSubsystem != null) {
            swerveSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    // L1 = boost mode (higher accel/velocity)
                    swerveSubsystem.setBoostMode(driveController.getLeftBumper());

                    // Left trigger controls speed limit: fully released = full speed, fully pressed
                    // = stopped
                    double leftTrigger = driveController.getLeftTriggerAxis();
                    double speedLimit = 1.0 - leftTrigger; // Invert so more trigger = slower
                    swerveSubsystem.setDriveSpeedLimit(speedLimit);

                    swerveSubsystem.setDrivePowers(
                        driveController.getForwardPower(),
                        driveController.getLeftPower(),
                        driveController.getRotatePower());
                },
                    swerveSubsystem));

            driveController.getRelativeMode().whileTrue(
                new RunCommand(
                    () -> {
                        swerveSubsystem.setRobotRelativeDrivePowers(
                            driveController.getForwardPower(),
                            driveController.getLeftPower(),
                            driveController.getRotatePower());
                        driveController.getRotatePower();
                    }, swerveSubsystem));

            /* Pressing the button resets the field axes to the current robot axes. */
            driveController.bindDriverHeadingReset(
                () -> {
                    swerveSubsystem.resetDriverHeading();
                },
                swerveSubsystem);
        }
        if (Constants.MECH_ENABLED) {

            // ==================== SHOOTER ====================
            // R2 = flywheel (analog speed control)
            // Left stick Y = hood manual control
            flywheelSubsystem.setDefaultCommand(Commands.run(() -> {
                rps = (int) SmartDashboard.getNumber("Shooter RPS", rps);
                flywheelSubsystem.shoot(rps);
            }, flywheelSubsystem));

            tower.setDefaultCommand(Commands.run(() -> {
                towerSpeed = (int) SmartDashboard.getNumber("Tower Speed", towerSpeed);
            }, tower));


            hoodSubsystem.setDefaultCommand(Commands.run(() -> {
                hoodAng = (int) SmartDashboard.getNumber("Hood Angle", hoodAng);
                hoodSubsystem.setHoodAngle(hoodAng);
            }, hoodSubsystem));

            // Swerve-dependent drive controller commands

        }

    }

    /**
     * Constructs the drive controller based on the name of the controller at port
     * 0
     */
    private void constructController() {
        driveController = new PS5DriveController();
        driveController.setDeadZone(0.035);
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

    // vision shit
    public void visionStuff() {
        visionSubsystem1.setInterface(swerveSubsystem::addVisionMeasurements);

        CommandScheduler.getInstance().schedule(
            new GetCameraDisplacement(visionSubsystem1,
                new Transform3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(-43 - 15),
                    Units.inchesToMeters(44.25),
                    new Rotation3d(0, 0, Math.PI / 2))));

    }

}
