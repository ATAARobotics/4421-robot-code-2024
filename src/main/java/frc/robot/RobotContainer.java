// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.HashMap;

import org.ejml.dense.block.MatrixOps_DDRB;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  /* Drive Controls */
  //private final int translationAxis = XboxController.Axis.kLeftY.value;
  //private final int strafeAxis = XboxController.Axis.kLeftX.value;
  //private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final OI joysticks = new OI();

  

  /* Driver Buttons */
  /* Subsystems */
  public final Swerve s_Swerve;
  public final Shooter m_Shooter;
  public SendableChooser<Command> autoChooser;
  public Command AutoCommand;
  private Shooting shoot;




  // temp

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() { 
    m_Shooter = new Shooter();

    // Register pathplanner commands
    NamedCommands.registerCommand("Fire Shooter", new InstantCommand(() -> {m_Shooter.Fire();}));
    NamedCommands.registerCommand("Intake", new InstantCommand(m_Shooter::IntakeIn));
    NamedCommands.registerCommand("Index", new InstantCommand(m_Shooter::Index));
    NamedCommands.registerCommand("Stop Index", new InstantCommand(m_Shooter::stopIndex));

    s_Swerve = new Swerve(m_Shooter::hasNote);
    shoot = new Shooting(m_Shooter, s_Swerve,joysticks::getXVelocity,
        joysticks::getYVelocity);
    NamedCommands.registerCommand("Auto Shooter", new RunCommand(() -> {shoot.execute();}).onlyWhile(m_Shooter::hasNote));

    NamedCommands.registerCommand("abandon path", new AbandonPath().a_AbandonPath(() -> true, // we do abandon path
    "Goal Path", "Alternate Path", s_Swerve));
    AutoBuilder.configureHolonomic(
        s_Swerve::getPose, // Robot pose supplier
        s_Swerve::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        s_Swerve::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        s_Swerve::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.5, 0.01), // Translation PID constants
            new PIDConstants(5.0, 0, 0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ), () -> true,
        s_Swerve // Reference to this subsystem to set requirements
    );
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            joysticks::getXVelocity, // translation
            joysticks::getYVelocity, // strafe
            joysticks::getRotationVelocity // rotation
            ));

    // PPHolonomicDriveController.setRotationTargetOverride(s_Swerve::getRotationTargetOverride);
    // Configure the button bindings
    autoChooser = AutoBuilder.buildAutoChooser();
  

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    

    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    joysticks.intake.onTrue(new InstantCommand(m_Shooter::IntakeIn));
    joysticks.intake.onFalse(new InstantCommand(m_Shooter::StopIntake));

    joysticks.zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    joysticks.reverseIntake.onTrue(new InstantCommand(m_Shooter::ReverseIndex));
    joysticks.reverseIntake.onFalse(new InstantCommand(m_Shooter::stopIndex));
    joysticks.runShooter.onTrue(new InstantCommand(m_Shooter::Fire));
    joysticks.shooterLock.whileTrue(shoot)
    .onFalse(new TeleopSwerve(
            s_Swerve,
            joysticks::getXVelocity,
            joysticks::getYVelocity,
            joysticks::getRotationVelocity
            ));
    // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    joysticks.toWaypoint.whileTrue(s_Swerve.driveToWaypoint(new Pose2d(578.77/39.37, (323.00/39.37) - 1, Rotation2d.fromDegrees(270))));
  }
  public OI getOI() {
    return joysticks;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  // public Swerve getSwerve(){
  //   return s_Swerve;
  // }
}
  