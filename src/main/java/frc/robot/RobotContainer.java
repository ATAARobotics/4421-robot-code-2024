// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.HashMap;

import javax.print.attribute.standard.JobPrioritySupported;

import org.ejml.dense.block.MatrixOps_DDRB;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final Joystick joystick = new Joystick(0);
  /* Drive Controls */

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final double manualShootAxis = XboxController.Axis.kRightTrigger.value;
  private final boolean shootOveride = false;

  private final JoystickButton intakeButton = new JoystickButton(joystick, 6);
  private final JoystickButton shootOverride = new JoystickButton(joystick, 5);
  private final JoystickButton shooterButton = new JoystickButton(joystick, 3);
  private final JoystickButton zeroGyro = new JoystickButton(joystick, 8);
  private final JoystickButton armUpButton = new JoystickButton(joystick, 4);
  private final JoystickButton armDownButton = new JoystickButton(joystick, 1);
  private final JoystickButton ampButton = new JoystickButton(joystick, 7);
  // private final JoystickButton lobButton = new JoystickButton(joystick, 8);
  private final JoystickButton shooterIntakeButton = new JoystickButton(joystick, 2);
  private final JoystickButton driveStraightButton = new JoystickButton(joystick, 9);
  //private final JoystickButton ampButton = new JoystickButton(joystick, 7);


  //private final OI joysticks = new OI();

  

  /* Driver Buttons */
  /* Subsystems */
  public final Swerve s_Swerve;
  public final Shooter m_Shooter;
  public final Index m_Index;
  public final Intake m_Intake;
  public final IntakeCommand intake;
  public final Pivot mPivot;
  public final Lighting s_Lighting;
  public SendableChooser<Command> autoChooser;
  public Command AutoCommand;
  private Shooting shoot;
  private LobShot lobShot;
  private AutoShooter autoShoot;




  // temp

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() { 
    m_Index = new Index();
    m_Intake = new Intake();

    m_Shooter = new Shooter();
    mPivot = new Pivot();
    // Register pathplanner commands
    NamedCommands.registerCommand("Index", new InstantCommand(() ->  m_Index.runIndex(1)));
    NamedCommands.registerCommand("Stop Index", new InstantCommand(m_Index::stopIndex));

    s_Swerve = new Swerve();
    s_Lighting = new Lighting(s_Swerve, m_Intake);
    shoot = new Shooting(m_Shooter, mPivot, m_Index, s_Swerve, 
                        () -> joystick.getRawAxis(translationAxis),
                        () -> joystick.getRawAxis(strafeAxis), 
                        ()-> false);
    lobShot = new LobShot(m_Shooter, mPivot, m_Index, s_Swerve,
                        () -> joystick.getRawAxis(translationAxis),
                        () -> joystick.getRawAxis(strafeAxis),
                        () -> false);
    autoShoot = new AutoShooter(m_Shooter, mPivot, m_Index, s_Swerve);

    intake = new IntakeCommand(m_Intake, m_Index);

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> joystick.getRawAxis(translationAxis), // translation
            () -> joystick.getRawAxis(strafeAxis), // strafe
            () -> -joystick.getRawAxis(rotationAxis),
            () -> -joystick.getRawAxis(rotationAxis), // rotation
            ()->false
            ));


    NamedCommands.registerCommand("Intake", intake);
    NamedCommands.registerCommand("Fire Shooter", autoShoot);
    NamedCommands.registerCommand("4 note Shoot 1", new AutoShooterPreset(mPivot, 2.26, 5.53, this::getSide));
    NamedCommands.registerCommand("4 note Shoot 3", new AutoShooterPreset(mPivot, 3.11, 7.19, this::getSide));

    NamedCommands.registerCommand("5 note Shoot 1", new AutoShooterPreset(mPivot, 2.31, 4.53, this::getSide));
    NamedCommands.registerCommand("5 note Shoot 2", new AutoShooterPreset(mPivot, 2.16, 5.49 , this::getSide));
    NamedCommands.registerCommand("5 note Shoot 3", new AutoShooterPreset(mPivot, 3.36, 5.60, this::getSide));
    NamedCommands.registerCommand("5 note Shoot 4", new AutoShooterPreset(mPivot, 4.84, 6.07, this::getSide));
    NamedCommands.registerCommand("5 note Shoot 5", new AutoShooterPreset(mPivot, 4.77, 6.07, this::getSide));

    NamedCommands.registerCommand("3 note 7,8 Shoot 1", new AutoShooterPreset(mPivot, 2.25, 3.38, this::getSide));
    NamedCommands.registerCommand("3 note 7,8 Shoot 2", new AutoShooterPreset(mPivot, 2.24, 3.33, this::getSide));
    NamedCommands.registerCommand("3 note 7,8 Shoot 3", new AutoShooterPreset(mPivot, 2.22, 2.74, this::getSide));

    NamedCommands.registerCommand("3 note 6,7 Shoot all", new AutoShooterPreset(mPivot, 2.16, 2.29, this::getSide));
    NamedCommands.registerCommand("Shooter Down", new InstantCommand(() -> mPivot.toSetpoint(Constants.Subsystems.pivotMin)));
    NamedCommands.registerCommand("Abandon Path GOALPATH to ALTPATH", new AbandonPath().a_AbandonPath(
    () -> true, // whether we do abandon path, the boolean supplier will correlate to note/bot detection
    "Goal Path", "Alternate Path", s_Swerve));

    AutoBuilder.configureHolonomic(
        s_Swerve::getPose, // Robot pose supplier
        s_Swerve::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        s_Swerve::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        s_Swerve::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(7.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.5, 0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ), this::getSide,
        s_Swerve // Reference to this subsystem to set requirements
    );


    PPHolonomicDriveController.setRotationTargetOverride(() -> s_Swerve.getRotationTargetOverride());
    // Configure the button bindings
    // autoChooser = AutoBuilder.buildAutoChooser("Test two paths");
  

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    

    // SmartDashboard.putData("Auto Chooser", autoChooser);
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
    // /* Driver Buttons */
    // joysticks.intake.onTrue(new InstantCommand(() -> {m_Intake.runIntake(0.3); m_Index.runIndex(1);}));
    // joysticks.intake.onFalse(new InstantCommand(() -> {m_Intake.stopIntake(); m_Index.stopIndex();}));
    intakeButton.whileTrue(intake);
    //joysticks.intake.whileTrue(intake);
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    // shooterButton.onTrue(new InstantCommand(m_Shooter::AutoFire));

    ampButton.onTrue(new InstantCommand(() -> m_Shooter.scoreAmp(m_Index, mPivot)));


    //joysticks.reverseIntake.onTrue(new InstantCommand(() -> m_Shooter.scoreAmp(m_Index, mPivot)));
    shooterButton.onTrue(new InstantCommand(m_Shooter::Fire));

    

    shooterButton.whileTrue(shoot);
    shooterButton.onFalse(new TeleopSwerve(
            s_Swerve,
            ()->joystick.getRawAxis(translationAxis),
            ()->joystick.getRawAxis(strafeAxis),
            () ->-joystick.getRawAxis(rotationAxis),
            () -> -joystick.getRawAxis(rotationAxis), // rotation
            () ->false
            ));

    // lobButton.whileTrue(lobShot);
    // lobButton.onFalse(new TeleopSwerve(
    //         s_Swerve,
    //         ()->joystick.getRawAxis(translationAxis),
    //         ()->joystick.getRawAxis(strafeAxis),
    //         () ->-joystick.getRawAxis(rotationAxis),
    //         () -> -joystick.getRawAxis(rotationAxis), // rotation
    //         () ->false
    //         ));
    // joysticks.lobShot.whileTrue(lobShot)
    // .onFalse(new TeleopSwerve(
    //         s_Swerve,
    //         joysticks::getXVelocity,
    //         joysticks::getYVelocity,
    //         joysticks::getRotationVelocity,
    //         joysticks::getRotationUp, // rotation
    //         joysticks::getSlow
    //         ));
    // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    // joysticks.toWaypoint.whileTrue(new SequentialCommandGroup(
    //   // s_Swerve.driveToWaypoint(new Pose2d(((DriverStation.getAlliance().get()==DriverStation.Alliance.Blue)?(72.5/39.37):(578.77/39.37)), (323.00/39.37) - 2, Rotation2d.fromDegrees(90))),
    //   // new WaitCommand(0.2)
    //   new GetToAmp(s_Swerve, false)
    // ));

    shooterIntakeButton.onTrue(new InstantCommand(() -> {m_Shooter.ReverseIndex();m_Index.runIndex(-0.75);}))
      .onFalse(new InstantCommand(() -> {m_Shooter.stopIndex();m_Index.stopIndex();}));

    shootOverride.onTrue(new InstantCommand(() -> m_Index.runIndex(0.75))).onFalse(new InstantCommand(() -> m_Index.stopIndex()));


    armUpButton.onTrue(new InstantCommand(mPivot::PivotUp, mPivot)).onFalse(new InstantCommand(mPivot::stop, mPivot));
    armDownButton.onTrue(new InstantCommand(mPivot::PivotDown, mPivot)).onFalse(new InstantCommand(mPivot::stop, mPivot));

    //joysticks.ReallyOverrideShooter.onTrue(new InstantCommand(() -> m_Index.runIndex(1), m_Index)).onFalse(new InstantCommand(m_Index::stopIndex, m_Index));
    // joysticks.pivotGoSetpoint.onTrue(new InstantCommand(() -> mPivot.toSetpoint(45))).onFalse(new InstantCommand(mPivot::stop));
    driveStraightButton.whileTrue(new TeleopSwerve(
            s_Swerve,
            ()->joystick.getRawAxis(translationAxis),
            ()->joystick.getRawAxis(strafeAxis),
            () ->-joystick.getRawAxis(rotationAxis),
            () -> -joystick.getRawAxis(rotationAxis), // rotation
            () ->true
            ));

  }
  // public OI getOI() {
  //   return joysticks;
  // }
  public Swerve getSwerve(){
    return s_Swerve;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Boolean getSide(){
    // var alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()) {
    //   return alliance.get() == DriverStation.Alliance.Red;
    // }
    // return false;
    return true;
  }
  // public Swerve getSwerve(){
  //   return s_Swerve;
  // }
}
  