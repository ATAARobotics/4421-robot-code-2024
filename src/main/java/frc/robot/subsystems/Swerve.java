package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.lib.config.SwerveModuleConstants;

public class Swerve extends SubsystemBase {
  private SwerveDrivePoseEstimator PoseEstimator;


  private final Pigeon2 gyro;

  private SwerveModule[] mSwerveMods;

  private Field2d field;

  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    gyro.configFactoryDefault();
    gyro.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);

    zeroGyro();

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    SwerveModulePosition[] positions = new SwerveModulePosition[] {
        mSwerveMods[0].getPostition(),
        mSwerveMods[1].getPostition(),
        mSwerveMods[2].getPostition(),
        mSwerveMods[3].getPostition(),
    };
    field = new Field2d();
    SmartDashboard.putData("Field", field);
    // Configure the AutoBuilder last
    
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.03, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ), () -> {return false;},
        this // Reference to this subsystem to set requirements
    );
    PoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), positions, new Pose2d(15.8, 8.0, getYaw()));
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }
  public void autoDrive(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
    }
  }
  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return PoseEstimator.getEstimatedPosition();
  }
  public void resetPose(Pose2d pose) {
    gyro.setYaw(pose.getRotation().getDegrees());
    PoseEstimator.resetPosition(getYaw(), getPositions(), pose);
  }

  public void resetOdometry(Pose2d pose) {
    PoseEstimator.resetPosition(getYaw(), this.getPositions(), pose);

  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getPostition();
    }
    return states;
  }
  public ChassisSpeeds getChassisSpeeds(){
    SwerveModuleState[] states = getStates();

    return Constants.Swerve.swerveKinematics.toChassisSpeeds(states[0], states[1], states[2], states[3]);
  }
  public void zeroGyro() {

    gyro.setYaw(0);
    System.out.println("gyro heading " + gyro.getYaw());
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
       ? Rotation2d.fromDegrees(360 - gyro.getYaw())
       : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {
    double[] pose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
    double poseX = -pose[0] + 8.27;
    double poseY = -pose[1] + 4.105;
    Rotation2d poseR = Rotation2d.fromDegrees(pose[5] - 180);
    double timeStamp = Timer.getFPGATimestamp() - (pose[6] / 1000.0);

    if (Math.abs(pose[0]) >= 0.1) {
      PoseEstimator.addVisionMeasurement(new Pose2d(poseX, poseY, poseR), timeStamp);
      //PoseEstimator.resetPosition(poseR, getPositions(), new Pose2d(poseX, poseY, poseR));
    }

    PoseEstimator.update(getYaw(), getPositions());

    field.setRobotPose(getPose());
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
        // mod.getAngleOffset().getDegrees() is used to add angle offset to canCoder values
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees() - mod.getAngleOffset().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Target Angle", mod.getTargetAngle().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond );
    }
  }
}
