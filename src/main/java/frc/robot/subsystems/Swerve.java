package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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

  private double[] pose;

  private final Pigeon2 gyro;

  private SwerveModule[] mSwerveMods;

  private Field2d field;

  private Pose2d vecPose;
  private Pose2d lastPose;
  private double lastTimeStamp = 0;

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
    

    PoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), positions, new Pose2d(15.8, 8.0, getYaw()));
  
    lastPose = PoseEstimator.getEstimatedPosition();
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

  public void setBrakes(boolean brake){
    if(brake){
      for(SwerveModule m : mSwerveMods){
        m.setBrakeMode(true);
      }
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

  public PoseEstimator getPoseEstimator(){
    return this.PoseEstimator;
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
    // pose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
    // double poseX = pose[0];
    // double poseY = pose[1];
    // Rotation2d poseR = Rotation2d.fromDegrees(pose[5]);
    // double timeStamp = Timer.getFPGATimestamp() - (pose[6] / 1000.0);
    lastTimeStamp = Timer.getFPGATimestamp();


    // if (Math.abs(pose[0]) >= 0.1) {
    //   PoseEstimator.addVisionMeasurement(new Pose2d(poseX, poseY, poseR), timeStamp);
    //   //PoseEstimator.resetPosition(poseR, getPositions(), new Pose2d(poseX, poseY, poseR));
    // }

    PoseEstimator.update(getYaw(), getPositions());

    vecPose = new Pose2d((PoseEstimator.getEstimatedPosition().getX() - lastPose.getX())/ (Timer.getFPGATimestamp()-lastTimeStamp), (PoseEstimator.getEstimatedPosition().getY() - lastPose.getY())/(Timer.getFPGATimestamp()-lastTimeStamp), PoseEstimator.getEstimatedPosition().getRotation());

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

  public Pose2d getVelocity(){
    return new Pose2d((PoseEstimator.getEstimatedPosition().getX() - lastPose.getX())/ (Timer.getFPGATimestamp()-lastTimeStamp), (PoseEstimator.getEstimatedPosition().getY() - lastPose.getY())/(Timer.getFPGATimestamp()-lastTimeStamp), PoseEstimator.getEstimatedPosition().getRotation());
  }

  public void setSwerveDrive(double xVelocity, double yVelocity, double rotationVelocity, boolean useOdometry, boolean fieldOriented) {
  if (fieldOriented) {
      setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates( ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, -rotationVelocity, Rotation2d.fromDegrees(gyro.getYaw()))));
  }else{
      ChassisSpeeds cs = new ChassisSpeeds(-xVelocity, -yVelocity, rotationVelocity);
      setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(cs));
  }
  
}

}
