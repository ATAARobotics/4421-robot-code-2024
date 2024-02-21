package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class ChaseTag extends Command {

    private Swerve m_swerveDriveSubsystem;
    private PoseEstimator poseEstimator;

    // Poses
    private Pose2d targetPose;
    private Pose2d robotPose;
    private Pose2d goalPose;

    // speed variables
    private double xSpeed;
    private double ySpeed;
    private double rotTemp;
    private double rotSpeed;
    private double speedLimit;
    private double rotLimit;

    private boolean isEndPoint;

    // PID
    private final PIDController xController = new PIDController(3.0, 0, 0);
    private final PIDController yController = new PIDController(5.0, 0, 0);
    private final PIDController rotController = new PIDController(10.0, 0.0, 0.0);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.Waypoint.driveKS, Constants.Swerve.Waypoint.driveKV);

    private SlewRateLimiter accelerationLimiter = new SlewRateLimiter(2.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    private boolean Y_ACH = false;
    private boolean X_ACH = false;
    private boolean ROT_ACH = false;

    public ChaseTag(Swerve swerveDriveSubsystem, PoseEstimator poseEstimator, Pose2d targetPose, double driveTolerance, double rotTolerance, double speedLimit, double rotLimit, boolean isEndPoint) {
        this.m_swerveDriveSubsystem = swerveDriveSubsystem;
        this.targetPose = targetPose;
        this.poseEstimator = poseEstimator;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        this.speedLimit = 1.0;
        this.rotLimit = 3*Math.PI;
        this.isEndPoint = isEndPoint;
        addRequirements(this.m_swerveDriveSubsystem);
    }

    public ChaseTag(Swerve swerveDriveSubsystem, Pose2d targetPose, boolean isEndPoint) {
      this(swerveDriveSubsystem, swerveDriveSubsystem.getPoseEstimator(), targetPose, Constants.Swerve.Waypoint.E_DTOLERANCE, Constants.Swerve.Waypoint.E_RTOLERANCE, Constants.Swerve.Waypoint.SPEEDLIMIT, Constants.Swerve.Waypoint.ROTLIMIT, isEndPoint);
    }

    @Override
    public void initialize() {
        m_swerveDriveSubsystem.setBrakes(true);
        goalPose = targetPose;

        if (false) {
          xController.setTolerance(Constants.Swerve.Waypoint.E_DTOLERANCE);
          yController.setTolerance(Constants.Swerve.Waypoint.E_DTOLERANCE);
          rotController.setTolerance(Units.degreesToRadians(Constants.Swerve.Waypoint.E_RTOLERANCE));
        } else {
          xController.setTolerance(10.75);
          yController.setTolerance(10.75);
          rotController.setTolerance(Units.degreesToRadians(Constants.Swerve.Waypoint.RTOLERANCE));
        }

        xController.setSetpoint(goalPose.getX());
        yController.setSetpoint(goalPose.getY());
        rotController.setSetpoint(goalPose.getRotation().getRadians());
    }

    @Override
    public void execute() {

      robotPose = poseEstimator.getEstimatedPosition();

      SmartDashboard.putNumber("X-Goal", goalPose.getX());
      SmartDashboard.putNumber("Y-Goal", goalPose.getY());
      SmartDashboard.putNumber("Rot-Goal", goalPose.getRotation().getRadians());
      
      SmartDashboard.putNumber("robotPoseX", robotPose.getX());
      SmartDashboard.putNumber("robotPoseY", robotPose.getY());
      SmartDashboard.putNumber("robotPoseR", robotPose.getRotation().getRadians());


      SmartDashboard.putBoolean("X-ACH", false);

            // if (xController.atSetpoint()) {
      //   SmartDashboard.putBoolean("X-ACH", true);
      //   xSpeed = 0;
      // }

      SmartDashboard.putBoolean("Y-ACH", false);

      rotSpeed = MathUtil.clamp(rotController.calculate(m_swerveDriveSubsystem.getYaw().getRadians()), -rotLimit, rotLimit);

      
      
      // if (yController.atSetpoint()) {
      //   SmartDashboard.putBoolean("Y-ACH", true);
      //   ySpeed = 0;
      // }
      

      // rotTemp = m_swerveDriveSubsystem.getHeading();
      // if(robotPose.getRotation().getRadians() > 0) {
      //   rotTemp = rotTemp - Math.PI;
      // }
      // else {
      //   rotTemp = rotTemp + Math.PI;
      // }

      SmartDashboard.putBoolean("ROT-ACH", false);

      if (rotController.atSetpoint()) {
        xSpeed = accelerationLimiter.calculate(xSpeed);
        xSpeed = MathUtil.clamp(xController.calculate(robotPose.getX()), -speedLimit, speedLimit);

        ySpeed = accelerationLimiter.calculate(ySpeed);
        ySpeed = MathUtil.clamp(yController.calculate(robotPose.getY()), -speedLimit, speedLimit);
      
      }

      

      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      SmartDashboard.putNumber("rotSpeed", rotSpeed);

      // Drive // x and y is flipped
      // velocity stored in pose2d: x is xvelocity and vise versa
      double xVel = m_swerveDriveSubsystem.getVelocity().getX();
      double yVel = m_swerveDriveSubsystem.getVelocity().getY();
      m_swerveDriveSubsystem.drive(
          new Translation2d(xSpeed + feedforward.calculate(xVel), ySpeed + feedforward.calculate(yVel)),
          rotSpeed,
          true,
          true);
      // m_swerveDriveSubsystem.drive(xSpeed + feedforward.calculate(xVel), ySpeed + feedforward.calculate(yVel), rotSpeed, true, true);
    }
}