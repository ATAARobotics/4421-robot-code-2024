package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class GetToAmp extends Command {

    private Swerve m_swerveDriveSubsystem;
    private PoseEstimator poseEstimator;

    // Poses
    private Pose2d targetPose;
    private Pose2d robotPose;

    // TODO: ADD BLUE GOAL POSE
    private Pose2d BluegoalPose =  new Pose2d((72.5/39.37), (323.00/39.37) - 1.5, Rotation2d.fromDegrees(90));
    private Pose2d RedgoalPose = new Pose2d((578.77/39.37), (323.00/39.37) - 1.5, Rotation2d.fromDegrees(90));

    private Pose2d goalPose = (DriverStation.getAlliance().get()==Alliance.Red) ? RedgoalPose : BluegoalPose;

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
    private final PIDController rotController = new PIDController(Constants.Subsystems.rotationPID.kP, Constants.Subsystems.rotationPID.kI, Constants.Subsystems.rotationPID.kD);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.Waypoint.driveKS, Constants.Swerve.Waypoint.driveKV);

    private SlewRateLimiter accelerationLimiter = new SlewRateLimiter(2.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    private boolean Y_ACH = false;
    private boolean X_ACH = false;
    private boolean ROT_ACH = false;

    public GetToAmp(Swerve swerveDriveSubsystem, PoseEstimator poseEstimator, double driveTolerance, double rotTolerance, double speedLimit, double rotLimit, boolean isEndPoint) {
        this.m_swerveDriveSubsystem = swerveDriveSubsystem;
        this.poseEstimator = poseEstimator;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        this.speedLimit = 1.0;
        this.rotLimit = 3*Math.PI;
        this.isEndPoint = isEndPoint;
        rotController.setIZone(Constants.Subsystems.rotationPID.iZone);
        addRequirements(this.m_swerveDriveSubsystem);
    }

    public GetToAmp(Swerve swerveDriveSubsystem, boolean isEndPoint) {
      this(swerveDriveSubsystem, swerveDriveSubsystem.getPoseEstimator(), Constants.Swerve.Waypoint.E_DTOLERANCE, Constants.Swerve.Waypoint.E_RTOLERANCE, Constants.Swerve.Waypoint.SPEEDLIMIT, Constants.Swerve.Waypoint.ROTLIMIT, isEndPoint);
    }

    @Override
    public void initialize() {
        goalPose = (DriverStation.getAlliance().get()==Alliance.Red) ? RedgoalPose : BluegoalPose;
        m_swerveDriveSubsystem.setBrakes(true);
        goalPose = targetPose;


          xController.setTolerance(0.05);
          yController.setTolerance(0.05);
          rotController.setTolerance(Units.degreesToRadians(Constants.Swerve.Waypoint.RTOLERANCE));

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
    @Override
    public boolean isFinished() {
        return (xController.atSetpoint() && yController.atSetpoint());
    }

}