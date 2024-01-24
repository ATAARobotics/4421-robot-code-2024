package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class Shooting extends Command {
    private Pose3d robotPose;
    private Pose3d goal;
    private double distance;
    private double angle;
    private double shooterAngle;

    private Shooter mShooter;
    private Swerve mSwerve;
    
    private PIDController rotController = new PIDController(1.0, 0.15, 0);
    public Shooting(Shooter m_shooter, Swerve m_swerve){
        mShooter = m_shooter;
        mSwerve = m_swerve;
    }

    @Override
    public void initialize(){
        distance = Math.sqrt(Math.pow(goal.getX()-robotPose.getX(), 2) + Math.pow(goal.getY()-robotPose.getY(), 2));
        angle = Math.atan2(goal.getY()-robotPose.getY(), goal.getX()-robotPose.getX());
        shooterAngle = Math.atan2(goal.getZ()-robotPose.getZ(), distance);
        rotController.setSetpoint(angle);
    }

    @Override
    public void execute(){
        rotController.calculate(mSwerve.getYaw().getRadians());
        mSwerve.drive(new Translation2d(0, 0), angle, true, false);
    }
}
