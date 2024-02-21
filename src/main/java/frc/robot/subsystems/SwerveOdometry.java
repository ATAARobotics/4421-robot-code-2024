package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveOdometry extends SwerveDriveOdometry{

    private boolean isInitialized = false;
    Pigeon2 pigeon;
    Alliance alliance;
    double time;
    private double prevXVel;
    private double prevYVel;
    private double prevRotVel;

    public double getXVel() {
        return prevXVel;
    }

    public double getYVel() {
        return prevYVel;
    }

    public double getRotVel() {
        return prevRotVel;
    }

    public SwerveOdometry(SwerveDriveKinematics kinematics, Rotation2d angle, SwerveModulePosition[] swerveModules, Pigeon2 pigeon, Alliance alliance) {
        super(kinematics, angle, swerveModules);
        this.pigeon = pigeon;
        this.alliance = alliance;
        this.time = Timer.getFPGATimestamp();
    }

    // take the average of the 2 poses
    public void addAprilTag(Pose2d pose, boolean redSide,SwerveModulePosition[] positions) {
        
        if (!isInitialized) {
            
            if (alliance == Alliance.Red) {
                pigeon.setYaw(pose.getRotation().getDegrees() + 180);
            } else {
                pigeon.setYaw(pose.getRotation().getDegrees());
            }
            resetPosition(new Rotation2d(pigeon.getYaw()), positions, pose);
            isInitialized = true;
            return;
        }
        if (AprilTagError(this.getPoseMeters(), pose)){
            double x, y;
        
            x = (this.getPoseMeters().getX() + pose.getX()) / 2.0;
            y = (this.getPoseMeters().getY() + pose.getY()) / 2.0;
            resetPosition(new Rotation2d(pigeon.getYaw()), positions, new Pose2d(x, y, this.getPoseMeters().getRotation())); 
        }
    }
    public boolean AprilTagError(Pose2d currentPose, Pose2d newPose){
        double error = Math.sqrt(Math.pow(currentPose.getX() - newPose.getX(), 2) + Math.pow(currentPose.getY() - newPose.getY(), 2));
        if (error < 0.1){
            return true;
        }
        else{
            return false;
        }
    }

    public void reinitialize() {
        isInitialized = false;
    }

    public boolean getIsInitialized() {
        return isInitialized;
    }
    @Override
    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        SmartDashboard.putNumber("Pose X", getPoseMeters().getX());
        SmartDashboard.putNumber("pose Y", getPoseMeters().getY());
        SmartDashboard.putNumber("Pose Rot", getPoseMeters().getRotation().getDegrees());
        double delta = Timer.getFPGATimestamp() - time;
        time = Timer.getFPGATimestamp();
        double tempX = getPoseMeters().getX();
        double tempY = getPoseMeters().getY();
        double tempRot = getPoseMeters().getRotation().getDegrees();
        Pose2d tempPose = super.update(gyroAngle, modulePositions);
        prevXVel = (getPoseMeters().getX() - tempX) / delta;
        prevYVel = (getPoseMeters().getY() - tempY) / delta;
        prevRotVel = (getPoseMeters().getRotation().getDegrees() - tempRot) / delta;
        return tempPose;
    }
}