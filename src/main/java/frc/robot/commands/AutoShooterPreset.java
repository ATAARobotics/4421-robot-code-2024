package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoShooterPreset extends Command {
     private Translation3d BluegoalPose = new Translation3d((8.84/39.37), 5.5408, (80.91/39.37));
     // private Translation3d RedgoalPose = new Translation3d(16.706342, 5.5408, 2.2);
     private Translation3d RedgoalPose = new Translation3d((642.38/39.37), 5.5408, (80.91/39.37)); //the 20 was 23.25



     // SIDE FLIP
     private Translation3d GoalPose = (DriverStation.getAlliance().get()==Alliance.Red) ? RedgoalPose : BluegoalPose;

    private Shooter mShooter;
    private Swerve mSwerve;
    private Pivot mPivot;
    private Index mIndex;
    
    private final PIDController rotController = new PIDController(10, 20, 1);

     private double ShooterAngle = 2.0;
     private double RobotAngle = 1.0;
     private DoubleSupplier translationSup;
     private DoubleSupplier strafeSup;
     private DoubleSupplier rotationSup;
     private BooleanSupplier flipSup;

     private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
     private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);

     private double G = 9.81;
          // Note Postion
     private double A = 0;
     private double B = 0.4572;
     private double C = 0;
          //TODO change goal pose to be set based on color
     private double M = 0;
     private double N = 0;
     private double O = 0;
          //Velocietys
     private double P = 0;
     private double Q = 0;
     private double R = 0;
     private double S = 12;

     private double H = M - A;
     private double J = O - C;
     private double K = N - B;
     private double L = -0.5 * G;

     private double c0 = L*L;
     private double c1 = -2*Q*L;
     private double c2 = Q*Q - 2*K*L - S*S + P*P + R*R;
     private double c3 = 2*K*Q + 2*H*P + 2*J*R;
     private double c4 = K*K + H*H + J*J;

     private double d = 0;
     private double e = 0;
     private double f = 0;
     private double X = 0;
     private Timer shootTimer = new Timer();
     private Timer indexTimer = new Timer();
     private Timer overrideTimer = new Timer();
     private double rotationVal = 0;

     private boolean hasNote = true;

    public AutoShooterPreset(
          Shooter m_shooter, 
          Pivot m_Pivot,
          Swerve m_swerve,
          Double X,
          Double Y,
          BooleanSupplier flip
     ){
        this.mShooter = m_shooter;
        this.mSwerve = m_swerve;
        this.mPivot = m_Pivot; 
  
        this.X = X;
        this.C = Y;
        this.flipSup = flip;

        addRequirements(mShooter);
          SmartDashboard.putNumber("Rot P", 0);
          SmartDashboard.putNumber("Rot I", 0);
          SmartDashboard.putNumber("Rot D", 0);
          rotController.enableContinuousInput(-Math.PI, Math.PI);

    }


    @Override
    public void initialize(){
        rotController.setTolerance(Math.toRadians(Constants.Subsystems.rotTolerance));
        rotController.setIZone(Math.toRadians(10));
        GoalPose = (DriverStation.getAlliance().get()==Alliance.Red) ? RedgoalPose : BluegoalPose;
     //    mSwerve.setAutoLock(true);
        shootTimer.reset();
        shootTimer.stop();
        indexTimer.reset();
        indexTimer.stop();
        overrideTimer.reset();
        overrideTimer.stop();
        overrideTimer.start();
    }

    @Override
    public void execute(){
          mSwerve.setAutoLock(true);
          // # g = 9.81
          // # A = proj_pos.x
          // # B = proj_pos.y
          // # C = proj_ pos.z
          // # M = target_pos.X
          // # N = target_pos.y
          // # O = target_pos.z
          // # P = target_velocity.x
          // # Q = target_velocity.y
          // # R = target_velocity.z
          // # S = proj_speed;
          // Note Postion
          //velocity
          ChassisSpeeds vec = mSwerve.getVelocityFromChassisSpeeds();
          // P = -vec.vyMetersPerSecond;
          P = 0;
          Q = 0;
          // R = -mSwerve.getChassisSpeeds().vyMetersPerSecond;
          // R = -vec.vxMetersPerSecond;
          R = 0;
          // Note Postion
          B = 0.4572;
          if(!flipSup.getAsBoolean()){
               A = X;
          }else{
               A = 16.541242-X;
          } 


          //TODO change goal pose to be set based on color
          M = GoalPose.getX();
          N = GoalPose.getZ();
          O = GoalPose.getY();
          S = 12.2;

          H = M - A;
          J = O - C;
          K = N - B;
          L = -0.5 * G;

          c0 = L*L;
          c1 = -2*Q*L;
          c2 = Q*Q - 2*K*L - S*S + P*P + R*R;
          c3 = 2*K*Q + 2*H*P + 2*J*R;
          c4 = K*K + H*H + J*J;
          double[] ts = solveQuartic(c0, c1, c2, c3, c4);
          double t = 1000000000;
          mShooter.AutoFire();
          ChassisSpeeds moving = mSwerve.getVelocityFromChassisSpeeds();
          rotationVal = 0;
          if(ts != null){
               for (int i=0; i<ts.length; i++){
                    if (ts[i] >= 0 & ts[i]<t){
                         t = ts[i];
                    }
               }
               d = ((H+P*t)/t);
               e = ((K+Q*t-L*t*t)/t);
               f = ((J+R*t)/t);
               ShooterAngle = Math.atan2(e, Math.sqrt(Math.pow(d,2) + Math.pow(f,2)));
               RobotAngle = Math.atan2(f, d);
               rotController.setSetpoint(RobotAngle);
               rotController.calculate(mSwerve.getPose().getRotation().getRadians());
               mSwerve.setAutoAngle(Math.toDegrees(RobotAngle));
               mPivot.toSetpoint(Math.toDegrees(ShooterAngle));
               rotationVal = MathUtil.clamp(rotController.calculate(mSwerve.getPose().getRotation().getRadians()), -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity);
          }
          SmartDashboard.putBoolean("Shooter At Setpoint", mShooter.CanShoot());
          SmartDashboard.putBoolean("Rotation At Setpoint", rotController.atSetpoint());
          SmartDashboard.putBoolean("Pivot At Setpoint", mPivot.AtSetpoint());

    }
     @Override
     public boolean isFinished() {
          return indexTimer.hasElapsed(0.25);
     }
     @Override
     public void end(boolean interrupted) {
          System.out.println("Auto Shooter Ended");
          mSwerve.setAutoLock(false);
          indexTimer.reset();
          indexTimer.stop();
     //  mPivot.stop();

         // actuator down
     }


    public static double[] solveQuartic(double a, double b, double c, double d, double e) {
          double inva = 1 / a;
          double c1 = b * inva;
          double c2 = c * inva;
          double c3 = d * inva;
          double c4 = e * inva;
          // cubic resolvant
          double c12 = c1 * c1;
          double p = -0.375 * c12 + c2;
          double q = 0.125 * c12 * c1 - 0.5 * c1 * c2 + c3;
          double r = -0.01171875 * c12 * c12 + 0.0625 * c12 * c2 - 0.25 * c1 * c3 + c4;
          double z = solveCubicForQuartic(-0.5 * p, -r, 0.5 * r * p - 0.125 * q * q);
          double d1 = 2.0 * z - p;
          if (d1 < 0) {
          if (d1 > 1.0e-10)
               d1 = 0;
          else
               return null;
          }
          double d2;
          if (d1 < 1.0e-10) {
          d2 = z * z - r;
          if (d2 < 0)
               return null;
          d2 = Math.sqrt(d2);
          } else {
          d1 = Math.sqrt(d1);
          d2 = 0.5 * q / d1;
          }
          // setup usefull values for the quadratic factors
          double q1 = d1 * d1;
          double q2 = -0.25 * c1;
          double pm = q1 - 4 * (z - d2);
          double pp = q1 - 4 * (z + d2);
          if (pm >= 0 && pp >= 0) {
          // 4 roots (!)
          pm = Math.sqrt(pm);
          pp = Math.sqrt(pp);
          double[] results = new double[4];
          results[0] = -0.5 * (d1 + pm) + q2;
          results[1] = -0.5 * (d1 - pm) + q2;
          results[2] = 0.5 * (d1 + pp) + q2;
          results[3] = 0.5 * (d1 - pp) + q2;
          // tiny insertion sort
          for (int i = 1; i < 4; i++) {
               for (int j = i; j > 0 && results[j - 1] > results[j]; j--) {
                    double t = results[j];
                    results[j] = results[j - 1];
                    results[j - 1] = t;
               }
          }
          return results;
          } else if (pm >= 0) {
          pm = Math.sqrt(pm);
          double[] results = new double[2];
          results[0] = -0.5 * (d1 + pm) + q2;
          results[1] = -0.5 * (d1 - pm) + q2;
          return results;
          } else if (pp >= 0) {
          pp = Math.sqrt(pp);
          double[] results = new double[2];
          results[0] = 0.5 * (d1 - pp) + q2;
          results[1] = 0.5 * (d1 + pp) + q2;
          return results;
          }
          return null;
     }

     /**
      * Return only one root for the specified cubic equation. This routine is
     * only meant to be called by the quartic solver. It assumes the cubic is of
     * the form: x^3+px^2+qx+r.
     * 
     * @param p
     * @param q
     * @param r
     * @return
     */
     private static final double solveCubicForQuartic(double p, double q, double r) {
          double A2 = p * p;
          double Q = (A2 - 3.0 * q) / 9.0;
          double R = (p * (A2 - 4.5 * q) + 13.5 * r) / 27.0;
          double Q3 = Q * Q * Q;
          double R2 = R * R;
          double d = Q3 - R2;
          double an = p / 3.0;
          if (d >= 0) {
          d = R / Math.sqrt(Q3);
          double theta = Math.acos(d) / 3.0;
          double sQ = -2.0 * Math.sqrt(Q);
          return sQ * Math.cos(theta) - an;
          } else {
          double sQ = Math.pow(Math.sqrt(R2 - Q3) + Math.abs(R), 1.0 / 3.0);
          if (R < 0)
               return (sQ + Q / sQ) - an;
          else
               return -(sQ + Q / sQ) - an;
          }
     }

     public void setHasNote(boolean hasNote) {
        this.hasNote = hasNote;
     }
}
