package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class Shooting extends Command {
    private Pose3d robotPose;
    private Translation3d goalPose = new Translation3d(12 , 12, 0);
    private Pose3d goalVelociety;
    private double robotAngle;
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
        
    }

    @Override
    public void execute(){
        mSwerve.getVelocity();
        mSwerve.getPose();
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
        double G = 9.81;
        double A = mSwerve.getPose().getX();
        double B = mSwerve.getPose().getY();
        double C = 1;
        double M = goalPose.getX();
        double N = goalPose.getY();
        double O = goalPose.getY();
        double P = -mSwerve.getVelocity().getX();
        double Q = -mSwerve.getVelocity().getY();
        double R = 0;
        double S = 30;
        
        double H = M - A;
        double J = O - C;
        double K = N - B;
        double L = -0.5 * G;
        
        double c0 = L*L;
        double c1 = -2*Q*L;
        double c2 = Q*Q - 2*K*L - S*S + P*P + R*R;
        double c3 = 2*K*Q + 2*H*P + 2*J*R;
        double c4 = K*K + H*H + J*J;
        
        double t = solveRealQuarticRoots(c0, c1, c2, c3, c4)[-1];
        double d = ((H+P*t)/t);
        double e = ((K+Q*t-L*t*t)/t);
        double f = ((J+R*t)/t);

        
        double rot = rotController.calculate(mSwerve.getYaw().getRadians());
        mSwerve.drive(new Translation2d(0, 0), rot, true, false);
    }

    public static double[] solveRealQuarticRoots(double a, double b, double c, double d, double e) {
        double s1 = 2 * c * c * c - 9 * b * c * d + 27 * (a * d * d + b * b * e) - 72 * a * c * e, q1 = c * c - 3 * b * d + 12 * a * e;
        double discrim1 = -4 * q1 * q1 * q1 + s1 * s1;
        if(discrim1 >0) {
            double s2 = s1 + Math.sqrt(discrim1);
            double q2 = Math.cbrt(s2 / 2);
            double s3 = q1 / (3 * a * q2) + q2 / (3 * a);
            double discrim2 = (b * b) / (4 * a * a) - (2 * c) / (3 * a) + s3;
            if(discrim2>0) {
                double s4 = Math.sqrt(discrim2);
                double s5 = (b * b) / (2 * a * a) - (4 * c) / (3 * a) - s3;
                double s6 = (-(b * b * b) / (a * a * a) + (4 * b * c) / (a * a) - (8 * d) / a) / (4 * s4);
                double discrim3 = (s5 - s6), discrim4 = (s5 + s6);
                // actual root values, may not be set
                double r1 = 0, r2 = 0, r3 =0, r4=0; 
    
                if(discrim3 > 0) {
                     double sqrt1 = Math.sqrt(s5-s6);
                     r1 = -b / (4 * a) - s4/2 + sqrt1 / 2;
                     r2 = -b / (4 * a) - s4/2 - sqrt1 / 2;
                } else if(discrim3 == 0) {
                     // repeated root case
                     r1 = -b / (4 * a) - s4/2;
                }
                if(discrim4 > 0) {
                     double sqrt2 = Math.sqrt(s5+s6);
                     r3 = -b / (4 * a) + s4/2 + sqrt2 / 2;
                     r4 = -b / (4 * a) + s4/2 - sqrt2 / 2;
                } else if(discrim4 ==0) {
                     r3 = -b / (4 * a) + s4/2;
                }
                if(discrim3 > 0 && discrim4 > 0) {
                    return new double[]{r1,r2,r3,r4};
                }
                else if( discrim3 > 0 && discrim4 == 0 )
                     return new double[]{r1,r2,r3};
                else if( discrim3 > 0 && discrim4 < 0 )
                     return new double[]{r1,r2};
                else if( discrim3 == 0 && discrim4 > 0 )
                     return new double[]{r1,r3,r4};
                else if( discrim3 == 0 && discrim4 == 0 )
                     return new double[]{r1,r3};
                else if( discrim3 == 0 && discrim4 < 0 )
                     return new double[]{r1};
                else if( discrim3 < 0 && discrim4 > 0 )
                     return new double[]{r3,r4};
                else if( discrim3 < 0 && discrim4 == 0 )
                     return new double[]{r3};
                else if( discrim3 < 0 && discrim4 < 0 )
                     return new double[0];
           } 
       }
       return new double[0];
    }
}
