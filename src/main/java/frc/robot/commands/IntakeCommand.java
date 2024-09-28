package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;


public class IntakeCommand extends Command{
    private Intake s_Intake;
    private Index s_Index;
    private Swerve s_Swerve;
    public static double tx;
    public static double ty;
    public static double boxlength;
    public static double boxwidth;
    public static double lenpct;
    public static double widpct;

    public static double adj;
    public static double dist;
    
    private enum IntakeTypes {
        beforeAnything, 
        inTook,
        afterAnything,
        nothing,
        end
    }
    private IntakeTypes intakePoint = IntakeTypes.nothing;

    public IntakeCommand(Intake s_Intake, Index s_Index, Swerve s_Swerve){
        this.s_Intake = s_Intake;
        this.s_Index = s_Index;
        this.s_Swerve = s_Swerve;

        this.intakePoint = IntakeTypes.beforeAnything;
        addRequirements(s_Intake);
    }
    @Override
    public void initialize(){
        intakePoint = IntakeTypes.beforeAnything;
    }

    @Override
    public void execute() {
        boolean targetNote;

        NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("pipeline").setNumber(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tx").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("ty").getDouble(0);
        boxlength = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tlong").getDouble(0);
        boxwidth = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tshort").getDouble(0);

        lenpct = (boxlength / 960) * 100;
        widpct = (boxwidth / 720) * 100;

        double noteWidth = 35;

        dist = ((-2.86) * Math.log(lenpct) + 13.1) * noteWidth;

        // f(ty) = tan(90-ty) x 15.24-5.56
        // 5.56 = Ring Height
        // 15.24 = LimeLight Height From Carpet
        // ty = Angle of Elevation From LimeLight

        SmartDashboard.putNumber("Detected Note Distance", dist);
        SmartDashboard.putNumber("x", tx);
        SmartDashboard.putNumber("y", ty);
        SmartDashboard.putNumber("boxlength", boxlength);
        SmartDashboard.putNumber("boxwidth", boxwidth);
        SmartDashboard.putNumber("boxlengthpct", lenpct);
        SmartDashboard.putNumber("boxwidthpct", widpct);

        double cameraYdistanceFromNote = (Math.tan(ty)/(Constants.Subsystems.FRONT_CAMERA_HEIGHT));
        double cameraXdistanceFromNote = (Math.tan(tx)*cameraYdistanceFromNote);

        adj = tx / 50.0; //Field of view is 100 degrees

        if (adj < 0.1 && adj > 0.0) {
            adj = 0.1;
        }
        if (adj > -0.1 && adj < 0.0) {
            adj = -0.1;
        }

        // Small adjustment: 0.4
        // Medium adjustment: 1
        // Large adjustment: 5
        // Very large adjustment: 7

        if (Math.abs(ty) > 0.0) {
            System.out.println("this is before TranslationX");
            double translationY = -(cameraXdistanceFromNote+Constants.Subsystems.X_CAMERA_DISTANCE_FROM_CENTER); //robot angle changing
            System.out.println("this is before TranslationY");
            double translationX = -(cameraYdistanceFromNote+Constants.Subsystems.Y_CAMERA_DISTANCE_FROM_CENTER); //the robot is not going forward for now
            System.out.println("this is before rotation");
            double rotation = Math.toRadians(Math.atan(translationY/translationX)); //robot is rotating
            System.out.println("this is before s_swerve.drive");
            Translation2d mytranslation = new Translation2d(translationX, -translationY);
            s_Swerve.drive(mytranslation, Math.signum(rotation)*(Constants.Swerve.maxAngularVelocity), false, true);
        } else {
            System.out.println("there appears to be no note detected. is the vision bad or is the note not there?");
        }

        switch (intakePoint) {
            case beforeAnything:
                if(!s_Intake.isIntaked()){
                    s_Intake.runIntake(0.4);
                    s_Index.runIndex(1);
                }else{
                    intakePoint =IntakeTypes.inTook;
                }
                break;
            case inTook:
                if(s_Intake.isIntaked()){
                    s_Intake.runIntake(0.4);
                    s_Index.runIndex(1);
                }else{
                    intakePoint = IntakeTypes.afterAnything;
                }
                break; 
            case afterAnything:
                if(!s_Intake.isIntaked()){
                    s_Intake.stopIntake();;
                    s_Index.runIndex(-0.3);
                }else{
                    intakePoint = IntakeTypes.end;
                }
                break;   
            default:
                break;
        }

    }

    @Override
    public boolean isFinished() {
        return (intakePoint == IntakeTypes.end);
    }
    @Override
    public void end(boolean isInterrupted) {
        s_Index.stopIndex();
        s_Intake.stopIntake();
    }
}
