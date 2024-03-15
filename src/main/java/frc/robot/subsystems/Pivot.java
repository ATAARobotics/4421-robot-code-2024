package frc.robot.subsystems;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;





public class Pivot extends SubsystemBase{


    private boolean GoingToSetpoint = false;
    private boolean ClimbingDown = false;
    private CANSparkFlex PivotMotor;
    private CANSparkFlex PivotMotorSecondary;

    private CANCoder pivotEncoder;

    private PIDController pivotPID = new PIDController(Constants.Subsystems.pivotP, Constants.Subsystems.pivotI, Constants.Subsystems.pivotD);

    private double ffConstant = Constants.Subsystems.pivotFF;
    private double speed = 0.0;
    public Pivot(){

        PivotMotor = new CANSparkFlex(Constants.Subsystems.shooterPivot, CANSparkLowLevel.MotorType.kBrushless);
        PivotMotorSecondary = new CANSparkFlex(23, CANSparkLowLevel.MotorType.kBrushless);
        PivotMotor.setInverted(true);
        PivotMotorSecondary.setInverted(false);
        PivotMotor.setIdleMode(IdleMode.kBrake);
        PivotMotorSecondary.setIdleMode(IdleMode.kBrake);
        PivotMotor.burnFlash();
        PivotMotorSecondary.burnFlash();
        pivotEncoder = new CANCoder(22);
        pivotPID.setTolerance(Constants.Subsystems.pivotTolerance);
        
    }
    @Override
    public void periodic(){

        double angle = pivotEncoder.getAbsolutePosition();
        SmartDashboard.putNumber("Pivot Angle", angle);
        SmartDashboard.putNumber("Pivot Setpoint", pivotPID.getSetpoint());
        SmartDashboard.putNumber("Pivot Error", (Math.abs(pivotPID.getSetpoint()-pivotEncoder.getAbsolutePosition())));
        double pidVal = pivotPID.calculate(angle);

        if(pivotEncoder.getAbsolutePosition() <= 80){
            if (GoingToSetpoint){
                double ffVal = ffConstant*Math.cos(Math.toRadians(angle));
                speed = MathUtil.clamp(pidVal+ffVal, -1, 1);
            }
        }
        else{
            if (GoingToSetpoint && (Math.abs(pivotPID.getSetpoint()-pivotEncoder.getAbsolutePosition()) >= 5)){
                speed = MathUtil.clamp(pidVal, -1, 1);
            }else{
                if(GoingToSetpoint){
                    speed = 0;
                }
            }      
        }

        if (speed < 0){
            if(pivotEncoder.getAbsolutePosition() >= 35){
                PivotMotor.set(speed);
                PivotMotorSecondary.set(speed);
            }
            else if(pivotEncoder.getAbsolutePosition() >=26){
                PivotMotor.set(speed * 0.4);
                PivotMotorSecondary.set(speed * 0.4);
            }
            else{
                PivotMotor.stopMotor();
                PivotMotorSecondary.stopMotor();
            }
        }else{
            if(pivotEncoder.getAbsolutePosition() <= 95){
                PivotMotor.set(speed);
                PivotMotorSecondary.set(speed);
            }else if(pivotEncoder.getAbsolutePosition()<=114){
                PivotMotor.set(speed * 0.4);
                PivotMotorSecondary.set(speed * 0.4);
            }else{
                PivotMotor.stopMotor();
                PivotMotorSecondary.stopMotor();
            }          
        }
    }
    public void PivotUp(){
        GoingToSetpoint = false;
        speed = 0.5;
    }
    public void PivotDown(){
        GoingToSetpoint = false;
        speed = -1;
    }
    public void toSetpoint(double setPoint){
        // pivotPID.setP(SmartDashboard.getNumber("pivot p", 0));
        // pivotPID.setI(SmartDashboard.getNumber("pivot i", 0));
        // pivotPID.setD(SmartDashboard.getNumber("pivot d", 0));
        // ffConstant = (SmartDashboard.getNumber("pivot ff", 0));

        GoingToSetpoint = true;
        ClimbingDown = false;
        pivotPID.setSetpoint(setPoint);
    }
    public void stop(){
        GoingToSetpoint = false;
        ClimbingDown = false;
        speed = 0;
        PivotMotor.set(0);
        PivotMotorSecondary.set(0);
    }
    public boolean AtSetpoint(){
        return (Math.abs(pivotPID.getSetpoint()-pivotEncoder.getAbsolutePosition()) <= Constants.Subsystems.pivotTolerance);
    }


}
