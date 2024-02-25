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
    private CANSparkFlex PivotMotor;
    private CANCoder pivotEncoder;

    private PIDController pivotPID = new PIDController(0.5, 0, 0.004);

    private double ffConstant = 0.5;

    public Pivot(){

        PivotMotor = new CANSparkFlex(Constants.Subsystems.shooterPivot, CANSparkLowLevel.MotorType.kBrushless);
        
        pivotEncoder = new CANCoder(22);

    }


    @Override
    public void periodic(){

        double angle = pivotEncoder.getAbsolutePosition();
        SmartDashboard.putNumber("Pivot Angle", angle);
        SmartDashboard.putNumber("Pivot Setpoint", pivotPID.getSetpoint());

        if (GoingToSetpoint){
            double pidVal = pivotPID.calculate(angle);
            double ffVal = ffConstant*Math.sin(Math.toRadians(angle));
            double val = MathUtil.clamp(pidVal+ffVal, -1, 1);
            PivotMotor.set(val);
        }
    }

    public void PivotUp(){
        GoingToSetpoint = false;
        PivotMotor.set(0.5);
    }
    public void PivotDown(){
        GoingToSetpoint = false;
        PivotMotor.set(-0.5);
    }
    public void toSetpoint(double setPoint){
        GoingToSetpoint = true;
        pivotPID.setSetpoint(setPoint);
    }
    public void stop(){
        GoingToSetpoint = false;
        PivotMotor.set(0);
    }
    public boolean AtSetpoint(){
        return (Math.abs(pivotPID.getSetpoint()-pivotEncoder.getAbsolutePosition()) <= 1.5);
    }


}
