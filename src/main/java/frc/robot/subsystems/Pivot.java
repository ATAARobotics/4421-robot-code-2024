package frc.robot.subsystems;
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
    private CANcoder pivotEncoder;

    private PIDController pivotPID = new PIDController(0.1, 0, 0);



    public Pivot(){

        PivotMotor = new CANSparkFlex(Constants.Subsystems.shooterPivot, CANSparkLowLevel.MotorType.kBrushless);
        
        pivotEncoder = new CANcoder(22);

        SmartDashboard.putNumber("Pivot P", 0);
        SmartDashboard.putNumber("Pivot I", 0);
        SmartDashboard.putNumber("Pivot D", 0);
        SmartDashboard.putNumber("Pivot FF", 0);

    }


    @Override
    public void periodic(){
        pivotPID.setP(SmartDashboard.getNumber("Pivot P", 0));
        pivotPID.setI(SmartDashboard.getNumber("Pivot I", 0));
        pivotPID.setD(SmartDashboard.getNumber("Pivot D", 0));

        double ffConstant = SmartDashboard.getNumber("Pivot FF", 0);

        double angle = Math.toDegrees(pivotEncoder.getPosition().getValueAsDouble()*2*Math.PI);
        SmartDashboard.putNumber("Pivot Angle", angle);

        if (GoingToSetpoint){
            double pidVal = pivotPID.calculate(angle);
            double ffVal = ffConstant*Math.sin(angle);
            double val = MathUtil.clamp(pidVal+ffVal, -1, 1);
            PivotMotor.set(val);
        }
    }

    public void PivotUp(){
        GoingToSetpoint = false;
        PivotMotor.set(0.1);
    }
    public void PivotDown(){
        GoingToSetpoint = false;
        PivotMotor.set(-0.1);
    }
    public void toSetpoint(double setPoint){
        GoingToSetpoint = true;
        pivotPID.setSetpoint(setPoint);
    }
    public void stop(){
        GoingToSetpoint = false;
        PivotMotor.set(0);
    }


}
