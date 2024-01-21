package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shooter extends SubsystemBase{
    public CANSparkFlex intake;
    private boolean isFiring = false;

    public Shooter(){
        intake = new CANSparkFlex(Constants.Subsystems.intake, CANSparkLowLevel.MotorType.kBrushless);
        SmartDashboard.setDefaultNumber("Intake", 0.5);
    }

    @Override
    public void periodic(){
        if(isFiring){
            // leftShooterPID.setReference(10000, ControlType.kVelocity);
            // rightShooterPID.setReference(10000, ControlType.kVelocity);
        }else{
        }
    }

    public void Fire(){
        
        if(isFiring){
            isFiring = false;
        }else{
            isFiring = true;
        }
        
    }
    public void Index(){
        intake.set(SmartDashboard.getNumber("Intake", 0));
    }
    public void stopIndex(){
        intake.stopMotor();
    }

    public void In(){
        //leftShooterPID.setReference(10000, ControlType.kVelocity);
        //rightShooterPID.setReference(10000, ControlType.kVelocity);
    }
    public void stop(){
    }

}
