package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.Constants;

public class Shooter {
    public CANSparkFlex leftShooter;
    public CANSparkFlex rightShooter;
    public SparkPIDController leftShooterPID;
    public SparkPIDController rightShooterPID;

    public Shooter(){
        leftShooter = new CANSparkFlex(Constants.Subsystems.leftSide, CANSparkLowLevel.MotorType.kBrushless);
        rightShooter = new CANSparkFlex(Constants.Subsystems.rightSide, CANSparkLowLevel.MotorType.kBrushless);
        leftShooter.setInverted(true);
        leftShooterPID = leftShooter.getPIDController();
        rightShooterPID = rightShooter.getPIDController();
        leftShooterPID.setP(5);
        rightShooterPID.setP(5);


    }

    public void Fire(){
        //leftShooterPID.setReference(10000, ControlType.kVelocity);
        //rightShooterPID.setReference(10000, ControlType.kVelocity);
        leftShooter.set(0.25);
        rightShooter.set(0.25);
    }
    public void In(){
        //leftShooterPID.setReference(10000, ControlType.kVelocity);
        //rightShooterPID.setReference(10000, ControlType.kVelocity);
        leftShooter.set(-0.2);
        rightShooter.set(-0.2);
    }
    public void stop(){
        leftShooter.stopMotor();;
        rightShooter.stopMotor();
    }

}
