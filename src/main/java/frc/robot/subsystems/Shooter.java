package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shooter extends SubsystemBase{
    public CANSparkFlex leftShooter;
    public CANSparkFlex rightShooter;
    public CANSparkFlex leftIndex;
    public CANSparkFlex rightIndex;
    public SparkPIDController leftShooterPID;
    public SparkPIDController rightShooterPID;
    private boolean isFiring = false;

    public Shooter(){
        leftShooter = new CANSparkFlex(Constants.Subsystems.leftSide, CANSparkLowLevel.MotorType.kBrushless);
        rightShooter = new CANSparkFlex(Constants.Subsystems.rightSide, CANSparkLowLevel.MotorType.kBrushless);
        leftIndex = new CANSparkFlex(Constants.Subsystems.leftSideIndex, CANSparkLowLevel.MotorType.kBrushless);
        rightIndex = new CANSparkFlex(Constants.Subsystems.rightSideIndex, CANSparkLowLevel.MotorType.kBrushless);
        
        leftShooter.setInverted(true);
        leftIndex.setInverted(true);

        leftShooterPID = leftShooter.getPIDController();
        rightShooterPID = rightShooter.getPIDController();

        SmartDashboard.putNumber("left velocity", leftShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("right velocity", rightShooter.getEncoder().getVelocity());

        
        SmartDashboard.setDefaultNumber("Left Shooter", 0.5);
        SmartDashboard.setDefaultNumber("Right Shooter", 0.5);
        SmartDashboard.setDefaultNumber("Left Index", 0.5);
        SmartDashboard.setDefaultNumber("Right Index", 0.5);

        SmartDashboard.setDefaultNumber("left shooter p", 1);
        SmartDashboard.setDefaultNumber("left shooter i", 0);
        SmartDashboard.setDefaultNumber("left shooter d", 0);
        SmartDashboard.setDefaultNumber("left shooter ff", 0);

        SmartDashboard.setDefaultNumber("right shooter p", 1);
        SmartDashboard.setDefaultNumber("right shooter i", 0);
        SmartDashboard.setDefaultNumber("right shooter d", 0);
        SmartDashboard.setDefaultNumber("right shooter ff", 0);


        
        leftShooterPID.setP(SmartDashboard.getNumber("left shooter p", 1));
        leftShooterPID.setI(SmartDashboard.getNumber("left shooter i", 0));
        leftShooterPID.setD(SmartDashboard.getNumber("left shooter d", 0));
        leftShooterPID.setFF(SmartDashboard.getNumber("left shooter ff", 0));

        rightShooterPID.setP(SmartDashboard.getNumber("right shooter p", 1));
        rightShooterPID.setI(SmartDashboard.getNumber("right shooter i", 0));
        rightShooterPID.setD(SmartDashboard.getNumber("right shooter d", 0));
        rightShooterPID.setFF(SmartDashboard.getNumber("right shooter ff", 0));


        
    }

    @Override
    public void periodic(){
        if(isFiring){
            // leftShooter.set(SmartDashboard.getNumber("Left Shooter", 0));
            // rightShooter.set(SmartDashboard.getNumber("Right Shooter", 0));

            leftShooterPID.setReference(4000, ControlType.kVelocity);
            rightShooterPID.setReference(4000, ControlType.kVelocity);
        }else{
            leftShooter.stopMotor();
            rightShooter.stopMotor();
        }

        leftShooterPID.setP(SmartDashboard.getNumber("left shooter p", 0));
        leftShooterPID.setI(SmartDashboard.getNumber("left shooter i", 0));
        leftShooterPID.setD(SmartDashboard.getNumber("left shooter d", 0));
        leftShooterPID.setFF(SmartDashboard.getNumber("left shooter ff", 0));

        rightShooterPID.setP(SmartDashboard.getNumber("right shooter p", 0));
        rightShooterPID.setI(SmartDashboard.getNumber("right shooter i", 0));
        rightShooterPID.setD(SmartDashboard.getNumber("right shooter d", 0));
        rightShooterPID.setFF(SmartDashboard.getNumber("right shooter ff", 0));
    
    
        SmartDashboard.putNumber("left velocity", leftShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("right velocity", rightShooter.getEncoder().getVelocity());
    }

    public void Fire(){
        isFiring = !isFiring;
    }
    public void Index(){
        leftIndex.set(SmartDashboard.getNumber("Left Index", 0));
        rightIndex.set(SmartDashboard.getNumber("Right Index", 0));
    }
    public void stopIndex(){
        leftIndex.stopMotor();
        rightIndex.stopMotor();
    }

    public void In(){
        //leftShooterPID.setReference(10000, ControlType.kVelocity);
        //rightShooterPID.setReference(10000, ControlType.kVelocity);
        leftShooter.set(-0.2);
        rightShooter.set(-0.2);
    }
    public void stop(){
        rightShooter.stopMotor();
        leftShooter.stopMotor();
        isFiring=false;
    }

}
