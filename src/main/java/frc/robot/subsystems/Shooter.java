package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shooter extends SubsystemBase{
    public CANSparkFlex intake;
    private boolean isFiring = false;

    public CANSparkFlex leftShooter;
    public CANSparkFlex rightShooter;
    public CANSparkFlex leftIndex;
    public CANSparkFlex rightIndex;
    public SparkPIDController leftShooterPID;
    public SparkPIDController rightShooterPID;

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
        SmartDashboard.putNumber("left power", leftShooter.getAppliedOutput());
        SmartDashboard.putNumber("right power", rightShooter.getAppliedOutput());

        
        leftShooterPID.setP(Constants.Subsystems.shooterP);
        leftShooterPID.setI(Constants.Subsystems.shooterI);
        leftShooterPID.setD(Constants.Subsystems.shooterD);
        leftShooterPID.setFF(Constants.Subsystems.shooterFF);
        leftShooterPID.setIZone(2000);

        rightShooterPID.setP(Constants.Subsystems.shooterP);
        rightShooterPID.setI(Constants.Subsystems.shooterI);
        rightShooterPID.setD(Constants.Subsystems.shooterD);
        rightShooterPID.setFF(Constants.Subsystems.shooterFF);
        rightShooterPID.setIZone(2000);
        intake = new CANSparkFlex(Constants.Subsystems.intake, CANSparkLowLevel.MotorType.kBrushless);
        SmartDashboard.setDefaultNumber("Intake", 0.5);

        SmartDashboard.putNumber("Left Shooter Ref", 5500);
        SmartDashboard.putNumber("Right Shooter Ref", 5500);
        SmartDashboard.putNumber("Left Index", 0.15);
        SmartDashboard.putNumber("Right Index", 0.15);
    }


    @Override
    public void periodic(){
        if(isFiring){
            leftShooterPID.setReference(SmartDashboard.getNumber("Left Shooter Ref", 0), ControlType.kVelocity);
            rightShooterPID.setReference(SmartDashboard.getNumber("Right Shooter Ref", 0), ControlType.kVelocity);
        }else{
            leftShooterPID.setReference(0, ControlType.kVelocity);
            rightShooterPID.setReference(0, ControlType.kVelocity);
            rightShooter.stopMotor();
            leftShooter.stopMotor();
        }

        SmartDashboard.putNumber("left velocity", leftShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("right velocity", rightShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("left power", leftShooter.getAppliedOutput());
        SmartDashboard.putNumber("right power", rightShooter.getAppliedOutput());
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

    public void IntakeIn(){
        intake.set(SmartDashboard.getNumber("Intake", 0));
    }
    public void IntakeStop(){
        intake.stopMotor();
    }

    public void stop(){ 
        rightShooter.stopMotor();
        leftShooter.stopMotor();
        isFiring=false;
    }


}
