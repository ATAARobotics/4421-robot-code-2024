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
    private boolean isFiring = false;

    public CANSparkFlex leftShooter;
    public CANSparkFlex rightShooter;
    public CANSparkFlex indexRollder;
    public SparkPIDController leftShooterPID;
    public SparkPIDController rightShooterPID;

    public Shooter(){
        leftShooter = new CANSparkFlex(Constants.Subsystems.leftSide, CANSparkLowLevel.MotorType.kBrushless);
        rightShooter = new CANSparkFlex(Constants.Subsystems.rightSide, CANSparkLowLevel.MotorType.kBrushless);
        indexRollder = new CANSparkFlex(17, CANSparkLowLevel.MotorType.kBrushless);
        // leftIndex = new CANSparkFlex(Constants.Subsystems.leftSideIndex, CANSparkLowLevel.MotorType.kBrushless);
        // rightIndex = new CANSparkFlex(Constants.Subsystems.rightSideIndex, CANSparkLowLevel.MotorType.kBrushless);
    

        leftShooterPID = leftShooter.getPIDController();
        rightShooterPID = rightShooter.getPIDController();

        SmartDashboard.setDefaultNumber("left power", 0);
        SmartDashboard.setDefaultNumber("right power",0);

        SmartDashboard.putNumber("left rpm", 0);
        SmartDashboard.putNumber("right rpm", 0);
        SmartDashboard.putNumber("left power% used", 0);
        SmartDashboard.putNumber("right power% used", 0);
        SmartDashboard.setDefaultNumber("index power", 0);
        
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
        // intake = new CANSparkFlex(Constants.Subsystems.intake, CANSparkLowLevel.MotorType.kBrushless);
    }


    @Override
    public void periodic(){
        if(isFiring){
            SmartDashboard.putNumber("right rpm", rightShooter.getEncoder().getVelocity());
            SmartDashboard.putNumber("left rpm", leftShooter.getEncoder().getVelocity());
            SmartDashboard.putNumber("right power% used", rightShooter.getAppliedOutput());
            SmartDashboard.putNumber("left power% used", leftShooter.getAppliedOutput());


            double lspower = SmartDashboard.getNumber("left power", 4000);
            double rspower = SmartDashboard.getNumber("right power", 4000);
            leftShooterPID.setReference(lspower, ControlType.kVelocity);
            rightShooterPID.setReference(rspower, ControlType.kVelocity);

            // leftShooter.set(lspower);
            // rightShooter.set(rspower);
        }else{
            leftShooterPID.setReference(0, ControlType.kVelocity);
            rightShooterPID.setReference(0, ControlType.kVelocity);
            rightShooter.stopMotor();
            leftShooter.stopMotor();
        }
    }

    public void Fire(){
        isFiring = !isFiring;
    }

    public void Index(){
        double ipower = SmartDashboard.getNumber("index power", 0.15);
        indexRollder.set(ipower);
    }
    public void stopIndex(){
        indexRollder.stopMotor();
    }

    public void stop(){ 
        rightShooter.stopMotor();
        leftShooter.stopMotor();
        isFiring=false;
    }


}
