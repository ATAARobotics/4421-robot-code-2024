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

 
/*  WORKING VALUES FOR SHOOTING
    shooters at 3600 rmp
    index at 60%    */



public class Shooter extends SubsystemBase{
    private boolean isFiring = false;

    public CANSparkFlex leftShooter;
    public CANSparkFlex rightShooter;
    public CANSparkFlex index;
    // public CANSparkFlex indexRollder;
    public SparkPIDController leftShooterPID;
    public SparkPIDController rightShooterPID;

    public int shooterInversion = 1; 

    public Shooter(){
        leftShooter = new CANSparkFlex(Constants.Subsystems.leftSide, CANSparkLowLevel.MotorType.kBrushless);
        rightShooter = new CANSparkFlex(Constants.Subsystems.rightSide, CANSparkLowLevel.MotorType.kBrushless);

        index = new CANSparkFlex(Constants.Subsystems.index, CANSparkLowLevel.MotorType.kBrushless);
        // indexRoller = new CANSparkFlex(0, CANSparkLowLevel.MotorType.kBrushed);
        // leftIndex = new CANSparkFlex(Constants.Subsystems.leftSideIndex, CANSparkLowLevel.MotorType.kBrushless);
        // rightIndex = new CANSparkFlex(Constants.Subsystems.rightSideIndex, CANSparkLowLevel.MotorType.kBrushless);
    

        leftShooterPID = leftShooter.getPIDController();
        rightShooterPID = rightShooter.getPIDController();

        SmartDashboard.setDefaultNumber("left rpm", 0);
        SmartDashboard.setDefaultNumber("right rpm",0);
        SmartDashboard.setDefaultNumber("index power", 0);

        
        SmartDashboard.setDefaultNumber("left reverse rpm", 0);
        SmartDashboard.setDefaultNumber("right reverse rpm",0);


        SmartDashboard.putNumber("left check rpm", 0);
        SmartDashboard.putNumber("right check rpm", 0);
        SmartDashboard.putNumber("left power% used", 0);
        SmartDashboard.putNumber("right power% used", 0);
        // SmartDashboard.setDefaultNumber("index power", 0);
        
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
            SmartDashboard.putNumber("right check rpm", rightShooter.getEncoder().getVelocity());
            SmartDashboard.putNumber("left check rpm", leftShooter.getEncoder().getVelocity());
            SmartDashboard.putNumber("right power% used", rightShooter.getAppliedOutput());
            SmartDashboard.putNumber("left power% used", leftShooter.getAppliedOutput());

            if(shooterInversion==1){
                double lspower = SmartDashboard.getNumber("left rpm", 0);
                double rspower = SmartDashboard.getNumber("right rpm", 0);
                leftShooterPID.setReference(lspower, ControlType.kVelocity);
                rightShooterPID.setReference(rspower, ControlType.kVelocity);   
            }else{
                double lspower = -1* SmartDashboard.getNumber("left reverse rpm", 0);
                double rspower = -1* SmartDashboard.getNumber("right reverse rpm", 0);
                leftShooterPID.setReference(lspower, ControlType.kVelocity);
                rightShooterPID.setReference(rspower, ControlType.kVelocity);   
            }
            

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
        shooterInversion = 1;
    }

    public void reverseFire(){
        isFiring = !isFiring;
        shooterInversion = -1;
    }

    public void Index(){
        double ipower = SmartDashboard.getNumber("index power", 0);
        index.set(ipower);
    }
    public void stopIndex(){
        index.stopMotor();
    }

    public void stop(){ 
        rightShooter.stopMotor();
        leftShooter.stopMotor();
        isFiring=false;
    }


}
