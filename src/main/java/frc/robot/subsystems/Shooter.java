package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;




public class Shooter extends SubsystemBase{
    public CANSparkFlex intake;
    private boolean isFiring = false;
    private boolean isIntaking = false;
    private boolean indexing = false;
    private double indexPower = 0.10;
    private boolean finishedIntake = false;

    private boolean hasNote = true;



    public CANSparkFlex leftShooter;
    public CANSparkFlex rightShooter;
    public CANSparkFlex leftIndex;
    public CANSparkFlex rightIndex;
    public SparkPIDController leftShooterPID;
    public SparkPIDController rightShooterPID;

    private DigitalInput IndexStop;

    private enum IntakeLevels{
        NotRunning,
        Running, 
        SeeSensor,
        Reverse,
        Shooting
    }

    private IntakeLevels IntakeLevel = IntakeLevels.NotRunning;

    public Shooter(){
        // leftShooter = new CANSparkFlex(Constants.Subsystems.leftSide, CANSparkLowLevel.MotorType.kBrushless);
        // rightShooter = new CANSparkFlex(Constants.Subsystems.rightSide, CANSparkLowLevel.MotorType.kBrushless);
        // leftIndex = new CANSparkFlex(Constants.Subsystems.leftSideIndex, CANSparkLowLevel.MotorType.kBrushless);
        // rightIndex = new CANSparkFlex(Constants.Subsystems.rightSideIndex, CANSparkLowLevel.MotorType.kBrushless);
        
        IndexStop = new DigitalInput(0);

        leftShooter.setInverted(true);
        leftIndex.setInverted(true);
        leftShooter.setIdleMode(IdleMode.kCoast);
        rightShooter.setIdleMode(IdleMode.kCoast);
        leftIndex.setIdleMode(IdleMode.kBrake);
        rightIndex.setIdleMode(IdleMode.kBrake);

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

        intake = new CANSparkFlex(Constants.Subsystems.intakeFront, CANSparkLowLevel.MotorType.kBrushless);
        SmartDashboard.setDefaultNumber("Intake", -1);

        SmartDashboard.putNumber("Left Shooter Ref", 5500);
        SmartDashboard.putNumber("Right Shooter Ref", 5500);
        
        
        //SmartDashboard.putNumber("Left Index", 0.10);
       // SmartDashboard.putNumber("Right Index", 0.10);
    }


    @Override
    public void periodic(){
        if(isFiring){
            leftShooterPID.setReference(5500.0, ControlType.kVelocity);
            rightShooterPID.setReference(5500.0, ControlType.kVelocity);
            // leftShooter.set(0.5);
            // rightShooter.set(0.5);
        }else{
            rightShooter.stopMotor();
            leftShooter.stopMotor();
        }
        switch (IntakeLevel){
            case NotRunning:
                leftIndex.stopMotor();
                rightIndex.stopMotor();
                intake.stopMotor();
                break;
            case Reverse:
                if(IndexStop.get()){
                    leftIndex.set(-indexPower);
                    rightIndex.set(-indexPower);
                    intake.set(0);
                } else{
                    IntakeLevel = IntakeLevels.NotRunning;
                }
                break;               
            case SeeSensor:
                if(!IndexStop.get()){
                    leftIndex.set(indexPower);
                    rightIndex.set(indexPower);
                    intake.set(0);

                    hasNote = true;

                } else{
                    IntakeLevel = IntakeLevels.Reverse;
                }
                break;
            case Running:
                if(IndexStop.get()){
                    leftIndex.set(indexPower);
                    rightIndex.set(indexPower);
                    intake.set(1.0);
                } else{
                    IntakeLevel = IntakeLevels.SeeSensor;
                }
                break;  
            case Shooting:
                break;
            default:
                break;
        }

        SmartDashboard.putNumber("left velocity", leftShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("right velocity", rightShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("left power", leftShooter.getAppliedOutput());
        SmartDashboard.putNumber("right power", rightShooter.getAppliedOutput());
    }


    public void Fire(){
        isFiring = !isFiring;
    }
    public void AutoFire(){
        isFiring = true;
    }
    public void AutoStop(){
        isFiring = false;
    }
    public void Index(){
        IntakeLevel = IntakeLevels.Shooting;
        leftIndex.set(indexPower);
        rightIndex.set(indexPower);

        if (isFiring) {
            hasNote = false;
        }
    }
    public void stopIndex(){
        IntakeLevel = IntakeLevels.NotRunning;
        leftIndex.stopMotor();
        rightIndex.stopMotor();
    }

    public void ReverseIndex(){
        IntakeLevel = IntakeLevels.Shooting;
        leftIndex.set(-indexPower);
        rightIndex.set(-indexPower);
    }

    public void IntakeIn(){
        if (IntakeLevel == IntakeLevels.NotRunning){
            IntakeLevel = IntakeLevels.Running;
        }
        else{
            IntakeLevel = IntakeLevels.NotRunning;
        }
    }

    public void StopIntake() {
        IntakeLevel = IntakeLevels.NotRunning;

    }

    public void stop(){ 
        rightShooter.stopMotor();
        leftShooter.stopMotor();
        isFiring=false;
    }

    public boolean CanShoot(){
        return(5000<leftShooter.getEncoder().getVelocity() && leftShooter.getEncoder().getVelocity()<6000);
    }

    public boolean hasNote() {
        return hasNote;
    }

}
