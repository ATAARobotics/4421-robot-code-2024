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
import frc.robot.subsystems.*;





public class Shooter extends SubsystemBase{
    private boolean isFiring = false;
    private boolean isIntaking = false;
    private boolean indexing = false;
    private double indexPower = 0.10;
    private boolean finishedIntake = false;

    private boolean hasNote = true;


    private Index mIndex;
    private Intake mIntake;


    public CANSparkFlex leftShooter;
    public CANSparkFlex rightShooter;
    public SparkPIDController leftShooterPID;
    public SparkPIDController rightShooterPID;

    private DigitalInput IndexStopBack;
    private DigitalInput IndexStopFront;

    private enum IntakeLevels{
        NotRunning,
        Running, 
        SeeSensor,
        Reverse,
        Shooting
    }

    private IntakeLevels IntakeLevel = IntakeLevels.NotRunning;

    public Shooter(Index m_Index, Intake m_Intake){
        this.mIndex = m_Index;
        this.mIntake = m_Intake;

        leftShooter = new CANSparkFlex(Constants.Subsystems.leftShooter, CANSparkLowLevel.MotorType.kBrushless);
        rightShooter = new CANSparkFlex(Constants.Subsystems.rightShooter, CANSparkLowLevel.MotorType.kBrushless);
        
        
        IndexStopBack = new DigitalInput(0);
        IndexStopFront = new DigitalInput(1);

        leftShooter.setInverted(true);
        leftShooter.setIdleMode(IdleMode.kCoast);
        rightShooter.setIdleMode(IdleMode.kCoast);

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
                mIndex.stopIndex();
                mIntake.stopIntake();
                break;
            case Reverse:
                if(IndexStopBack.get()){
                    mIndex.runIndex(-indexPower);
                    mIntake.stopIntake();

                } else if(!IndexStopFront.get()) {
                    IntakeLevel = IntakeLevels.NotRunning;
                }
                break;               
            case SeeSensor:
                if(!IndexStopFront.get()){
                    mIndex.runIndex(indexPower);

                    mIntake.stopIntake();


                    hasNote = true;

                } else if (!IndexStopBack.get()) {
                    IntakeLevel = IntakeLevels.Reverse;
                }
                break;
            case Running:
                if(IndexStopBack.get()){
                    mIndex.runIndex(indexPower);

                    mIntake.runIntake(1.0);
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
        mIndex.runIndex(indexPower);


        if (isFiring) {
            hasNote = false;
        }
    }
    public void stopIndex(){
        IntakeLevel = IntakeLevels.NotRunning;
        mIndex.stopIndex();
    }

    public void ReverseIndex(){
        IntakeLevel = IntakeLevels.Shooting;
        mIndex.runIndex(-indexPower);

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
