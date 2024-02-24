package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
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
import frc.robot.commands.AmpScore;
import frc.robot.subsystems.*;





public class Shooter extends SubsystemBase{
    private boolean isFiring = false;
    private boolean isIntaking = false;
    private boolean indexing = false;
    private double indexPower = 0.75;
    private boolean finishedIntake = false;

    private boolean atAmpPoint = false;
    private boolean isAmpScoring = false;

    private boolean hasNote = true;

    public CANSparkFlex leftShooter;
    public CANSparkFlex rightShooter;
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
        leftShooter = new CANSparkFlex(Constants.Subsystems.leftShooter, CANSparkLowLevel.MotorType.kBrushless);
        rightShooter = new CANSparkFlex(Constants.Subsystems.rightShooter, CANSparkLowLevel.MotorType.kBrushless);
        
        SmartDashboard.setDefaultNumber("left shooter p%", 0);
        SmartDashboard.setDefaultNumber("right shooter p%", 0);
        SmartDashboard.setDefaultNumber("right shooter p% a", 0);
        SmartDashboard.setDefaultNumber("left shooter p% a", 0);

        SmartDashboard.setDefaultNumber("shooter ref", 0);

        SmartDashboard.setDefaultNumber("index p%", 0);

        IndexStop = new DigitalInput(0);

        leftShooter.setInverted(false);
        rightShooter.setInverted(true);

        leftShooter.setIdleMode(IdleMode.kCoast);
        rightShooter.setIdleMode(IdleMode.kCoast);

        leftShooterPID = leftShooter.getPIDController();
        rightShooterPID = rightShooter.getPIDController();

        leftShooterPID.setP(Constants.Subsystems.shooterP);
        leftShooterPID.setI(Constants.Subsystems.shooterI);
        leftShooterPID.setD(Constants.Subsystems.shooterD);
        leftShooterPID.setFF(Constants.Subsystems.shooterFF);

        rightShooterPID.setP(Constants.Subsystems.shooterP);
        rightShooterPID.setI(Constants.Subsystems.shooterI);
        rightShooterPID.setD(Constants.Subsystems.shooterD);
        rightShooterPID.setFF(Constants.Subsystems.shooterFF);
        

        // SmartDashboard.putNumber("left velocity", leftShooter.getEncoder().getVelocity());
        // SmartDashboard.putNumber("right velocity", rightShooter.getEncoder().getVelocity());
        // SmartDashboard.putNumber("left power", leftShooter.getAppliedOutput());
        // SmartDashboard.putNumber("right power", rightShooter.getAppliedOutput());

        
        SmartDashboard.setDefaultNumber("shooter p", 0);
        SmartDashboard.setDefaultNumber("shooter i", 0);
        SmartDashboard.setDefaultNumber("shooter d", 0);
        SmartDashboard.setDefaultNumber("shooter ff", 0);
        
        // rightShooterPID.setIZone(2000);
        

        SmartDashboard.putNumber("Left Shooter Ref", 5500);
        SmartDashboard.putNumber("Right Shooter Ref", 5500);
        
        
        //SmartDashboard.putNumber("Left Index", 0.10);
       // SmartDashboard.putNumber("Right Index", 0.10);
    }


    @Override
    public void periodic(){

        SmartDashboard.putNumber("rpm l", leftShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("rpm r", rightShooter.getEncoder().getVelocity());

        if(isFiring && !isAmpScoring){
            leftShooterPID.setReference(5500.0, ControlType.kVelocity);
            rightShooterPID.setReference(5500.0, ControlType.kVelocity);
            // leftShooterPID.setReference(SmartDashboard.getNumber("Left Shooter Ref", 0), ControlType.kVelocity);
            // rightShooterPID.setReference(SmartDashboard.getNumber("Right Shooter Ref", 0), ControlType.kVelocity);
            // leftShooter.set(SmartDashboard.getNumber("left shooter p%", 0));
            // rightShooter.set(SmartDashboard.getNumber("right shooter p%", 0));

        }else{
            rightShooter.stopMotor();
            leftShooter.stopMotor();
        }
    }


    public void scoreAmp(Index sIndex){
        isAmpScoring = true;
        if(!atAmpPoint){
            sIndex.index.set(SmartDashboard.getNumber("left index p%", 0));
            leftShooter.set(SmartDashboard.getNumber("left shooter p% a", 0));
            rightShooter.set(SmartDashboard.getNumber("right shooter p% a", 0));
        }else{
            sIndex.index.set(SmartDashboard.getNumber("right index p%", 0));
            leftShooter.set(-SmartDashboard.getNumber("left shooter p% a", 0));
            rightShooter.set(-SmartDashboard.getNumber("right shooter p% a", 0));
        }
    }
    public void stopScoreAmp(Index sIndex){
        isAmpScoring= false;
        sIndex.index.set(SmartDashboard.getNumber("right index p%", 0));
        leftShooter.set(SmartDashboard.getNumber("left shooter p% a", 0));
        rightShooter.set(SmartDashboard.getNumber("right shooter p% a", 0));
    }

    public boolean getHasNote() {
        return hasNote;
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

        if (isFiring) {
            hasNote = false;
        }
    }
    public void stopIndex(){
        IntakeLevel = IntakeLevels.NotRunning;
    }

    public void ReverseIndex(){
        IntakeLevel = IntakeLevels.Shooting;

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
