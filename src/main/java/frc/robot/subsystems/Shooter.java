package frc.robot.subsystems;
import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;





public class Shooter extends SubsystemBase{
    private boolean isFiring = false;
    private boolean isLob = false;
    private boolean isIntaking = false;
    private boolean indexing = false;
    private double indexPower = 0.75;
    private boolean finishedIntake = false;

    private boolean atAmpPoint = false;
    private int isAmpScoring = 0;

    private boolean hasNote = true;

    public CANSparkFlex leftShooter;
    public CANSparkFlex rightShooter;
    public SparkPIDController leftShooterPID;
    public SparkPIDController rightShooterPID;

    

    private DigitalInput AmpStop;

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

        AmpStop = new DigitalInput(9);

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
        // leftShooterPID.setIZone(1000);
        // rightShooterPID.setIZone(1000);
        

        // SmartDashboard.putNumber("left velocity", leftShooter.getEncoder().getVelocity());
        // SmartDashboard.putNumber("right velocity", rightShooter.getEncoder().getVelocity());
        // SmartDashboard.putNumber("left power", leftShooter.getAppliedOutput());
        // SmartDashboard.putNumber("right power", rightShooter.getAppliedOutput());

        
        SmartDashboard.setDefaultNumber("shooter p", 0);
        SmartDashboard.setDefaultNumber("shooter i", 0);
        SmartDashboard.setDefaultNumber("shooter d", 0);
        SmartDashboard.setDefaultNumber("shooter ff", 0);
        
        // rightShooterPID.setIZone(2000);
        

        SmartDashboard.putNumber("Left Shooter Ref", Constants.Subsystems.shooterSetPoint);
        SmartDashboard.putNumber("Right Shooter Ref", Constants.Subsystems.shooterSetPoint);
        SmartDashboard.setDefaultBoolean("SHOOTER REVVED", false);
        
        
        //SmartDashboard.putNumber("Left Index", 0.10);
       // SmartDashboard.putNumber("Right Index", 0.10);
    }


    @Override
    public void periodic(){

        SmartDashboard.putNumber("rpm l", leftShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("rpm r", rightShooter.getEncoder().getVelocity());



        SmartDashboard.putBoolean("SHOOTER REVVED", (leftShooter.getEncoder().getVelocity() > 5000));

        if(isFiring && isAmpScoring == 0){
            if (!isLob) {
                leftShooterPID.setReference(Constants.Subsystems.shooterSetPoint, ControlType.kVelocity);
                // leftShooter.set(1);
                rightShooterPID.setReference(Constants.Subsystems.shooterSetPointAlt, ControlType.kVelocity);                
            }else{
            leftShooterPID.setReference(Constants.Subsystems.shooterSetPoint*0.75, ControlType.kVelocity);
            // leftShooter.set(1);
            rightShooterPID.setReference(Constants.Subsystems.shooterSetPointAlt*0.75, ControlType.kVelocity);
            }

            // leftShooterPID.setReference(SmartDashboard.getNumber("Left Shooter Ref", 0), ControlType.kVelocity);
            // rightShooterPID.setReference(SmartDashboard.getNumber("Right Shooter Ref", 0), ControlType.kVelocity);
            // leftShooter.set(SmartDashboard.getNumber("left shooter p%", 0));
            // rightShooter.set(SmartDashboard.getNumber("right shooter p%", 0));

        }else if (isAmpScoring == 0){
            rightShooter.stopMotor();
            leftShooter.stopMotor();
        } else{
            switch (isAmpScoring) {
                case 1:
                    if(AmpStop.get()){
                        leftShooter.set(0.2);
                        rightShooter.set(0.2);
                     }else{
                        isAmpScoring = 2;
                    }
                    break;
                case 2:
                    if(!AmpStop.get()){
                        leftShooter.set(0.07);
                        rightShooter.set(0.07);
                     }else{
                        isAmpScoring = 4;
                        leftShooter.stopMotor();
                        rightShooter.stopMotor();
                    }                    
                    break;
                case 3:
                    leftShooter.set(-0.25);
                    rightShooter.set(-0.25);                
                    break;
                default:
                    break;
            }
        }
    }


    public void scoreAmp(Index sIndex, Pivot sPivot){
        if(isAmpScoring == 0){
            sPivot.toSetpoint(80); 
            isAmpScoring = 5;
        }else{
            if(isAmpScoring != 4){
                if (isAmpScoring != 3 && isAmpScoring != 1){
                    isAmpScoring = 1;
                    sIndex.index.set(1); 
                    sPivot.toSetpoint(114); 
                }
                else{
                    sPivot.toSetpoint(Constants.Subsystems.pivotMin);
                    sIndex.index.set(0);
                    leftShooter.set(0);
                    rightShooter.set(0);
                    isAmpScoring = 0;
                }
            }
            else{
                    isAmpScoring = 3;
            }
        }

 
    }
    public void stopScoreAmp(Index sIndex){
        isAmpScoring = 0;
        sIndex.index.set(SmartDashboard.getNumber("right index p%", 0));
        leftShooter.set(SmartDashboard.getNumber("left shooter p% a", 0));
        rightShooter.set(SmartDashboard.getNumber("right shooter p% a", 0));
    }

    public boolean getHasNote() {
        return hasNote;
    }


    public void Fire(){
        isFiring = !isFiring;
        rightShooterPID.setP(SmartDashboard.getNumber("shooter p", 0));
        rightShooterPID.setI(SmartDashboard.getNumber("shooter i", 0));
        rightShooterPID.setD(SmartDashboard.getNumber("shooter d", 0));
        leftShooterPID.setP(SmartDashboard.getNumber("shooter p", 0));
        leftShooterPID.setI(SmartDashboard.getNumber("shooter i", 0));
        leftShooterPID.setD(SmartDashboard.getNumber("shooter d", 0));

        rightShooterPID.setFF(SmartDashboard.getNumber("shooter ff", 0));
        leftShooterPID.setFF(SmartDashboard.getNumber("shooter ff", 0));
    }
    public void AutoFire(){
        isFiring = true;
        isLob = false;
    }
    public void LobFire(){
        isFiring = true;
        isLob = true;
    }
    public void AutoStop(){
        isFiring = false;
        isLob = false;

    }
    public void Index(){
        IntakeLevel = IntakeLevels.Shooting;

        if (isFiring) {
            hasNote = false;
        }
    }
    public void stopIndex(){
        IntakeLevel = IntakeLevels.NotRunning;
        isAmpScoring = 0;
    }
    public double getRPM(){
        return leftShooter.getEncoder().getVelocity();
    }
    public void ReverseIndex(){
        IntakeLevel = IntakeLevels.Shooting;
        isAmpScoring = 3;
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
        return(Math.abs(leftShooter.getEncoder().getVelocity()-Constants.Subsystems.shooterSetPoint) < Constants.Subsystems.shooterTolerance);
    }

    public boolean hasNote() {

        return hasNote;
    }


}
