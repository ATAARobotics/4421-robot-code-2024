package frc.robot.subsystems;


import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class Intake extends SubsystemBase {

    public CANSparkFlex frontIntake;
    public CANSparkFlex backIntake;

    private Index mIndex;
    private DigitalInput IndexStop;


    private enum IntakeLevels{
        NotRunning,
        Running, 
        SeeSensor,
        Reverse
    }

    private IntakeLevels IntakeLevel = IntakeLevels.NotRunning;


    public Intake(Index m_Index) {

        IndexStop = new DigitalInput(0);
        this.mIndex = m_Index;

        frontIntake = new CANSparkFlex(Constants.Subsystems.intakeFront, CANSparkLowLevel.MotorType.kBrushless);
        backIntake = new CANSparkFlex(Constants.Subsystems.intakeBack, CANSparkLowLevel.MotorType.kBrushless);

        backIntake.setInverted(true);

        frontIntake.setIdleMode(IdleMode.kCoast);
        backIntake.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void periodic() {
        switch (IntakeLevel) {
            case NotRunning:
                frontIntake.stopMotor();
                backIntake.stopMotor();
                break;
            case Reverse:
                if(IndexStop.get()){
                    mIndex.runIndex(-0.1);
                    frontIntake.stopMotor();
                backIntake.stopMotor();
                } else{
                    IntakeLevel = IntakeLevels.NotRunning;
                }
            case SeeSensor:
                if (!IndexStop.get()) {
                    frontIntake.stopMotor();
                    backIntake.stopMotor();
                } else {
                    IntakeLevel = IntakeLevels.Reverse;
                }
                break;
            case Running:
                if(IndexStop.get()){
                    frontIntake.set(0.1);
                    backIntake.set(0.1);
                } else{
                    IntakeLevel = IntakeLevels.SeeSensor;
                }
                break;
            default:
                break;
        }
    }

    public void runIntake(double intakeSpeed, double indexSpeed) {
        frontIntake.set(intakeSpeed);
        backIntake.set(intakeSpeed);
        mIndex.runIndex(indexSpeed);
    }

    public void stopIntake() {
        frontIntake.stopMotor();
        backIntake.stopMotor();
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


}
