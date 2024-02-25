package frc.robot.subsystems;


import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    public CANSparkFlex frontIntake;
    public CANSparkFlex backIntake;
    private DigitalInput intakeStop;


    public Intake() {
        frontIntake = new CANSparkFlex(Constants.Subsystems.intakeFront, CANSparkLowLevel.MotorType.kBrushless);
        backIntake = new CANSparkFlex(Constants.Subsystems.intakeBack, CANSparkLowLevel.MotorType.kBrushless);
        intakeStop = new DigitalInput(8);
        backIntake.setInverted(true);

        frontIntake.setIdleMode(IdleMode.kCoast);
        backIntake.setIdleMode(IdleMode.kCoast);
    }

    public void runIntake(double speed) {
        frontIntake.set(speed);
        backIntake.set(speed);
    }

    public void stopIntake() {
        frontIntake.stopMotor();
        backIntake.stopMotor();
    }

    public boolean isIntaked(){
        return !intakeStop.get();
    }


}
