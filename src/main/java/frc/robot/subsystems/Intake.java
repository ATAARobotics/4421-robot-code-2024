package frc.robot.subsystems;


import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    public CANSparkFlex frontIntake;
    public CANSparkFlex backIntake;


    public Intake() {
        frontIntake = new CANSparkFlex(Constants.Subsystems.frontIntake, CANSparkLowLevel.MotorType.kBrushless);
        backIntake = new CANSparkFlex(Constants.Subsystems.backIntake, CANSparkLowLevel.MotorType.kBrushless);

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


}
