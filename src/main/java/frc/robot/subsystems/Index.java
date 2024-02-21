package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase{
    public CANSparkMax index;


    public Index() {
        index = new CANSparkMax(Constants.Subsystems.index, CANSparkLowLevel.MotorType.kBrushless);
        index.setIdleMode(IdleMode.kBrake);
    }

    public void runIndex(double speed) {
        index.set(speed);
    }

    public void stopIndex() {
        index.stopMotor();
    }

}
