package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase{
    public CANSparkFlex leftIndex;
    public CANSparkFlex rightIndex;


    public Index() {
        leftIndex = new CANSparkFlex(Constants.Subsystems.leftSideIndex, CANSparkLowLevel.MotorType.kBrushless);
        rightIndex = new CANSparkFlex(Constants.Subsystems.rightSideIndex, CANSparkLowLevel.MotorType.kBrushless);
    

        leftIndex.setInverted(true);

        leftIndex.setIdleMode(IdleMode.kBrake);
        rightIndex.setIdleMode(IdleMode.kBrake);
    }

    public void runIndex(double speed) {
        leftIndex.set(speed);
        rightIndex.set(speed);
    }

    public void stopIndex() {
        leftIndex.stopMotor();
        rightIndex.stopMotor();
    }

}
