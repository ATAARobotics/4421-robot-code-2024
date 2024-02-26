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




public class Climber extends SubsystemBase{
    public CANSparkFlex intake;
    private boolean isFiring = false;
    private boolean isIntaking = false;
    private boolean indexing = false;
    private double indexPower = 0.10;
    private boolean finishedIntake = false;

    private boolean hasNote = true;



    public CANSparkMax leftClimb;
    public CANSparkMax rightClimb;


    private DigitalInput IndexStop;



    public Climber(){
        leftClimb = new CANSparkMax(Constants.Subsystems.leftClimb, CANSparkLowLevel.MotorType.kBrushless);
        rightClimb = new CANSparkMax(Constants.Subsystems.rightClimb, CANSparkLowLevel.MotorType.kBrushless);
        leftClimb.getEncoder().setPosition(0);
        rightClimb.getEncoder().setPosition(0);
    }


    @Override
    public void periodic(){
        // SmartDashboard.putNumber("Left Climb Position", leftClimb.getEncoder().getPosition());
        // SmartDashboard.putNumber("Right Climb Position", rightClimb.getEncoder().getPosition());

    }
    public void ClimbUp(){
        leftClimb.set(0.75);
        rightClimb.set(0.75);
    }
    public void stopClimb(){

    }
    public boolean hasNote() {
        return hasNote;
    }

}
