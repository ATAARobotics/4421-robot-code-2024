package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ChaseTag;


public class ChaseTagSubsystem extends SubsystemBase{
    
    ChaseTag chaseTag;

    public ChaseTagSubsystem(Swerve m_Swerve, Pose2d targetPose){
        this.chaseTag = new ChaseTag(m_Swerve, targetPose, false);
    }

    public void runChaseTag(){
        chaseTag.initialize();
        chaseTag.execute();
    }


}
