package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class AmpScore extends Command{

    private Timer ampTimer = new Timer();
    private boolean finishAmp = false;

    private Shooter s_Shooter;
    private Index s_Index;
    public AmpScore(Shooter s_Shooter, Index s_Index){
        this.s_Shooter = s_Shooter;
        this.s_Index = s_Index;
        addRequirements(s_Shooter);
    }


    @Override
    public void execute() {
        s_Shooter.scoreAmp(s_Index);
    }

    @Override
    public boolean isFinished() {
        if (!s_Shooter.getHasNote() && !finishAmp) {
            ampTimer.start();
        }
        if (!finishAmp && ampTimer.hasElapsed(Constants.Subsystems.ampTime)) {
            ampTimer.stop();
            ampTimer.reset();
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean isInterrupted) {
        s_Shooter.stopScoreAmp(s_Index);
    }


    

}
