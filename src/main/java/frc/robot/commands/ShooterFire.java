package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterFire extends Command{
    
    private Shooter s_Shooter;
    public ShooterFire(Shooter s_Shooter){
        this.s_Shooter = s_Shooter;
        addRequirements(s_Shooter);
    }

    @Override
    public void execute(){
        s_Shooter.Fire();
    }
    // @Override
    // public void initialize() {
    //     s_Shooter.grabHatch();
    // }

    @Override
    public boolean isFinished() {
        return true;
    }
}
