package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends Command{
    private Intake s_Intake;
    private Index s_Index;
    private enum IntakeTypes {
        beforeAnything, 
        inTook,
        afterAnything,
        nothing,
        end
    }
    private IntakeTypes intakePoint = IntakeTypes.nothing;

    public IntakeCommand(Intake s_Intake, Index s_Index){
        this.s_Intake = s_Intake;
        this.s_Index = s_Index;
        this.intakePoint = IntakeTypes.beforeAnything;
        addRequirements(s_Intake);
    }
    @Override
    public void initialize(){
        intakePoint = IntakeTypes.beforeAnything;
    }

    @Override
    public void execute() {
        switch (intakePoint) {
            case beforeAnything:
                if(!s_Intake.isIntaked()){
                    s_Intake.runIntake(0.4);
                    s_Index.runIndex(1);
                }else{
                    intakePoint =IntakeTypes.inTook;
                }
                break;
            case inTook:
                if(s_Intake.isIntaked()){
                    s_Intake.runIntake(0.4);
                    s_Index.runIndex(1);
                }else{
                    intakePoint = IntakeTypes.afterAnything;
                }
                break; 
            case afterAnything:
                if(!s_Intake.isIntaked()){
                    s_Intake.stopIntake();;
                    s_Index.runIndex(-0.3);
                }else{
                    intakePoint = IntakeTypes.end;
                }
                break;   
            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return (intakePoint == IntakeTypes.end);
    }
    @Override
    public void end(boolean isInterrupted) {
        s_Index.stopIndex();
        s_Intake.stopIntake();
    }
}
