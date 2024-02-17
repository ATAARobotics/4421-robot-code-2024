package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class ChaseTag extends Command {

    private Swerve m_swerveDriveSubsystem;
    private Pose2d targetPose;
    private Command pathfindingCommand;

    public ChaseTag(Pose2d targetPose) {
        this.targetPose = targetPose;
    }


    @Override
    public void initialize() {
      // Create the constraints to use while pathfinding
      PathConstraints constraints = new PathConstraints(
              3.0, 4.0,
              Units.degreesToRadians(540), Units.degreesToRadians(720));

      // Since AutoBuilder is configured, we can use it to build pathfinding commands
      pathfindingCommand = AutoBuilder.pathfindToPose(
              targetPose,
              constraints,
              0.0, // Goal end velocity in meters/sec
              0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
      );
    }
    @Override
    public void execute(){
      pathfindingCommand.execute();
    }
    @Override
    public void end(boolean interrupted) {
      pathfindingCommand.cancel();
    }

}