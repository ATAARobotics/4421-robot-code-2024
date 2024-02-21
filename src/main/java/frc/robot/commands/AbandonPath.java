package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.lime;

public class AbandonPath extends Command {

    public static double boxlength;
    public static double boxwidth;
    public static double lenpct;
    public static double widpct;

    public static double adj;
    public static double dist;

    public Command a_AbandonPath(BooleanSupplier pathFailed, String goalRoutePath,
            String alternateRoutePath, Swerve drive) {
        addRequirements(drive);

        PathPlannerPath path;
        // Load the path you want to follow using its name in the GUI

        if (pathFailed.getAsBoolean()) {
            path = PathPlannerPath.fromPathFile(alternateRoutePath);
        } else {
            path = PathPlannerPath.fromPathFile(goalRoutePath);
        }
        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        // return AutoBuilder.followPath(path);

        return b_AbandonPath(path, drive);
    }

    public Command b_AbandonPath(PathPlannerPath path /* String pathURL */, Swerve m_Swerve) {
        // PathPlannerPath path = PathPlannerPath.fromPathFile(pathURL);

        return new FollowPathHolonomic(
                path,
                m_Swerve::getPose, // Robot pose supplier
                m_Swerve::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                m_Swerve::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.03, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    // var alliance = DriverStation.getAlliance();
                    // if (alliance.isPresent()) {
                    // return alliance.get() == DriverStation.Alliance.Red;
                    // }

                    return false;
                },
                m_Swerve // Reference to this subsystem to set requirements
        );
    }

    public static boolean avoidBot(double distancelimit, double distance) {

        boolean avoid;

        if (dist < distance) {
            avoid = true;
        } else {
            avoid = false;
        }

        return distancelimit > distance;
    }
}
