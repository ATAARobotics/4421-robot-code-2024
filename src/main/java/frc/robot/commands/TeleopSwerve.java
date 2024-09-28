package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private DoubleSupplier rotationAbs;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier slowButton;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);
  private final PIDController rotController = new PIDController(10, 20, 1);
  private boolean absHeading = false;

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      DoubleSupplier rotationAbs,
      BooleanSupplier slowButton) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.rotationAbs = rotationAbs;
    this.slowButton = slowButton;
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    rotController.setIZone(Math.toRadians(10));
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double rotationVal = 0;
    // if(absHeading){
    //     double Heading = Math.atan2(rotationAbs.getAsDouble(), rotationSup.getAsDouble());
    //     rotController.setSetpoint(Heading);
    //     rotationVal = MathUtil.clamp(rotController.calculate(s_Swerve.getPose().getRotation().getRadians()), -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity);


    // }else{
    //   rotationVal =
    //     rotationLimiter.calculate(
    //         MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    // }
    rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    }
    rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));


    /* Drive */
    if(!slowButton.getAsBoolean()){
      s_Swerve.drive(
          new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
          rotationVal * Constants.Swerve.maxAngularVelocity,
          true,
          true);
    }else{
      s_Swerve.drive(
          new Translation2d(translationVal, strafeVal).times(1.5),
          rotationVal * Math.PI,
          true,
          true);
    }
  }
}
