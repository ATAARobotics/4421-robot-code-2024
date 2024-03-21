package frc.robot.subsystems;

import java.sql.Driver;
import java.time.Period;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class Lighting extends SubsystemBase{

    double targetX;
    double targetY;
    double targetRot;

    double diffX;
    double diffY;
    double diffRot;

    double distXY;
    int distColor;
    int ledCount;

    double distanceLimit;
    double rotationLimit;

    ColorFlowAnimation redFlow;
    ColorFlowAnimation greenFlow;

    double speed;

    Swerve sSwerve;
    Shooter sShooter;

    public boolean isEnabled = DriverStation.isEnabled();

    public CANdle candle = new CANdle(Constants.Subsystems.CandleID);

    public Lighting(Swerve s_Swerve, Shooter s_Shooter) {
        ledCount = 7 + 40; // 7 initial LEDs on the CANdle


        distColor = 255;
        distanceLimit = 2.0; // Meters
        rotationLimit = 5; // Degrees

        speed = 0.2;

        this.sSwerve = s_Swerve;
        this.sShooter = s_Shooter;
        
        redFlow = new ColorFlowAnimation(255, 0, 0, 0, speed, ledCount, Direction.Forward);
        greenFlow = new ColorFlowAnimation(0, 255, 0, 0, speed, ledCount, Direction.Backward);


        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1;
        config.vBatOutputMode = VBatOutputMode.Off;
        candle.configAllSettings(config, 100);

        candle.setLEDs(0, 0, 0, 0, 0, ledCount);

    }

    @Override
    public void periodic() {

        if (isEnabled) {
            if (sShooter.hasNote()) {
                candle.animate(redFlow);
            }
            else {
                candle.animate(greenFlow);

            }
        }


        /* 
         * When the robot's pose is not within either of the ranges, the LEDs will be red
         * The color will vary from red to green if the rotation of the robot is within rotationLimit
         * If the rotation is not met, but is within distanceLimit, the color will go from red to blue
        */


        
    }

    public void setLights(int r, int g, int b, int count) {
        candle.setLEDs(r, g, b, 0, 0, count);
    }
}
