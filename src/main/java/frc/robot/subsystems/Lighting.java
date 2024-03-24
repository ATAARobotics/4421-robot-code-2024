package frc.robot.subsystems;

import java.sql.Driver;
import java.time.Period;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
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

    ColorFlowAnimation flowLeft;
    ColorFlowAnimation flowRight;
    RainbowAnimation rainbowLeft;
    RainbowAnimation rainbowRight;
    double speed;
    DoubleSupplier s_speed;

    boolean intake;
    boolean enabled;

    Swerve sSwerve;
    Intake sIntake;

    public boolean isEnabled = DriverStation.isEnabled();

    public CANdle candle = new CANdle(Constants.Subsystems.CandleID);

    public Lighting(Swerve s_Swerve, Intake sIntake, DoubleSupplier s_speed) {
        ledCount = 32; // 7 initial LEDs on the CANdle


        distColor = 255;
        distanceLimit = 2.0; // Meters
        rotationLimit = 5; // Degrees
        this.s_speed = s_speed;

        this.sSwerve = s_Swerve;
        this.sIntake = sIntake;
        
        flowLeft = new ColorFlowAnimation(255, 0, 0, 0, speed, ledCount, Direction.Forward, 0);
        flowRight = new ColorFlowAnimation(255, 0, 0, 0, speed, ledCount, Direction.Backward, 40);
        rainbowLeft = new RainbowAnimation(1, 0.2, 32, false, 0);
        rainbowRight = new RainbowAnimation(1, 0.2, 32, true, 40);

        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1;
        config.vBatOutputMode = VBatOutputMode.Off;
        candle.configAllSettings(config, 100);

        candle.setLEDs(0, 0, 0, 0, 0, ledCount);
        intake = !sIntake.isIntaked();
        enabled = true;
    }

    @Override
    public void periodic() {
        speed = s_speed.getAsDouble();
        flowLeft.setSpeed(speed);
        flowRight.setSpeed(speed);
        rainbowLeft.setSpeed(speed);
        rainbowRight.setSpeed(speed);
        if (DriverStation.isEnabled()) {
            if (sIntake.isIntaked() && (intake != sIntake.isIntaked())) {
                flowLeft = new ColorFlowAnimation(0, 255, 0, 0, speed, ledCount, Direction.Forward, 0);
                flowRight = new ColorFlowAnimation(255, 255, 0, 0, speed, ledCount, Direction.Backward, 40);
                candle.animate(flowLeft);
                candle.animate(flowRight);
                intake = sIntake.isIntaked();
            }
            else if (sIntake.isIntaked() == false && (intake != sIntake.isIntaked())) {
                flowLeft = new ColorFlowAnimation(255, 0, 0, 0, speed, ledCount, Direction.Forward, 0);
                flowRight = new ColorFlowAnimation(255, 0, 0, 0, speed, ledCount, Direction.Backward, 40);
                candle.animate(flowLeft);
                candle.animate(flowRight);
                intake = sIntake.isIntaked();
            }
            enabled = true;
        } else if (enabled == true) {
            candle.animate(rainbowLeft);
            candle.animate(rainbowRight);
            enabled = false;
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
