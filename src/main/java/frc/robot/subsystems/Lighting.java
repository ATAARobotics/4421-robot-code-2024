package frc.robot.subsystems;

import frc.robot.Robot;


import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lighting extends SubsystemBase {
    CANdle ledController = new CANdle(21);

    double targetX = 8.0;
    double targetY = 4.0;
    double targetAngle = 0.0;

    Swerve s_Swerve;
    double xDifference;
    double yDifference;
    double angleDifference;

    public Lighting(Swerve s_Swerve) {
        CANdleConfiguration configAll = new CANdleConfiguration();
        // configAll.statusLedOffWhenActive = false;
        // configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.Off;
        ledController.configAllSettings(configAll, 100);
        ledController.setLEDs(200, 200, 200);
        
        this.s_Swerve = s_Swerve;
        
    }
    
    @Override
    public void periodic() {

        if(Robot.isTeleop) {

            boolean hasNote = NetworkTableInstance.getDefault().getTable("sense").getEntry("note").getBoolean(false);
            if(hasNote) {
                ledController.setLEDs(0, 255, 0);
            }
            else {
                ledController.setLEDs(0, 255, 255);
            }
        } 
        else {
            s_Swerve.getPose();

            xDifference = targetX - s_Swerve.getPose().getX();
    
            if (Math.abs(xDifference) < 2.0) {
                int xColor = (int) Math.round(Math.abs(xDifference * 127));
                ledController.setLEDs(xColor, 255 - xColor, 0, 10, 0, 45);
            }
            else {
                ledController.setLEDs(200, 200, 200);
            }
    
            yDifference = targetY - s_Swerve.getPose().getY();
    
            if (Math.abs(yDifference) < 2.0) {
                int yColor = (int) Math.round(Math.abs(yDifference * 127));
                ledController.setLEDs(yColor, 255 - yColor, 0, 10, 46, 90);
            }
            else {
                ledController.setLEDs(200, 200, 200);
            }
    
            angleDifference = targetAngle - s_Swerve.getPose().getRotation().getDegrees();
    
            if (Math.abs(angleDifference) < 90) {
                int angleColor = (int) Math.round(Math.abs(angleDifference * 2.83));
                ledController.setLEDs(angleColor, 255 - angleColor, 0, 10, 91, 135);
            }
            else {
                ledController.setLEDs(200, 200, 200);
            }
        }

       
    }

    

}
