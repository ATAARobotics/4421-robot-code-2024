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
            
            double[] pose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
            double poseX = -pose[0] + 8.27;
            double poseY = -pose[1] + 4.105;
            Rotation2d poseR = Rotation2d.fromDegrees(pose[5] - 180);
            double timeStamp = Timer.getFPGATimestamp() - (pose[6] / 1000.0);
            xDifference = targetX - poseX;
            yDifference = targetY - poseY;
        
            double distance = Math.sqrt(Math.pow(xDifference, 2) + Math.pow(yDifference, 2));
    
            angleDifference = targetAngle - poseR.getDegrees();
    
            int distColor;
            if (Math.abs(angleDifference) < 90) {
                if (Math.abs(distance) < 2.0) {
                    distColor = (int) Math.round(Math.abs(distance * 127));
                    
                }
                else {
                    distColor = 2;
                }
                int angleInt = (int) Math.round(Math.abs(angleDifference * 1.915));
                ledController.setLEDs(distColor, 255 - distColor, 0, 10, 0, angleInt + 65);
            }
            else {
                ledController.setLEDs(200, 200, 200, 0, 0, 65);
            }
        }

       

    

}
