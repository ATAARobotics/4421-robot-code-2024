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
    final int targetAngle = 270;

    Swerve s_Swerve;
    double xDifference;
    double yDifference;
    int angleDifference;

    public Lighting(Swerve s_Swerve) {
        CANdleConfiguration configAll = new CANdleConfiguration();
        // configAll.statusLedOffWhenActive = false;
        // configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.Off;
        ledController.configAllSettings(configAll, 100);
        ledController.setLEDs(200, 200, 200);

        ledController.setLEDs(0, 0, 0, 0, 0, 128);
        
        this.s_Swerve = s_Swerve;
        
    }
    
    @Override
    public void periodic() {
            
            double[] pose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
            double poseX = -pose[0] + 8.27;
            double poseY = -pose[1] + 4.105;
            double poseR = s_Swerve.getYaw().getDegrees();
            double timeStamp = Timer.getFPGATimestamp() - (pose[6] / 1000.0);
            xDifference = targetX - poseX;
            yDifference = targetY - poseY;
        
            double distance = Math.sqrt(Math.pow(xDifference, 2) + Math.pow(yDifference, 2));
    
            angleDifference = targetAngle - (int) Math.round(poseR);
            System.out.println(Math.round(poseR));
            int distInt = 1;
            int angleColor = 0;
            // if (Math.abs(angleDifference) < 90) {
            //     if (Math.abs(distance) < 2.0) {
            //         distColor = (int) Math.round(Math.abs(distance * 127));
                    
            //     }
            //     else {
            //         distColor = 2;
            //     }
            //     int angleInt = (int) Math.round(Math.abs(angleDifference * 1.915));
            //     ledController.setLEDs(distColor, 255 - distColor, 0, 10, 0, angleInt + 65);
            // }
            // else {
            //     ledController.setLEDs(200, 200, 200, 0, 0, 65);
            // } 

            if (distance <= 1.0) {
                distInt = (int) Math.round(Math.abs(distance * 88));
                // System.out.println(distance);
            }

            angleColor = (int) Math.round(Math.abs(angleDifference * 0.7));
            double limelightid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-1.0);
             if(limelightid <= 16 && limelightid >= 1 && distance != 1.0 ){
                ledController.setLEDs(0, 0, 0, 0, distInt + 40, 128 - (distInt + 40));    
                ledController.setLEDs(angleColor, 255 - angleColor, 0, 10, 0, distInt + 40);

             
            }

            
        }

       

    

}
