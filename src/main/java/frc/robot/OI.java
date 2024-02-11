package frc.robot;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

class OI {

    private BetterJoystick driveStick = new BetterJoystick(0, 1);
    private BetterJoystick rotationStick = new BetterJoystick(1, 1);
    private BetterJoystick gunnerStick = new BetterJoystick(2, 0);

    // Driver Values
    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;
    private boolean toggleFieldOriented;
    private double speed;
    private boolean override = false;
    // public Trigger RotateRight;


    public JoystickButton zeroGyro;
    public JoystickButton robotCentric;
    public JoystickButton intake;
    public JoystickButton runShooter;
    public JoystickButton runIndex;
    public JoystickButton shooterLock;
    public JoystickButton reverseIntake;
    public JoystickButton toWaypoint;

    public OI() {
        // Configure the button bindings
        try (InputStream input = new FileInputStream("/home/lvuser/deploy/bindings.properties")) {
            Properties bindings = new Properties();

            bindings.load(input);

            driveStick.configureBindings(bindings);
            rotationStick.configureBindings(bindings);
            gunnerStick.configureBindings(bindings);

            input.close();
        } catch (FileNotFoundException e) {
            DriverStation.reportError("Button bindings file not found!", false);
        } catch (IOException e) {
            DriverStation.reportError("IOException on button binding file", false);
        }
        shooterLock = rotationStick.getWPIJoystickButton("AutoShooter");
        intake = driveStick.getWPIJoystickButton("Intake");
        zeroGyro = driveStick.getWPIJoystickButton("Zero");

        reverseIntake = driveStick.getWPIJoystickButton("ReverseIntake");

        toWaypoint = driveStick.getWPIJoystickButton("ScoreAmp");

        // Set up command-based stuff
        /** 
        RotIntake = rotationStick.getWPIJoystickButton("RotIntake");
        RotIntake = rotationStick.getWPIJoystickButton("RotIntake");
        Forward = driveStick.getWPIJoystickButton("Forward");
        AutoBalance = driveStick.getWPIJoystickButton("AutoBalance");
        OverridePivotUp = driveStick.getWPIJoystickButton("OverridePivotUp");
        OverridePivotUp = driveStick.getWPIJoystickButton("OverridePivotUp");
        ResetOdo = driveStick.getWPIJoystickButton("ResetOdo");

        PivotUp = gunnerStick.getWPIJoystickButton("PivotUp");
        PivotDown = gunnerStick.getWPIJoystickButton("PivotDown");
        IntakeIn = gunnerStick.getWPIJoystickButton("IntakeIn");
        IntakeOut = gunnerStick.getWPIJoystickButton("IntakeOut");
        TelescopingOut = gunnerStick.getWPIJoystickButton("TelecopingOut");
        TelescopingIn = gunnerStick.getWPIJoystickButton("TelecopingIn");
        DownToStop = gunnerStick.getWPIJoystickButton("DownToStop");
        LightSwitch = gunnerStick.getWPIJoystickButton("LightSwitch");
        DownToStop = gunnerStick.getWPIJoystickButton("DownToStop");
        LightSwitch = gunnerStick.getWPIJoystickButton("LightSwitch");
        SlideLeft = gunnerStick.getDPadTrigger("SlideLeft");
        SlideRight = gunnerStick.getDPadTrigger("SlideRight");
        RotateLeft = gunnerStick.getDPadTrigger("RotateLeft");
        RotateRight = gunnerStick.getDPadTrigger("RotateRight");
        */
    }

    public void rumbleGunnerOn() {
        gunnerStick.setRumble(1);
    }

    public void rumbleGunnerOff() {
        gunnerStick.setRumble(0);
    }

    public void checkInputs() {
        xVelocity = driveStick.getAnalog("XVelocity");
        yVelocity = driveStick.getAnalog("YVelocity");
        rotationVelocity = rotationStick.getAnalog("XVelocity");
        //rotationVelocity = driveStick.getAnalog("RotationVelocity");

        speed = (-driveStick.getAnalog("Speed") + 1) / 4 + 0.5;

        // Dead zones
        if (Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2)) < Constants.Swerve.JOY_DEAD_ZONE) {
            xVelocity = 0;
            yVelocity = 0;
        }
        if (Math.abs(rotationVelocity) < Constants.Swerve.JOY_DEAD_ZONE) {
            rotationVelocity = 0;
        }
        // kinda crimnal
        xVelocity = Math.signum(xVelocity) * Math.abs(Math.pow(xVelocity, Constants.Swerve.JOYSTICK_SENSITIVITY));
        yVelocity = Math.signum(yVelocity) * Math.abs(Math.pow(yVelocity, Constants.Swerve.JOYSTICK_SENSITIVITY));
        rotationVelocity = Math.signum(rotationVelocity)
                * Math.abs(Math.pow(rotationVelocity, Constants.Swerve.TURNING_SENSITIVITY));

        toggleFieldOriented = driveStick.getButton("ToggleFieldOriented");
    }

    // Getter functions for controls
    public double getXVelocity() {
        return -yVelocity;
    }

    public double getYVelocity() {
        return -xVelocity;
    }

    public double getSpeed() {
        return speed;
    }

    public double getRotationVelocity() {
        return -rotationVelocity;
    }

    public boolean getToggleFieldOriented() {
        return toggleFieldOriented;
    }

    public boolean getOverride(){
        return !override;
    }
    public boolean notgetOverride(){
        return override;
    }
    public double getOuttake(){
        return gunnerStick.getAnalog("Override");
    }
    public double getOuttakeInversed(){
        return gunnerStick.getAnalog("Quick");
    }

}
