package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.util.DynamicSlewRateLimiter;

public class Constants {

    public static class SwerveK {
        public static final Distance wheelDiameter = Inches.of(3); 
        public static final Distance driveBaseRadius = Inches.of(12.75);

        public static final double steerGearRatio = 41.25; 
        public static final double driveGearRatio = 4.4;

        public static final LinearVelocity maxRobotSpeed = MetersPerSecond.of(5.426);

        public static final PIDConstants translationConstants = new PIDConstants(0, 0, 0); //! TODO: Tune
        public static final PIDConstants rotationConstants = new PIDConstants(0, 0, 0); //! TODO: Tune
        public static RobotConfig robotConfig; static {
            try {
                robotConfig = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                e.printStackTrace();
            }
        }

        public static final PIDConstants angularPID = new PIDConstants(5, 0, 0.5); // kP = degrees/second per degree
        public static final Angle angularDeadband = Degrees.of(2);

        public static final File swerveDirectory = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/swerve");
    }

    public static class ControllerK {
        public static final int xboxPort = 0;
        public static final double leftJoystickDeadband = 0.05;
        public static final double rightJoystickDeadband = 0.05;
    }

    public static class DriveK {
        public static final DynamicSlewRateLimiter translationalYLimiter = new DynamicSlewRateLimiter(1.25, 2); // Larger number = faster rate of change
        public static final DynamicSlewRateLimiter translationalXLimiter = new DynamicSlewRateLimiter(1.25, 2);
        public static final DynamicSlewRateLimiter rotationalLimiter = new DynamicSlewRateLimiter(1.25, 2);
    }

    public static class AlgaeK { // TODO: Tune everything here
        public static final int sparkMaxID = 0;
        public static final int limitSwitchID = 0;
        public static final double gearRatio = 50;
        public static final Voltage zeroingVoltage = Volts.of(-1.5);

        public static final Angle algaePosition = Degrees.of(0);
        public static final Angle stowPosition = Degrees.of(0);
        public static final Angle zeroPosition = Degrees.of(0);
        public static final Angle allowableError = Degrees.of(0);

        public static final double kP = 0;
        public static final double kD = 0;
        public static final AngularVelocity maxVelocity = DegreesPerSecond.of(0);
        public static final AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(0);
    }
    
}
