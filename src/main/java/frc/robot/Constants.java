package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.io.File;
import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.util.DynamicSlewRateLimiter;

public class Constants {

    public static class SwerveK {
        public static final Distance wheelDiameter = Inches.of(3); 
        public static final Distance driveBaseRadius = Meters.of(0.4579874); //? Verify

        public static final double steerGearRatio = 41.25; 
        public static final double driveGearRatio = 4.4;

        public static final LinearVelocity maxRobotSpeed = MetersPerSecond.of(4.24); //? Recalculate with krakens

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

    public static class ElevatorK {
        // Motor ID's
        public static final int elevatorID = 0; // ! make correct
        public static final int followID = 0; // ! make correct

        public static final double gearRatio = 16;

        public static final Distance wheelDiameter = Inches.of(0); //! Find

        // Motion Magic Values
        public static final LinearAcceleration acceleration = MetersPerSecondPerSecond.of(0); //? do we want linear
        public static final LinearVelocity velocity = MetersPerSecond.of(0); //? same question

        // Elevator-relative heights
        public static final Distance stowHeight = Inches.of(0); //! Find
        public static final Distance intakeHeight = Inches.of(0); //! Find
        public static final Distance allowableError = Inches.of(0); //! Find

        // Height Offsets
        public static final Distance reefOffset = Inches.of(0); //! Find
        public static final Distance algaeOffset = Inches.of(0); //! Find

        // PID Constants
        public static final int kP = 0; //! Find
        public static final int kD = 0; //! Find

        // Configs
        public static final FeedbackConfigs elevatorConfig = new FeedbackConfigs().withSensorToMechanismRatio(gearRatio);
        public static final Slot0Configs pidConfig = new Slot0Configs().withKP(kP).withKD(kD);

        // Set Height Positions
        public enum Positions {
            L1(Field.levelOneHeight.plus(reefOffset)), //! Possible tune
            L2(Field.levelTwoHeight.plus(reefOffset)),
            L3(Field.levelThreeHeight.plus(reefOffset)),
            L4(Field.levelFourHeight.plus(reefOffset)),
            ALGAE_LOW(Field.algaeLowHeight.plus(reefOffset)),
            ALGAE_HIGH(Field.algaeHighHeight.plus(reefOffset)),
            STOW(ElevatorK.stowHeight), //! Find
            INTAKE(ElevatorK.intakeHeight); //! Find
    
            public final Distance level;
    
            Positions(Distance position) {
                this.level = position;
            }
        }
    }  
}
