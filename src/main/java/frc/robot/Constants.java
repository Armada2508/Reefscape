package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.util.DynamicSlewRateLimiter;

public class Constants {

    public static class SwerveK {
        public static final Distance wheelDiameter = Inches.of(3); 
        public static final Distance driveBaseRadius = Meters.of(0.4579874); //? Verify

        //? Either make this variable correct or remove it, right now it's being used in a calculation that needs it to be one
        public static final double steerGearRatio = 1; 
        public static final double driveGearRatio = 4.4;

        public static final LinearVelocity maxRobotSpeed = MetersPerSecond.of(4.24); //? Recalculate with krakens

        public static final PIDConstants translationConstants = new PIDConstants(1, 1, 1); //! TODO: Tune
        public static final PIDConstants rotationConstants = new PIDConstants(1, 1, 1); //! TODO: Tune
        public static RobotConfig robotConfig; static {
            try {
                robotConfig = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                e.printStackTrace();
            }
        }

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
        public static final int elevatorID = 0; // ! make correct
        public static final double gearRatio = 16;

        public static final Distance stowHeight = Inches.of(0); //! Find
        public static final Distance intakeHeight = Inches.of(0);
        
        public enum Positions {
            L1(Field.levelOneHeight), //! Possible tune
            L2(Field.levelTwoHeight),
            L3(Field.levelThreeHeight),
            L4(Field.levelFourHeight),
            ALGAE_LOW(Field.algaeLowHeight),
            ALGAE_HIGH(Field.algaeHighHeight),
            STOW(ElevatorK.stowHeight), //! Find
            INTAKE(ElevatorK.intakeHeight); //! Find
    
            public final Distance level;
    
            Positions(Distance position) {
                this.level = position;
            }
        }
    }
    
}
