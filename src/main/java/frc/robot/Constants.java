package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.util.DynamicSlewRateLimiter;
import frc.robot.lib.util.Encoder;

public class Constants {

    public static class SwerveK {
        public static final Distance wheelDiameter = Inches.of(2.7); 
        public static final Distance driveBaseRadius = Inches.of(Math.hypot(12.75, 12.75));

        public static final double steerGearRatio = 41.25; 
        public static final double driveGearRatio = 4.4;

        public static final LinearVelocity maxRobotSpeed = MetersPerSecond.of(5.426);

        // Drive Feedforward
        public static final double kS = 0.10431;
        public static final double kV = 2.0967;
        public static final double kA = 0.055428;

        // PathPlanner
        public static final PIDConstants translationConstants = new PIDConstants(5, 0, 0);
        public static final PIDConstants rotationConstants = new PIDConstants(5, 0, 0);
        public static RobotConfig robotConfig; static {
            try {
                robotConfig = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                e.printStackTrace();
            }
        }

        public static final PIDConstants angularPID = new PIDConstants(5, 0, 0.4); // kP = degrees/second per degree
        public static final Angle angularDeadband = Degrees.of(2);
        public static final AngularVelocity angularVelocityDeadband = DegreesPerSecond.of(0.3);

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
        public static final DynamicSlewRateLimiter rotationalLimiter = new DynamicSlewRateLimiter(1, 2);
    }

    public static class ElevatorK {
        //! Motor ID's, Find these values
        public static final int elevatorID = 0;
        public static final int followID = 0;

        public static final double gearRatio = 16;

        public static final Distance sprocketDiameter = Inches.of(0); //! Find

        public static final int stageCount = 3;

        public static final SoftwareLimitSwitchConfigs softwareLimitConfig = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Encoder.linearToAngular(ElevatorK.maxHeight.div(ElevatorK.stageCount), sprocketDiameter))
        .withReverseSoftLimitThreshold(Encoder.linearToAngular(ElevatorK.minHeight.div(ElevatorK.stageCount), sprocketDiameter));


        // Motion Magic Values
        public static final LinearVelocity velocity = MetersPerSecond.of(0);
        public static final LinearAcceleration acceleration = MetersPerSecondPerSecond.of(0);

        //! Elevator-relative heights, Find all these values
        public static final Distance stowHeight = Inches.of(0);
        public static final Distance intakeHeight = Inches.of(0);
        public static final Distance allowableError = Inches.of(0);
        public static final Distance minHeight = Inches.of(0); //! Should be 0
        public static final Distance maxHeight = Inches.of(0);

        //! Height Offsets, Find both
        public static final Distance reefOffset = Inches.of(0); 
        public static final Distance algaeOffset = Inches.of(0);

        //! PID Constants, Tune all
        public static final double kP = 0; 
        public static final double kD = 0; 
        public static final double kG = 0; 

        // Configs
        public static final FeedbackConfigs gearRatioConfig = new FeedbackConfigs().withSensorToMechanismRatio(gearRatio);
        public static final Slot0Configs pidConfig = new Slot0Configs().withKP(kP).withKD(kD).withKG(kG).withGravityType(GravityTypeValue.Elevator_Static);

        //? Set Height Positions, Possible Tune of these
        public enum Positions {
            L1(Field.levelOneHeight.plus(reefOffset)),
            L2(Field.levelTwoHeight.plus(reefOffset)),
            L3(Field.levelThreeHeight.plus(reefOffset)),
            L4(Field.levelFourHeight.plus(reefOffset)),
            ALGAE_LOW(Field.algaeLowHeight.plus(algaeOffset)),
            ALGAE_HIGH(Field.algaeHighHeight.plus(algaeOffset)),
            STOW(ElevatorK.stowHeight),
            INTAKE(ElevatorK.intakeHeight);
    
            public final Distance level;
    
            private Positions(Distance position) {
                this.level = position;
            }
        }
    }  

    public static class AlgaeK { // TODO: Tune everything here
        public static final int sparkMaxID = 0;
        public static final int limitSwitchID = 0;
        public static final double gearRatio = 50;
        public static final Voltage zeroingVoltage = Volts.of(-1.5);

        public static final Angle maxPosition = Degrees.of(120);
        public static final Angle algaePosition = Degrees.of(0);
        public static final Angle stowPosition = Degrees.of(0);
        public static final Angle zeroPosition = Degrees.of(0);
        public static final Angle allowableError = Degrees.of(1);

        public static final double kP = 0;
        public static final double kD = 0;
        public static final AngularVelocity maxVelocity = DegreesPerSecond.of(0);
        public static final AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(0);
    }
    
}
