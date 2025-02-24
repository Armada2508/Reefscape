package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.util.DynamicSlewRateLimiter;
import frc.robot.lib.util.Encoder;

public class Constants {

    public static final Angle halfTurn = Degrees.of(180);
    public static final LinearAccelerationUnit InchesPerSecondPerSecond = InchesPerSecond.per(Second);

    public static class SwerveK {
        public static final Distance driveBaseRadius = Inches.of(Math.hypot(12.75, 12.75));
        public static final Distance driveBaseLength = Inches.of(35); // Base is a square so this is the same as the width

        // Currently Unused
        public static final double steerGearRatio = 41.25; 
        public static final double driveGearRatio = 4.4;

        public static final LinearVelocity maxPossibleRobotSpeed = MetersPerSecond.of(5.426);
        public static final CurrentLimitsConfigs currentLimitsConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(40)).withSupplyCurrentLimitEnable(true);

        // Path Constraints
        public static final LinearVelocity maxRobotVelocity = FeetPerSecond.of(6); // Should be just under 3/4 of our max possible speed, arbitrary value
        public static final LinearAcceleration maxRobotAcceleration = FeetPerSecondPerSecond.of(3.5); 
        public static final AngularVelocity maxRobotAngularVelocity = DegreesPerSecond.of(180); 
        public static final AngularAcceleration maxRobotAngularAcceleration = DegreesPerSecondPerSecond.of(270); 

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

        // Turn PID
        public static final PIDConstants angularPID = new PIDConstants(5, 0, 0.4); // kP = degrees/second per degree
        public static final Angle angularDeadband = Degrees.of(2);
        public static final AngularVelocity angularVelocityDeadband = DegreesPerSecond.of(0.3);

        public static final File swerveDirectory = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/swerve");
    }

    public static class ControllerK {
        public static final int xboxPort = 0;
        public static final double leftJoystickDeadband = 0.06;
        public static final double rightJoystickDeadband = 0.06;

        public static final double overrideThreshold = leftJoystickDeadband * 1.5;
    }

    public static class DriveK {
        public static final DynamicSlewRateLimiter translationalYLimiter = new DynamicSlewRateLimiter(1.25, 2); // Larger number = faster rate of change
        public static final DynamicSlewRateLimiter translationalXLimiter = new DynamicSlewRateLimiter(1.25, 2);
        public static final DynamicSlewRateLimiter rotationalLimiter = new DynamicSlewRateLimiter(1, 2);

        public static final double driveSpeedModifier = 0.7                                                                                         ;
        public static final double rotationSpeedModifier = 1;
    }

    public static class ElevatorK {
        public static final int talonID = 8;
        public static final int talonFollowID = 9;
        public static final double gearRatio = 12.75;
        public static final Distance sprocketDiameter = Inches.of(1.751); // Pitch Diameter
        public static final int stageCount = 3;
        public static final Voltage zeroingVoltage = Volts.of(-0.5);

        // Feedfoward and feedback gains
        public static final double kG = 0.285; // Volts
        public static final double kS = 0.085; // Volts
        public static final double kV = 1.4389; // Volts/rps of target, 1.4389
        public static final double kP = 6; // Volts/rotation of error
        public static final double kD = 0; // Volts/rps of error

        public static final LinearVelocity maxVelocity = InchesPerSecond.of(35);
        public static final LinearAcceleration maxAcceleration = InchesPerSecondPerSecond.of(46);

        // All heights are relative to the top of the bottom bar of the carriage station to the ground floor
        public static final Distance minHeight = Inches.of(5.925);
        public static final Distance maxHeight = Inches.of(67.5);
        public static final Distance armThresholdHeight = Inches.of(30); // height that is safe to move algae arm w/o hitting robot
        public static final Distance allowableError = Inches.of(0.125);

        // Configs
        public static final FeedbackConfigs gearRatioConfig = new FeedbackConfigs().withSensorToMechanismRatio(gearRatio);
        public static final CurrentLimitsConfigs currentLimitsConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(40)).withSupplyCurrentLimitEnable(true);
        public static final Slot0Configs pidConfig = new Slot0Configs()
            .withKG(kG)
            .withKS(kS)
            .withKV(kV)
            .withKP(kP)
            .withKD(kD)
            .withGravityType(GravityTypeValue.Elevator_Static);
        public static final SoftwareLimitSwitchConfigs softwareLimitConfig = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Encoder.linearToAngular(ElevatorK.maxHeight.div(ElevatorK.stageCount), sprocketDiameter))
            .withReverseSoftLimitThreshold(Encoder.linearToAngular(ElevatorK.minHeight.div(ElevatorK.stageCount), sprocketDiameter));
        public static final HardwareLimitSwitchConfigs hardwareLimitConfig = new HardwareLimitSwitchConfigs()
            .withReverseLimitAutosetPositionEnable(true)
            .withReverseLimitAutosetPositionValue(Encoder.linearToAngular(ElevatorK.minHeight.div(ElevatorK.stageCount), sprocketDiameter));

        public enum Positions {
            L1(Inches.of(22)),
            L2(Inches.of(31.25)),
            L3(Inches.of(46.375)),
            L4(Inches.of(46.375)),
            ALGAE_LOW(Inches.of(29)), // Not Found
            ALGAE_HIGH(Inches.of(29)), // Not Found
            INTAKE(Inches.of(33)),
            STOW(ElevatorK.minHeight);
    
            public final Distance level;
    
            private Positions(Distance position) {
                this.level = position;
            }
        }
    }  

    public static class IntakeK { // TODO: Confirm voltages and detection range
        public static final int sparkMaxLeftID = 2; 
        public static final int sparkMaxRightID = 3; 
        public static final int timeOfFlightId = 0; 

        public static final Distance coralDetectionRange = Inches.of(4);

        public static final int currentLimit = 20; // Amps

        public static final Voltage coralIntakeVolts = Volts.of(12);
        public static final Time intakeAfterTrip = Seconds.of(0.25);

        public static final Voltage levelOneVolts = Volts.of(-7.5);
        public static final Time levelOneWait = Seconds.of(0.04);
        public static final Voltage levelOneReverseVolts = Volts.of(5.5);

        public static final Voltage levelTwoThreeVolts = Volts.of(-5.5);
        public static final Voltage levelFourVolts = Volts.of(-5);
    }

    public static class AlgaeK { // TODO: Tune everything
        public static final int sparkMaxID = 1;
        public static final double gearRatio = 47.045881;
        public static final Voltage zeroingVoltage = Volts.of(0.5);
        public static final int currentLimit = 20;

        public static final Angle maxPosition = Degrees.of(160);
        public static final Angle algaePosition = Degrees.of(90);
        public static final Angle loweredAlgaePosition = Degrees.of(90);
        public static final Angle stowPosition = Degrees.of(30);
        public static final Angle zeroPosition = Degrees.of(5);
        public static final Angle allowableError = Degrees.of(2);

        public static final double kP = 0.1;
        public static final double kD = 0;
        public static final AngularVelocity maxVelocity = DegreesPerSecond.of(15);
        public static final AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(15);
    }

    public static class VisionK { // TODO: Find transform and standard deviations
        public static final String frontCameraName = "ArducamFront";
        public static final String backCameraName = "ArducamBack";
        public static final Transform3d robotToFrontCamera = new Transform3d(Inches.of(14.5), Inches.of(7), Inches.of(7.5), new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(0)));
        public static final Transform3d robotToBackCamera = new Transform3d(Inches.of(-14.5), Inches.of(-7), Inches.of(7.5), new Rotation3d(Degrees.of(0), Degrees.of(15), Degrees.of(180)));
        // Acceptable height of pose estimation to consider it a valid pose
        public static final Distance maxPoseZ = Inches.of(12);
        public static final Distance minPoseZ = Inches.of(-6);
        // Used in scaling the standard deviations by average distance to april tags
        public static final Distance baseLineAverageTagDistance = Inches.of(60);
        // Vision Standard Deviations (Meters, Meters, Radians)
        public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(Units.feetToMeters(9), Units.feetToMeters(9), Units.degreesToRadians(360));
        public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(Units.feetToMeters(1.5), Units.feetToMeters(1.5), Units.degreesToRadians(180));
        public static final Matrix<N3, N1> untrustedStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    public static class ClimbK { // TODO: Tune everything
        // Motors
        public static final int talonID = 10;
        public static final int talonFollowID = 11;

        // Positions/Angles/Voltage
        public static final Voltage climbVoltage = Volts.of(2);
        
        public static final Angle maxAngle = Degrees.of(90);
        public static final Angle minAngle = Degrees.of(0);
        public static final Angle allowableError = Degrees.of(1);

        // Motion Magic
        public static final AngularVelocity maxVelocity = DegreesPerSecond.of(0);
        public static final AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(0); 
        
        public static final double kP = 0;
        public static final double kD = 0;

        public static final Slot0Configs pidconfig = new Slot0Configs().withKD(kD).withKP(kP);
        public static final double gearRatio = 100;
        public static final FeedbackConfigs gearRatioConfig = new FeedbackConfigs().withSensorToMechanismRatio(gearRatio);
        
        public static final SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs() // Forward limit
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(maxAngle);
        
        public static final HardwareLimitSwitchConfigs hardLimitSwitchConfigs = new HardwareLimitSwitchConfigs() // Reverse limit
            .withReverseLimitEnable(true)
            .withReverseLimitAutosetPositionEnable(true)
            .withReverseLimitAutosetPositionValue(minAngle);
    }
    
}
