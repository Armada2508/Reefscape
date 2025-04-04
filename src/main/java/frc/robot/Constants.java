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
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.util.Encoder;

public class Constants {

    public static final Angle halfTurn = Degrees.of(180);
    public static final LinearAccelerationUnit InchesPerSecondPerSecond = InchesPerSecond.per(Second);

    public static class SwerveK {
        public static final Distance driveBaseRadius = Inches.of(Math.hypot(12.75, 12.75));
        public static final Distance driveBaseLength = Inches.of(35); // Base is a square so this is the same as the width
        public static final Time coastDisableTime = Seconds.of(10);

        // Currently Unused
        public static final double steerGearRatio = 41.25; 
        public static final double driveGearRatio = 4.4;

        public static final LinearVelocity maxPossibleRobotSpeed = MetersPerSecond.of(5.426);
        public static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(10.477);
        public static final CurrentLimitsConfigs currentLimitsConfig = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Amps.of(70)).withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(Amps.of(75)).withStatorCurrentLimitEnable(true);

        // Path Constraints
        public static final LinearVelocity maxRobotVelocity = FeetPerSecond.of(6); // Should be just under 3/4 of our max possible speed, arbitrary value
        public static final LinearAcceleration maxRobotAcceleration = FeetPerSecondPerSecond.of(3.5); 
        public static final AngularVelocity maxRobotAngularVelocity = DegreesPerSecond.of(180); 
        public static final AngularAcceleration maxRobotAngularAcceleration = DegreesPerSecondPerSecond.of(270); 

        // Drive Feedforward
        public static final double kS = 0.23118;
        public static final double kV = 2.1701;
        public static final double kA = 0.15136;

        // PathPlanner
        public static final PIDConstants ppTranslationConstants = new PIDConstants(5.25, 0, 0); // m/s / m
        public static final PIDConstants ppRotationConstants = new PIDConstants(5, 0, 0); // rad/s / rad
        public static RobotConfig robotConfig; static {
            try {
                robotConfig = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                e.printStackTrace();
            }
        }

        // PID Alignment
        public static final PIDConstants translationConstants = new PIDConstants(5.25, 0, 0); // m/s / m of error
        public static final PIDConstants rotationConstants = new PIDConstants(5, 0, 0); // rad/s / rad of error
        public static final TrapezoidProfile.Constraints defaultTranslationConstraints = 
            new TrapezoidProfile.Constraints(Units.feetToMeters(5), Units.feetToMeters(10)); // m/s & m/s^2
        public static final TrapezoidProfile.Constraints rotationConstraints = 
            new TrapezoidProfile.Constraints(Units.degreesToRadians(360), Units.degreesToRadians(360)); // rad/s & rad/s^2
        public static final TrapezoidProfile.Constraints climbTranslationConstraints = 
            new TrapezoidProfile.Constraints(Units.feetToMeters(2), Units.feetToMeters(4)); // m/s & m/s^2
        public static final Distance maximumTranslationError = Inches.of(0.25);
        public static final Angle maximumRotationError = Degrees.of(0.5);

        public static final File swerveDirectory = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/swerve");
    }

    public static class ControllerK {
        public static final int xboxPort = 0;
        public static final double leftJoystickDeadband = 0.07;
        public static final double rightJoystickDeadband = 0.07;

        // Teleop Alignment Overriding
        public static final double overrideThreshold = 0.14;
        public static final Time overrideTime = Seconds.of(0.25);
    }

    public static class DriveK {
        // Larger number = faster rate of change, limit is in units of (units)/second. In this case the joystick [-1, 1].
        public static final Pair<Double, Double> translationAccelLimits = Pair.of(1.25, 2.0); 
        public static final Pair<Double, Double> rotationAccelLimits = Pair.of(1.0, 2.0);
        public static final double elevatorAccelScaling = 0.5; // Acceleration is halved when elevator is at max height
        public static final RangeTransformer elevatorAccelTransformer = new RangeTransformer(ElevatorK.minHeight.in(Inches), ElevatorK.maxHeight.in(Inches), 1, elevatorAccelScaling, true);

        public static final double driveSpeedModifier = 1;
        public static final double rotationSpeedModifier = 1;
        public static final double exponentialControl = 1.75;
    }

    public static class ElevatorK {
        public static final int talonID = 8;
        public static final int talonFollowID = 9;
        public static final int tofID = 1;
        public static final double gearRatio = 12.75;
        public static final Distance sprocketDiameter = Inches.of(1.751); // Pitch Diameter
        public static final int stageCount = 3;
        public static final Current currentSpike = Amps.of(47.5);
        public static final Time currentTripTime = Seconds.of(0.125);

        // Feedfoward and feedback gains
        public static final double kG = 0.28; // Volts
        public static final double kS = 0.07; // Volts
        public static final double kV = 1.4389; // Volts/rps of target, 1.4389
        public static final double kP = 30; // Volts/rotation of error
        public static final double kD = 0; // Volts/rps of error

        public static final LinearVelocity maxVelocity = InchesPerSecond.of(105);
        public static final LinearAcceleration maxAcceleration = InchesPerSecondPerSecond.of(300);

        // All heights are relative to the top of the bottom bar of the carriage station to the ground floor
        public static final Distance minHeight = Inches.of(6.05);
        public static final Distance maxHeight = Inches.of(74.25);
        public static final Distance intakeBumpHeight = Inches.of(4);
        public static final Distance armThresholdHeight = Inches.of(20); // height that is safe to move algae arm w/o hitting robot
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

        public enum Positions {
            L1(Inches.of(22), Inches.of(22)),
            L2(Inches.of(30.125), Inches.of(30.125)),
            L3(Inches.of(45.70), Inches.of(45.70)),
            L4(Inches.of(70.125), Inches.of(70.125)),
            ALGAE_LOW(Inches.of(25), Inches.of(25)), // Not Found
            ALGAE_HIGH(Inches.of(40), Inches.of(40)), // Not Found
            INTAKE(Inches.of(32.25), Inches.of(27)), // Min: 31.25" | Max: 33.25", Mid: 32.25"
            STOW(Inches.of(7.5), Inches.of(7.5));
    
            public final Distance close, far;
    
            private Positions(Distance close, Distance far) {
                this.close = close;
                this.far = far;
            }
        }

        // Linear Interpolation
        public static final Distance timeOfFlightOffset = Inches.of(-13.7);
        public static final Distance maxLinearDistance = Inches.of(4.5);
        public static final int sampleTime = 24; // ms
    }  


    public static class IntakeK {
        public static final int sparkMaxLeftID = 2; 
        public static final int sparkMaxRightID = 3; 
        public static final int timeOfFlightId = 0; 

        public static final Distance coralDetectionRange = Inches.of(4);

        public static final int currentLimit = 20; // Amps

        public static final double holdVoltageAtMaxSpeed = 1;

        public static final Voltage coralIntakeVolts = Volts.of(12);
        public static final Time intakeSecureTime = Seconds.of(0.25);

        public static final Voltage levelOneVolts = Volts.of(-7.5);
        public static final Time levelOneWait = Seconds.of(0.04);
        public static final Voltage levelOneReverseVolts = Volts.of(5.5);
        public static final Time levelOneSecondWait = Seconds.of(0.5);

        public static final Voltage levelTwoThreeVolts = Volts.of(-5.5);
        public static final Voltage levelFourVolts = Volts.of(-6);
    }

    public static class AlgaeK {
        public static final int sparkMaxID = 1;
        public static final double gearRatio = 47.045881;
        public static final Voltage zeroingVoltage = Volts.of(-0.5);
        public static final int currentLimit = 30;

        public static final Angle zeroPosition = Degrees.of(5);
        public static final Angle maxPosition = Degrees.of(120);
        public static final Angle algaePosition = Degrees.of(70);
        public static final Angle loweredAlgaePosition = Degrees.of(90);
        public static final Angle stowPosition = Degrees.of(15);
        public static final Angle allowableError = Degrees.of(2);

        public static final double kP = 230;
        public static final double kD = 0;
        public static final AngularVelocity maxVelocity = DegreesPerSecond.of(90);
        public static final AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(90);
    }

    public static class VisionK {
        public static final String frontCameraName = "ArducamFront"; // 7.5, 34.77, 5.22
        public static final String backCameraName = "ArducamBack"; 
        public static final Transform3d robotToFrontCamera = new Transform3d(Inches.of(0.577), Inches.of(-1.023), Inches.of(29.223), new Rotation3d(Degrees.of(11.5), Degrees.of(30.75), Degrees.of(5.8)));
        public static final Transform3d robotToBackCamera = new Transform3d(Inches.of(-3.148), Inches.of(7.729), Inches.of(32.452), new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(-155)));
        // Acceptable height of pose estimation to consider it a valid pose
        public static final Distance maxPoseZ = Inches.of(12);
        public static final Distance minPoseZ = Inches.of(-6);
        // Used in scaling the standard deviations by average distance to april tags
        public static final Distance baseLineAverageTagDistance = Inches.of(84);
        public static final Distance maxAverageTagDistance = Inches.of(160);
        // Vision Standard Deviations (Meters, Meters, Radians)
        public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(Units.feetToMeters(3), Units.feetToMeters(3), Units.degreesToRadians(360));
        public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(Units.feetToMeters(1.5), Units.feetToMeters(1.5), Units.degreesToRadians(180));
        public static final Matrix<N3, N1> untrustedStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    public static class ClimbK { // TODO: Tune everything
        public static final int talonID = 10;
        public static final int talonFollowID = 11;
        public static final int servoRID = 0;
        public static final int servoLID = 1;

        public static final Voltage climbVoltage = Volts.of(-4);
        public static final Voltage prepVoltage = Volts.of(4);
        
        public static final Angle maxAngle = Degrees.of(62);
        public static final Angle minAngle = Degrees.of(-105.5);
        public static final Angle allowableError = Degrees.of(0.25);
        public static final Angle gripAngle = Degrees.of(-31); // Angle of gription
        public static final Angle stowAngle = Degrees.of(-90);

        public static final double servoMin = 0;
        public static final double servoMax = 0.25;
        public static final Time servoAcutateTime = Seconds.of(0.5);

        // Motion Magic
        public static final AngularVelocity climbVelocity = DegreesPerSecond.of(45);
        public static final AngularAcceleration climbAcceleration = DegreesPerSecondPerSecond.of(60); 
        public static final AngularVelocity gripVelocity = DegreesPerSecond.of(-60);
        public static final AngularAcceleration gripAcceleration = DegreesPerSecondPerSecond.of(90); 
        
        public static final double kP = 2500;
        public static final double kD = 0;
        public static final double gearRatio = 100;

        public static final MotorOutputConfigs outputConfigs = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
        public static final Slot0Configs pidconfig = new Slot0Configs().withKP(kP).withKD(kD);
        public static final FeedbackConfigs gearRatioConfig = new FeedbackConfigs().withSensorToMechanismRatio(gearRatio);
        public static final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(220))
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(110))
            .withSupplyCurrentLowerLimit(Amps.of(110))
            .withSupplyCurrentLimitEnable(true);
        
        public static final SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(maxAngle)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(minAngle);
    }

}
