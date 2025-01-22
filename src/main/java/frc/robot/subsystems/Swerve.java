package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveK;
import frc.robot.subsystems.Vision.VisionResults;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

@SuppressWarnings("unused")
@Logged
public class Swerve extends SubsystemBase { // physicalproperties/conversionFactors/angle/factor = 360.0 deg/4096.0 units per rotation

    private final SwerveDrive swerveDrive;
    private final Supplier<VisionResults> visionSource;
    private final TalonFX frontLeft;
    private final TalonFX frontRight;
    private final TalonFX backLeft;
    private final TalonFX backRight;
    private final PIDController rotationPIDController = new PIDController(SwerveK.angularPID.kP, SwerveK.angularPID.kI, SwerveK.angularPID.kD);

    public Swerve(Supplier<VisionResults> visionSource) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        SwerveParser parser = null;
        try {
            parser = new SwerveParser(SwerveK.swerveDirectory);
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException("Swerve directory not found.");
        }
        swerveDrive = parser.createSwerveDrive(SwerveK.maxRobotSpeed.in(MetersPerSecond));
        this.visionSource = visionSource;
        frontLeft = (TalonFX) swerveDrive.getModules()[0].getDriveMotor().getMotor();
        frontRight = (TalonFX) swerveDrive.getModules()[1].getDriveMotor().getMotor();
        backLeft = (TalonFX) swerveDrive.getModules()[2].getDriveMotor().getMotor();
        backRight = (TalonFX) swerveDrive.getModules()[3].getDriveMotor().getMotor();
        rotationPIDController.setTolerance(SwerveK.angularDeadband.in(Degrees), SwerveK.angularVelocityDeadband.in(DegreesPerSecond));
        rotationPIDController.enableContinuousInput(-Rotation2d.k180deg.getDegrees(), Rotation2d.k180deg.getDegrees());
        setupPathPlanner();
        SmartDashboard.putData(rotationPIDController);
    }

    @Override
    public void periodic() {
        for (var result : visionSource.get().results()) {
            EstimatedRobotPose pose = result.getFirst();
            swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, result.getSecond());
        }
    }

    /**
     * Configures PathPlanner
     */
    private void setupPathPlanner() {
        AutoBuilder.configure(
            this::getPose, 
            this::resetOdometry, 
            this::getRobotVelocity, 
            (speeds, feedforward) -> setChassisSpeeds(speeds), 
            new PPHolonomicDriveController(SwerveK.translationConstants, SwerveK.rotationConstants),
            SwerveK.robotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                } 
                return false;
            }
            , this);
    }

    /**
     * Commands the robot to drive according to the given velocities
     * @param TranslationX Translation in the X direction (Forwards, Backwards) between -1 and 1
     * @param TranslationY Translation in the Y direction (Left, Right) between -1 and 1
     * @param angularVelocity Angular Velocity to set between -1 and 1
     * @param fieldRelative Whether or not swerve is controlled using field relative speeds
     * @return A command to drive the robot according to given velocities
     */
    public Command driveCommand(DoubleSupplier TranslationX, DoubleSupplier TranslationY, DoubleSupplier angularVelocity, boolean fieldRelative) {
        return runOnce(() -> drive(
                new Translation2d(TranslationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(), TranslationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 
                RadiansPerSecond.of(angularVelocity.getAsDouble() * (swerveDrive.getMaximumChassisVelocity() / SwerveK.driveBaseRadius.in(Meters))),
                fieldRelative, true)).withName("Swerve Drive");
    }

    /**
     * Turns the robot to the desired angle
     * @param target Desired angle
     * @return A command that turns the robot until it's at the desired angle
     */
    public Command turnCommand(Angle target) {
        return runOnce(() -> {
            rotationPIDController.reset();
            rotationPIDController.setSetpoint(target.in(Degrees));
            SmartDashboard.putNumber("Reference", target.in(Degrees));
        })
        .andThen(runEnd(() -> {
            SmartDashboard.putNumber("Current", getHeading().in(Degrees));
            AngularVelocity velocity = DegreesPerSecond.of(rotationPIDController.calculate(getHeading().in(Degrees)));
            drive(Translation2d.kZero, velocity, true, false);
        }, this::stop))
        .until(rotationPIDController::atSetpoint)
        .withTimeout(Seconds.of(2))
        .withName("Swerve Turn");
    }

    /**
     * Commands the drivebase to move according to the given linear and rotational velocities
     * @param translation Linear velocity of the robot in meters per second
     * @param rotation Rotation rate of the robot in Radians per second
     * @param fieldRelative Whether the robot is field relative (true) or robot relative (false)
     * @param isOpenLoop Whether it uses a closed loop velocity control or an open loop
     */
    private void drive(Translation2d translation, AngularVelocity rotation, boolean fieldRelative, boolean isOpenLoop) {
        swerveDrive.drive(translation, rotation.in(RadiansPerSecond), fieldRelative, isOpenLoop);
    }

    public void stop() {
        drive(Translation2d.kZero, RadiansPerSecond.zero(), true, false);
    }

    /**
     * Resets the odometry to the given pose
     * @param pose Pose to reset the odemetry to
     */
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /**
     * Returns the robot's pose
     * @return Current pose of the robot as a Pose2d
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Returns the robot's velocity (x, y, and omega)
     * @return Current velocity of the robot
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Set the speed of the robot with closed loop velocity control
     * @param chassisSpeeds to set speed with
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Returns the robot's heading as an Angle
     * @return The heading of the robot
     */
    public Angle getHeading() {
        return getPose().getRotation().getMeasure();
    }

    public String getCurrentCommandName() {
        var cmd = getCurrentCommand();
        if (cmd == null) return "None";
        return cmd.getName();
    }

    /**
     * Resets the gyro and odometry to the current position but the current direction is now seen as 0.
     * Useful for resetting the forward direction for field relative driving
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

}