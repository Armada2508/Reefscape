package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveK;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {

    private final SwerveDrive swerveDrive;

    public Swerve() {
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(SwerveK.steerGearRatio, 4096);
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(SwerveK.driveGearRatio, SwerveK.wheelDiameter.in(Inches));

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        SwerveParser parser = null;
        try {
            parser = new SwerveParser(SwerveK.swerveDirectory);
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException("Swerve directory not found.");
        }
        swerveDrive = parser.createSwerveDrive(SwerveK.maxRobotSpeed.in(MetersPerSecond), angleConversionFactor, driveConversionFactor);
        setupPathPlanner();
    }

    @Override
    public void periodic() {
        // Do nothing
    }

    /**
     * Configures PathPlanner
     */
    public void setupPathPlanner() {
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
                angularVelocity.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(), 
                fieldRelative, true));
    }

    /**
     * Turns the robot to the desired angle
     * @param targetAngle Desired angle
     * @param currentAngle Current angle
     * @param fieldRelative Whether or not swerve is controlled using field relative speeds
     * @return An instant command that turns the robot
     */
    public Command turnCommand(Angle targetAngle, Angle currentAngle, boolean fieldRelative) {
        return runOnce(() -> turn(targetAngle, currentAngle, fieldRelative));
    }

    /**
     * Turns the robot to the desired angle
     * @param targetAngle Desired angle
     * @param currentAngle Current angle
     * @param fieldRelative Whether or not swerve is controlled using field relative speeds
     */
    public void turn(Angle targetAngle, Angle currentAngle, boolean fieldRelative) {
        drive(getPose().getTranslation(), getTurningAngle(targetAngle, currentAngle).in(Degrees), fieldRelative, false);
    }

    /**
     * Gets the closest angle to turn to depending on the current heading of the robot
     * @param desiredAngle Angle to turn to
     * @param currentHeading Current heading
     * @return Angle to turn to
     */
    public Angle getTurningAngle(Angle desiredAngle, Angle currentHeading) {
        double angle = (desiredAngle.minus(currentHeading).plus(Degrees.of(540))).in(Degrees);
        angle = (angle % 360) - 180;
        return Degrees.of(angle);
    }

    /**
     * Commands the drivebase to move according to the given linear and rotational velocities
     * @param translation Linear velocity of the robot in meters per second
     * @param rotation Rotation rate of the robot in Radians per second
     * @param fieldRelative Whether the robot is field relative (true) or robot relative (false)
     * @param isOpenLoop Whether it uses a closed loop velocity control or an open loop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    /**
     * Resets the odometry to the given pose
     * @param initialHolonomicPose Pose to reset the odemetry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
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
     * Returns the robot's heading as a Rotation2d
     * @return Rotational component of the robots pose
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Resets the gyro and odometry to the current position but the current direction is now seen as 0.
     * Useful for resetting the forward direction for field relative driving
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

}