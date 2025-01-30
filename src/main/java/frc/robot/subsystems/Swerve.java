package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveK;
import frc.robot.Field;
import frc.robot.Robot;
import frc.robot.commands.DriveWheelCharacterization;
import frc.robot.subsystems.Vision.VisionResults;
import swervelib.SwerveDrive;
import swervelib.motors.TalonFXSwerve;
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
    private final SysIdRoutine sysIdRoutine; 
    private final PIDController rotationPIDController = new PIDController(SwerveK.angularPID.kP, SwerveK.angularPID.kI, SwerveK.angularPID.kD);
    private final PPHolonomicDriveController pathPlannerController = new PPHolonomicDriveController(SwerveK.translationConstants, SwerveK.rotationConstants);
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Robot").getSubTable("swerve");
    private boolean initializedOdometryFromVision = false;

    public Swerve(Supplier<VisionResults> visionSource) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        SwerveParser parser = null;
        try {
            parser = new SwerveParser(SwerveK.swerveDirectory);
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException("Swerve directory not found.");
        }
        swerveDrive = parser.createSwerveDrive(SwerveK.maxPossibleRobotSpeed.in(MetersPerSecond));
        this.visionSource = visionSource;
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(SwerveK.kS, SwerveK.kV, SwerveK.kA));
        frontLeft = (TalonFX) swerveDrive.getModules()[0].getDriveMotor().getMotor();
        frontRight = (TalonFX) swerveDrive.getModules()[1].getDriveMotor().getMotor();
        backLeft = (TalonFX) swerveDrive.getModules()[2].getDriveMotor().getMotor();
        backRight = (TalonFX) swerveDrive.getModules()[3].getDriveMotor().getMotor();
        rotationPIDController.setTolerance(SwerveK.angularDeadband.in(Degrees), SwerveK.angularVelocityDeadband.in(DegreesPerSecond));
        rotationPIDController.enableContinuousInput(-Rotation2d.k180deg.getDegrees(), Rotation2d.k180deg.getDegrees());
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)
                            // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    for (var module : swerveDrive.getModules()) {
                        var motor = (TalonFXSwerve) module.getDriveMotor();
                        ((TalonFX) motor.getMotor()).setControl(new VoltageOut(volts));
                    }
                },
                null,
                this
            )
        );
        setupPathPlanner();
    }

    @Override
    public void periodic() {
        for (var result : visionSource.get().results()) {
            EstimatedRobotPose pose = result.getFirst();
            if (!initializedOdometryFromVision) {
                resetOdometry(pose.estimatedPose.toPose2d());
                initializedOdometryFromVision = true;
                continue;
            }
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
            pathPlannerController,
            SwerveK.robotConfig,
            Robot::onRedAlliance, 
            this);
    }

    /**
     * Commands the robot to drive according to the given velocities, this switches the direction depending on what alliance you're on
     * @param TranslationX Translation in the X direction (Forwards, Backwards) between -1 and 1
     * @param TranslationY Translation in the Y direction (Left, Right) between -1 and 1
     * @param angularVelocity Angular Velocity to set between -1 and 1
     * @param fieldRelative Whether or not swerve is controlled using field relative speeds
     * @return A command to drive the robot according to given velocities
     */
    public Command driveCommand(DoubleSupplier TranslationX, DoubleSupplier TranslationY, DoubleSupplier angularVelocity, boolean fieldRelative, boolean openLoop) {
        return runOnce(() -> {
            Translation2d translation = new Translation2d(TranslationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(), TranslationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity());
            AngularVelocity rotation = RadiansPerSecond.of(angularVelocity.getAsDouble() * (swerveDrive.getMaximumChassisVelocity() / SwerveK.driveBaseRadius.in(Meters)));
            drive(Robot.onRedAlliance() ? translation.unaryMinus() : translation, rotation, fieldRelative, openLoop);
        }).withName("Swerve Drive");
    }

    /**
     * Turns the robot to the desired field relative angle
     * @param target Desired angle
     * @return A command that turns the robot until it's at the desired angle
     */
    public Command turnCommand(Angle target) {
        return turnCommand(() -> target);
    }

    /**
     * Turns the robot to the desired field relative angle
     * @param targetSupplier Desired angle
     * @return A command that turns the robot until it's at the desired angle
     */
    public Command turnCommand(Supplier<Angle> targetSupplier) {
        return runOnce(() -> {
            Angle target = targetSupplier.get();
            rotationPIDController.reset();
            rotationPIDController.setSetpoint(target.in(Degrees));
            table.getEntry("Reference").setDouble(target.in(Degrees));
        })
        .andThen(runEnd(() -> {
            table.getEntry("Current").setDouble(getHeading().in(Degrees));
            AngularVelocity velocity = DegreesPerSecond.of(rotationPIDController.calculate(getHeading().in(Degrees)));
            drive(Translation2d.kZero, velocity, true, false);
        }, this::stop))
        .until(rotationPIDController::atSetpoint)
        .withTimeout(Seconds.of(2))
        .withName("Swerve Turn");
    }

    /**
     * Constructs a command to take the robot from current position to an end position
     * @param x x component of the final position
     * @param y y component of the final position
     * @param rotation Rotations of the final position
     * @return Command to drive along the constructed path
     */
    public Command driveToPoseCommand(Distance x, Distance y, Rotation2d rotation) {
        return driveToPoseCommand(new Pose2d(x, y, rotation));
    }

    /**
     * Constructs a command to take the robot from current position to an end position. This does not flip the path depending on alliance
     * @param endPose Final pose to end the robot at
     * @return Command to drive along the constructed path
     */
    public Command driveToPoseCommand(Pose2d endPose) {
        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(getPose(), endPose), 
            new PathConstraints(SwerveK.maxRobotVelocity, SwerveK.maxRobotAcceleration, SwerveK.maxRobotAngularVelocity, SwerveK.maxRobotAngularAcceleration), 
            null, 
            new GoalEndState(MetersPerSecond.of(0), endPose.getRotation()));
        return new FollowPathCommand(
            path, 
            this::getPose, 
            this::getRobotVelocity, 
            (speeds, feedforward) -> setChassisSpeeds(speeds),
            pathPlannerController, 
            SwerveK.robotConfig,
            () -> false, 
            this 
        );
    }

    /**
     * Creates a command to drive the robot to the nearest reef from its current position
     * @return driveToPoseCommand to drive to the nearest reef on your side
     */
    public Command alignToReef() {
        Pose2d reefPose = getPose().nearest(Robot.onRedAlliance() ? Field.redReefList : Field.blueReefList);
        Translation2d reefOffset = new Translation2d(Field.reefOffsetDistance, Inches.of(0)).rotateBy(reefPose.getRotation());
        return driveToPoseCommand(
            reefPose.getMeasureX().plus(reefOffset.getMeasureX()),
            reefPose.getMeasureY().plus(reefOffset.getMeasureY()),
            reefPose.getRotation().plus(Rotation2d.fromDegrees(180))
        );
    }

    /**
     * Creates a command to drive the robot to the nearest coral station to it
     * @return driveToPoseCommand to drive to the nearest station on your side
     */
    public Command alignToCoralStation() {
        Pose2d stationPose = getPose().nearest(Robot.onRedAlliance() ? Field.redCoralStationList : Field.blueCoralStationList);
        Translation2d stationOffset = new Translation2d(Field.stationOffsetDistance, Inches.of(0)).rotateBy(stationPose.getRotation());
        return driveToPoseCommand(            
            stationPose.getMeasureX().plus(stationOffset.getMeasureX()),
            stationPose.getMeasureY().plus(stationOffset.getMeasureY()),
            stationPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    /**
     * Creates a command to drive to the top cage of your side
     * @return driveToPoseCommand to drive to the top cage
     */
    public Command alignToTopCage() {
        if (Robot.onRedAlliance()) { 
            return driveToPoseCommand(
                Field.redCageTop.getMeasureX().plus(Field.cageOffset), Field.redCageTop.getMeasureY(), Field.redCageTop.getRotation()
            ); 
        }
        return driveToPoseCommand(
            Field.blueCageTop.getMeasureX().minus(Field.cageOffset), Field.blueCageTop.getMeasureY(), Field.blueCageTop.getRotation()
        );
    }

    /**
     * Creates a command to drive to the mid cage of your side
     * @return driveToPoseCommand to drive to the mid cage
     */
    public Command alignToMidCage() {
        if (Robot.onRedAlliance()) {
            return driveToPoseCommand(
                Field.redCageMid.getMeasureX().plus(Field.cageOffset), Field.redCageMid.getMeasureY(), Field.redCageMid.getRotation()
            ); 
        }
        return driveToPoseCommand(
            Field.blueCageMid.getMeasureX().minus(Field.cageOffset), Field.blueCageMid.getMeasureY(), Field.blueCageMid.getRotation()
        );
    }

    /**
     * Creates a command to drive to the low cage of your side
     * @return driveToPoseCommand to drive to the low cage
     */
    public Command alignToLowCage() {
        if (Robot.onRedAlliance()) {
            return driveToPoseCommand(
                Field.redCageLow.getMeasureX().plus(Field.cageOffset), Field.redCageLow.getMeasureY(), Field.redCageLow.getRotation()
            );
        }
        return driveToPoseCommand(
            Field.blueCageLow.getMeasureX().minus(Field.cageOffset), Field.blueCageLow.getMeasureY(), Field.blueCageLow.getRotation()
        );
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
     * @param chassisSpeeds to set speed with (robot relative)
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Returns the robot's heading as an Angle wrapped between -180 and 180
     * @return The heading of the robot
     */
    public Angle getHeading() {
        return getPose().getRotation().getMeasure();
    }

    /**
     * Returns the gyro's yaw, not wrapped
     * @return The yaw of the gyro
     */
    public Angle getYaw() {
        return ((Pigeon2) swerveDrive.getGyro().getIMU()).getYaw().getValue();
    }

    /**
     * Returns the positions of each drive wheel in radians, front left -> front right -> back left -> back right.
     * @return
     */
    public double[] getWheelPositions() {
        var modulePositions = swerveDrive.getModulePositions();
        double[] wheelPositions = new double[modulePositions.length];
        for (int i = 0; i < modulePositions.length; i++) {
            double diameter = Units.inchesToMeters(swerveDrive.swerveDriveConfiguration.physicalCharacteristics.conversionFactor.drive.diameter);
            wheelPositions[i] = Units.rotationsToRadians(modulePositions[i].distanceMeters / (diameter * Math.PI));
        }
        return wheelPositions;
    }

    @Logged(name = "Current Command")
    public String getCurrentCommandName() {
        var cmd = getCurrentCommand();
        if (cmd == null) return "None";
        return cmd.getName();
    }

    public boolean initializedOdometryFromVision() {
        return initializedOdometryFromVision;
    }

    /**
     * Resets the gyro and odometry to the current position but the current direction is now seen as 0.
     * Useful for resetting the forward direction for field relative driving
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }
     
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command characterizeDriveWheelDiameter() {
        return new DriveWheelCharacterization(this);
    }

}