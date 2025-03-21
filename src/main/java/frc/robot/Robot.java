// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeK;
import frc.robot.Constants.ControllerK;
import frc.robot.Constants.DriveK;
import frc.robot.Constants.ElevatorK;
import frc.robot.Constants.ElevatorK.Positions;
import frc.robot.Constants.IntakeK;
import frc.robot.Constants.SwerveK;
import frc.robot.Field.ReefSide;
import frc.robot.commands.Autos;
import frc.robot.commands.Routines;
import frc.robot.lib.logging.LogUtil;
import frc.robot.lib.logging.TalonFXLogger;
import frc.robot.lib.util.DriveUtil;
import frc.robot.lib.util.DynamicSlewRateLimiter;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

@Logged
public class Robot extends TimedRobot {
    
    @Logged(name = "Vision")
    private final Vision vision = new Vision();
    private final CommandXboxController xboxController = new CommandXboxController(ControllerK.xboxPort);
    @Logged(name = "Swerve")
    private final Swerve swerve = new Swerve(vision::getVisionResults, () -> 
        Math.abs(xboxController.getLeftX()) > ControllerK.overrideThreshold
        || Math.abs(xboxController.getLeftY()) > ControllerK.overrideThreshold
        || Math.abs(xboxController.getRightX()) > ControllerK.overrideThreshold);
    @Logged(name = "Elevator")
    private final Elevator elevator = new Elevator();
    @Logged(name = "Intake")
    private final Intake intake = new Intake(() -> swerve.getChassisSpeeds().omegaRadiansPerSecond);
    // @Logged(name = "Algae")
    // private final Algae algae = new Algae();
    @Logged(name = "Climb")
    private final Climb climb = new Climb();
    private final SendableChooser<Command> autoChooser;
    private final Timer swerveCoastTimer = new Timer();
    @Logged(name = "State")
    private ElevatorK.Positions state = Positions.STOW;

    public Robot() {
        // Mostly logging setup and callbacks
        DataLog dataLog = DataLogManager.getLog();
        DriverStation.silenceJoystickConnectionWarning(true);
        Epilogue.bind(this); // Should be configured for Network Tables or DataLog
        URCL.start(Map.of(AlgaeK.sparkMaxID, "Algae Spark", IntakeK.sparkMaxLeftID, "Intake Left Spark", IntakeK.sparkMaxRightID, "Intake Right Spark")); // Can be passed dataLog to only log to DataLog
        LogUtil.logDriverStation(this); // Network Tables
        LogUtil.logCommandInterrupts(dataLog); // Network Tables & DataLog
        DriverStation.startDataLog(dataLog); // DataLog
        TalonFXLogger.refreshAllLoggedTalonFX(this, Seconds.of(kDefaultPeriod), Seconds.zero()); // Epilogue
        logGitConstants();
        Field.dumpToNT(); //? Comment out for comp
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        var xyError = NetworkTableInstance.getDefault().getDoubleTopic("/PathPlanner/XY Error (in.)").publish();
        var thetaError = NetworkTableInstance.getDefault().getDoubleTopic("/PathPlanner/Theta Error (deg.)").publish();
        PathPlannerLogging.setLogTargetPoseCallback((p) -> {
            xyError.accept(Units.metersToInches(p.getTranslation().getDistance(swerve.getPose().getTranslation())));
            thetaError.accept(p.getRotation().minus(swerve.getPose().getRotation()).getDegrees());
        });
        swerve.setDefaultCommand(teleopDriveCommand());
        configureBindings();
        autoChooser = Autos.initPathPlanner(swerve, elevator, intake);
    }

    public Command teleopDriveCommand() {
        DynamicSlewRateLimiter translationXLimiter = new DynamicSlewRateLimiter(getAccelLimit(DriveK.translationAccelLimits.getFirst()), getAccelLimit(DriveK.translationAccelLimits.getSecond()));
        DynamicSlewRateLimiter translationYLimiter = new DynamicSlewRateLimiter(getAccelLimit(DriveK.translationAccelLimits.getFirst()), getAccelLimit(DriveK.translationAccelLimits.getSecond()));
        DynamicSlewRateLimiter rotationLimiter = new DynamicSlewRateLimiter(getAccelLimit(DriveK.rotationAccelLimits.getFirst()), getAccelLimit(DriveK.rotationAccelLimits.getSecond()));
        return swerve.driveCommand(
            () -> {
                double val = MathUtil.applyDeadband(-xboxController.getLeftY(), ControllerK.leftJoystickDeadband);
                val = translationXLimiter.calculate(val);
                val = DriveUtil.powKeepSign(val, DriveK.exponentialControl);
                val *= DriveK.driveSpeedModifier;
                return val;
            }, 
            () -> {
                double val = MathUtil.applyDeadband(-xboxController.getLeftX(), ControllerK.leftJoystickDeadband);
                val = translationYLimiter.calculate(val);
                val = DriveUtil.powKeepSign(val, DriveK.exponentialControl);
                val *= DriveK.driveSpeedModifier;
                return val; 
            },  
            () -> {
                double val = MathUtil.applyDeadband(-xboxController.getRightX(), ControllerK.rightJoystickDeadband);
                val = rotationLimiter.calculate(val);
                val = DriveUtil.powKeepSign(val, DriveK.exponentialControl);
                val *= DriveK.rotationSpeedModifier;
                return val; 
            },
            true, true
        ).withName("Swerve Drive Field Oriented");
    }

    private DoubleSupplier getAccelLimit(double limit) {
        return () -> limit * DriveK.elevatorAccelTransformer.calculate(elevator.getPositionInches());
    }

    private void configureBindings() {
        Trigger paddle1 = xboxController.start();
        Trigger paddle2 = xboxController.y();
        Trigger paddle3 = xboxController.leftStick();
        Trigger paddle4 = xboxController.rightStick();
        // Testing
        // xboxController.y().whileTrue(elevator.setVoltage(Volts.of(1)).andThen(Commands.idle(elevator)).finallyDo(elevator::stop));
        // xboxController.a().whileTrue(elevator.setVoltage(Volts.of(-1)).andThen(Commands.idle(elevator)).finallyDo(elevator::stop).withName("Elevator Down"));
        // xboxController.x().onTrue(Commands.defer(() -> elevator.setPosition(elevator.getPosition().plus(Inches.of(0.25))), Set.of(elevator)).withName("Bump Up"));
        // xboxController.b().onTrue(Commands.defer(() -> elevator.setPosition(elevator.getPosition().minus(Inches.of(0.25))), Set.of(elevator)).withName("Bump Down"));
        // xboxController.a().onTrue(swerve.alignToPosePID(Field.blueReefA));
        // xboxController.rightTrigger().onTrue(elevator.setPosition(Positions.STOW));
        // xboxController.rightBumper().onTrue(elevator.setPosition(Positions.INTAKE));
        // xboxController.leftTrigger().onTrue(elevator.setPosition(Positions.L2));
        // xboxController.leftBumper().onTrue(elevator.setPosition(Positions.L4));
        // xboxController.povLeft().onTrue(elevator.setPosition(Positions.INTAKE));
        // xboxController.povRight().onTrue(intake.coralIntake());
        // xboxController.back().onTrue(elevator.zeroManual());
        
        // paddle1.onTrue(elevator.setPosition(Positions.L2));
        // xboxController.rightTrigger().onTrue(elevator.setPosition(Positions.L3));
        // xboxController.rightBumper().onTrue(elevator.setPosition(Positions.L4));

        // xboxController.povUp().onTrue(intake.scoreLevelOne());
        // xboxController.povRight().onTrue(intake.scoreLevelTwoThree());
        // xboxController.povDown().onTrue(intake.scoreLevelFour());
        // xboxController.leftTrigger().onTrue(intake.coralIntake());

        // paddle4.onTrue(algae.loweredPosition());
        // paddle3.onTrue(algae.algaePosition());
        // xboxController.leftTrigger().onTrue(algae.zero());

        // xboxController.povDown().onTrue(Commands.defer(() -> intake.scoreLevelOne(), Set.of(intake)));
        // xboxController.povUp().onTrue(elevator.setPosition(Positions.INTAKE));
        // xboxController.povRight().onTrue(intake.runOnce(intake::stop));
        // xboxController.y().onTrue(intake.scoreLevelFour());
        // xboxController.rightTrigger().onTrue(intake.setVoltage(Volts.of(4)));
        // xboxController.povUp().onTrue(algae.stow());
        // xboxController.povDown().onTrue(algae.algaePosition());
        // xboxController.povRight().onTrue(algae.zero());
        // xboxController.povLeft().onTrue(algae.runOnce(algae::stop));
        // xboxController.povRight().onTrue(algae.zero());
        xboxController.povUp().whileTrue(swerve.run(() -> swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(0.5, 0, 0, swerve.getPose().getRotation()))));
        xboxController.povRight().whileTrue(swerve.run(() -> swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(1, 0, 0, swerve.getPose().getRotation()))));
        xboxController.povDown().whileTrue(swerve.run(() -> swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(1.5, 0, 0, swerve.getPose().getRotation()))));
        xboxController.povLeft().whileTrue(swerve.run(() -> swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(2, 0, 0, swerve.getPose().getRotation()))));
        xboxController.leftTrigger().whileTrue(swerve.run(() -> swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(2.5, 0, 0, swerve.getPose().getRotation()))));
        // xboxController.leftTrigger().onTrue(swerve.setP());

        /// Real Bindings ///

        // Reset forward direction for field relative
        xboxController.back().onTrue(swerve.runOnce(swerve::zeroGyro));

        // Zeroing
        // xboxController.back().and(xboxController.start()).onTrue(Routines.zeroAll(elevator, algae, climb));
        xboxController.a().onTrue(Routines.stow(elevator, intake, climb).alongWith(Commands.runOnce(() -> state = Positions.STOW)).withName("Stow Everything"));

        // Alignment
        xboxController.x().onTrue(Routines.alignToReef(ReefSide.LEFT, swerve));
        xboxController.b().onTrue(Routines.alignToReef(ReefSide.RIGHT, swerve));
        // xboxController.a().onTrue(Routines.alignToCoralStation(swerve));
        
        // Intake
        xboxController.leftBumper().onTrue(Routines.intakeCoral(elevator, intake).alongWith(Commands.runOnce(() -> state = Positions.STOW)));

        // Reef Levels
        paddle2.onTrue(switchStateOrAction(
            Positions.L1,
            Routines.scoreCoralLevelOne(elevator, intake)
        ));
        paddle1.onTrue(switchStateOrAction(
            Positions.L2,
            Routines.scoreCoralLevelTwo(elevator, intake)
        ));

        xboxController.rightTrigger().onTrue(switchStateOrAction(
            Positions.L3,
            Routines.scoreCoralLevelThree(elevator, intake)
        ));

        xboxController.rightBumper().onTrue(switchStateOrAction(
            Positions.L4,
            Routines.scoreCoralLevelFour(elevator, intake)
        ));
        
        // Algae
        // paddle4.onTrue(Routines.algaeLowPosition(elevator, algae));
        // paddle3.onTrue(Routines.algaeHighPosition(elevator, algae));
        // xboxController.leftTrigger().onTrue(algae.loweredPosition());

        // Climb
        // xboxController.povUp().onTrue(climb.prep());
        // xboxController.povDown().onTrue(climb.climb());
        // xboxController.povRight().onTrue(climb.servoCoast());
        // xboxController.povLeft().onTrue(climb.servoRatchet());

        // xboxController.povUp().onTrue(swerve.turnCommand(Robot.onRedAlliance() ? Degrees.of(Field.redCageMid.getRotation().getDegrees()) : Degrees.of(Field.blueCageMid.getRotation().getDegrees())));
        // xboxController.povDown().onTrue(Routines.alignToCage(Cage.MIDDLE, swerve)); // Still needs to work for any cage
        // xboxController.povRight().onTrue(climb.deepclimb()); // Incase auto-alignment fails

        // xboxController.leftTrigger().onTrue(swerve.turnCommand(flipAngleAlliance(Degrees.of(Field.blueStationTop.getRotation().getDegrees() + 180))));
        // xboxController.rightTrigger().onTrue(swerve.turnCommand(flipAngleAlliance(Degrees.of(Field.blueStationLow.getRotation().getDegrees() + 180))));
        //^ Please for the love of god do not delete / touch this lest peril be upon ye of remaking it


        // xboxController.povLeft().onTrue(Routines.alignToTopCage(swerve));
        // xboxController.povUp().onTrue(Routines.alignToMidCage(swerve));
        // xboxController.povRight().onTrue(Routines.alignToLowCage(swerve));

        // SysID
        // xboxController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        // xboxController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
        // paddle2.whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // xboxController.a().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // xboxController.b().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // xboxController.x().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // xboxController.rightTrigger().whileTrue(swerve.run(() -> swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(2, 0, 0, swerve.getPose().getRotation()))));
        // xboxController.leftTrigger().whileTrue(swerve.faceWheelsForward());

        // xboxController.povDown().whileTrue(swerve.characterizeDriveWheelDiameter());
        // xboxController.povDown().whileTrue(swerve.setDriveVoltage(Volts.of(3)));
    }

    /**
     * Puts the robot into newState and performs action if the robot is already in the specified state
     * @param newState new state to put the robot in
     * @param action to perform when robot is in specified state
     */
    private Command switchStateOrAction(ElevatorK.Positions newState, Command action) {
        return Commands.runOnce(() -> {
            if (state == newState) {
                action.schedule();
                state = Positions.STOW; // Ready to take a new position
            }
            else {
                elevator.setPositionCommand(newState).alongWith(intake.coralIntake()).schedule();
                state = newState;
            }
        }).withName("Switch State");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (isAutonomous()) SmartDashboard.putString("Running Path", PathPlannerAuto.currentPathName);
        if (isDisabled() && swerveCoastTimer.hasElapsed(SwerveK.coastDisableTime.in(Seconds))) {
            swerveCoastTimer.stop();
            swerveCoastTimer.reset();
            swerve.setCoastMode();
        }
    }

    @Override
    public void autonomousInit() {
        var selected = autoChooser.getSelected();
        if (selected instanceof PathPlannerAuto auto) {
            if (!swerve.initializedOdometryFromVision()) {
                var pose = auto.getStartingPose();
                if (onRedAlliance()) {
                    pose = FlippingUtil.flipFieldPose(pose);
                }
                swerve.resetOdometry(pose);
            }
        }
        // CommandScheduler.getInstance().removeComposedCommand(selected);
        // selected.alongWith(algae.zero().andThen(algae.stow())).withName(selected.getName()).schedule();
        selected.schedule();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        swerve.stop();
        elevator.stop();
        intake.stop();
        // algae.stop();
        climb.stop();
        swerveCoastTimer.restart();
    }

    @Override
    public void disabledExit() {
        swerveCoastTimer.stop();
        swerveCoastTimer.reset();
        swerve.setBrakeMode();
    }

    /**
     * Returns the alliance
     * @return true if the robot is on Red, false if the robot is on Blue
     */
    public static boolean onRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    /**
     * Flips an angle across the x and y axis if we're on the red alliance
     * @param angle angle to flip
     * @return A supplier that returns the correct angle depending on the current alliance
     */
    public Supplier<Angle> flipAngleAlliance(Angle angle) {
        return () -> {
            Angle flippedAngle = angle.gte(Degrees.zero()) ? angle.minus(Constants.halfTurn) : angle.plus(Constants.halfTurn);
            return onRedAlliance() ? flippedAngle : angle;
        };
    }

    private void logGitConstants() {
        var table = NetworkTableInstance.getDefault().getTable("Robot").getSubTable("Git");
        table.getEntry("Project Name").setString(GitConstants.MAVEN_NAME);
        table.getEntry("Build Date").setString(GitConstants.BUILD_DATE);
        table.getEntry("Git SHA").setString(GitConstants.GIT_SHA);
        table.getEntry("Git Date").setString(GitConstants.GIT_DATE);
        table.getEntry("Git Branch").setString(GitConstants.GIT_BRANCH);
        switch (GitConstants.DIRTY) {
          case 0 -> table.getEntry("Git Dirty").setString("All changes committed");
          case 1 -> table.getEntry("Git Dirty").setString("Uncommitted changes");
          default -> table.getEntry("Git Dirty").setString("Unknown");
        }
    }

    @Logged(name = "RobotController/Battery Voltage (V)")
    public double getBatteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    @Logged(name = "RobotController/RIO Voltage (V)")
    public double getRIOVoltage() {
        return RobotController.getInputVoltage();
    }

    @Logged(name = "RobotController/RIO Current (A)")
    public double getRIOCurrent() {
        return RobotController.getInputCurrent();
    }   
}
