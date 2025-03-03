// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeK;
import frc.robot.Constants.ControllerK;
import frc.robot.Constants.ElevatorK.Positions;
import frc.robot.Constants.IntakeK;
import frc.robot.Constants.SwerveK;
import frc.robot.commands.Autos;
import frc.robot.commands.Routines;
import frc.robot.lib.logging.LogUtil;
import frc.robot.lib.logging.TalonFXLogger;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
    
    @Logged(name = "Vision")
    private final Vision vision = new Vision();
    private final CommandXboxController xboxController = new CommandXboxController(ControllerK.xboxPort);
    @Logged(name = "Swerve")
    private final Swerve swerve = new Swerve(vision::getVisionResults, () -> 
        xboxController.getLeftX() > ControllerK.overrideThreshold
        || xboxController.getLeftY() > ControllerK.overrideThreshold
        || xboxController.getRightX() > ControllerK.overrideThreshold);
    @Logged(name = "Elevator")
    private final Elevator elevator = new Elevator();
    @Logged(name = "Intake")
    private final Intake intake = new Intake();
    @Logged(name = "Algae")
    private final Algae algae = new Algae();
    // private final Climb climb = new Climb();
    private final SendableChooser<Command> autoChooser;
    private final Timer swerveCoastTimer = new Timer();
    
    @Logged(name = "Score Ready")
    boolean isScoreReady = true;

    public Robot() {
        DataLog dataLog = DataLogManager.getLog();
        DriverStation.silenceJoystickConnectionWarning(true);
        Epilogue.bind(this); // Should be configured for Network Tables or DataLog
        URCL.start(Map.of(AlgaeK.sparkMaxID, "Algae Spark", IntakeK.sparkMaxLeftID, "Intake Left Spark", IntakeK.sparkMaxRightID, "Intake Right Spark")); // Can be passed dataLog to only log to DataLog
        LogUtil.logDriverStation(this); // Network Tables
        LogUtil.logCommandInterrupts(dataLog); // Network Tables & DataLog
        DriverStation.startDataLog(dataLog); // DataLog
        TalonFXLogger.refreshAllLoggedTalonFX(this, Seconds.of(kDefaultPeriod), Seconds.zero()); // Epilogue
        logGitConstants();
        // SmartDashboard.putData(new PowerDistribution()); // TODO: Wait until we get a new PDH
        swerve.setDefaultCommand(swerve.teleopDriveCommand(() -> -xboxController.getLeftY(), () -> -xboxController.getLeftX(), () -> -xboxController.getRightX()));
        configureBindings();
        autoChooser = Autos.initPathPlanner(swerve, elevator, intake);
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

    private void configureBindings() {
        Trigger paddle1 = xboxController.start();
        Trigger paddle2 = xboxController.y();
        Trigger paddle3 = xboxController.leftStick();
        Trigger paddle4 = xboxController.rightStick();
        // Testing
        // xboxController.y().whileTrue(elevator.setVoltage(Volts.of(1)).andThen(Commands.idle(elevator)).finallyDo(elevator::stop));
        xboxController.a().whileTrue(elevator.setVoltage(Volts.of(-1)).andThen(Commands.idle(elevator)).finallyDo(elevator::stop).withName("Elevator Down"));
        xboxController.x().onTrue(Commands.defer(() -> elevator.setPosition(elevator.getPosition().plus(Inches.of(0.25))), Set.of(elevator)).withName("Bump Up"));
        xboxController.b().onTrue(Commands.defer(() -> elevator.setPosition(elevator.getPosition().minus(Inches.of(0.25))), Set.of(elevator)).withName("Bump Down"));
        // xboxController.rightTrigger().onTrue(elevator.setPosition(Positions.STOW));
        // xboxController.rightBumper().onTrue(elevator.setPosition(Positions.INTAKE));
        // xboxController.leftTrigger().onTrue(elevator.setPosition(Positions.L2));
        // xboxController.leftBumper().onTrue(elevator.setPosition(Positions.L4));
        // xboxController.back().onTrue(elevator.zeroManual());

        // paddle2.onTrue(elevator.setPosition(Positions.L1));
        // if (isScoreReady) paddle2.onTrue(Commands.parallel(updateControllerState(), Routines.scoreCoralLevelOne(elevator, intake)));
        // else paddle2.onTrue(Commands.parallel(updateControllerState(), elevator.setPosition(Positions.L1)));
        paddle2.and(() -> isScoreReady).onTrue(Routines.scoreCoralLevelOne(elevator, intake).alongWith(updateControllerState())).onFalse(elevator.setPosition(Positions.L1));
        paddle1.and(() -> isScoreReady).onTrue(Routines.scoreCoralLevelTwo(elevator, intake).alongWith(updateControllerState())).onFalse(elevator.setPosition(Positions.L2));
        xboxController.rightTrigger().and(() -> isScoreReady).onTrue(Routines.scoreCoralLevelThree(elevator, intake).alongWith(updateControllerState())).onFalse(elevator.setPosition(Positions.L3));
        xboxController.rightBumper().and(() -> isScoreReady).onTrue(Routines.scoreCoralLevelFour(elevator, intake).alongWith(updateControllerState())).onFalse(elevator.setPosition(Positions.L4));
        
        // paddle1.onTrue(elevator.setPosition(Positions.L2));
        // xboxController.rightTrigger().onTrue(elevator.setPosition(Positions.L3));
        // xboxController.rightBumper().onTrue(elevator.setPosition(Positions.L4));

        // xboxController.povUp().onTrue(intake.scoreLevelOne());
        // xboxController.povRight().onTrue(intake.scoreLevelTwoThree());
        // xboxController.povDown().onTrue(intake.scoreLevelFour());
        // xboxController.leftTrigger().onTrue(intake.coralIntake());

        paddle4.onTrue(algae.loweredPosition());
        paddle3.onTrue(algae.algaePosition());
        xboxController.leftTrigger().onTrue(algae.zero());

        // xboxController.povDown().onTrue(Commands.defer(() -> intake.scoreLevelOne(), Set.of(intake)));
        // xboxController.povUp().onTrue(elevator.setPosition(Positions.INTAKE));
        // xboxController.povRight().onTrue(intake.runOnce(intake::stop));
        // xboxController.y().onTrue(intake.scoreLevelFour());
        // xboxController.rightTrigger().onTrue(intake.setVoltage(Volts.of(4)));
        // xboxController.povUp().onTrue(algae.stow());
        // xboxController.povDown().onTrue(algae.algaePosition());
        // xboxController.povRight().onTrue(algae.zero());
        // xboxController.povLeft().onTrue(algae.runOnce(algae::stop));

        // Reset forward direction for field relative
        xboxController.back().onTrue(swerve.runOnce(swerve::zeroGyro));

        // Zeroing
        // xboxController.back().and(xboxController.start()).onTrue(Routines.zeroAll(elevator, algae, climb));
        xboxController.a().onTrue(Routines.stow(elevator, intake, algae));

        // Alignment
        // xboxController.x().onTrue(Routines.alignToLeftReef(swerve));
        // xboxController.b().onTrue(Routines.alignToRightReef(swerve));
        // xboxController.a().onTrue(Routines.alignToCoralStation(swerve));

        // Intake
        xboxController.leftBumper().onTrue(Routines.intakeCoral(elevator, intake));

        // Reef Levels
        // paddle2.onTrue(elevator.setPosition(Positions.L1));
        // paddle1.onTrue(elevator.setPosition(Positions.L2));
        // xboxController.rightTrigger().onTrue(elevator.setPosition(Positions.L3));
        // xboxController.rightBumper().onTrue(elevator.setPosition(Positions.L4));

        // Algae
        // paddle4.onTrue(Routines.algaeLowPosition(elevator, algae));
        // paddle3.onTrue(Routines.algaeHighPosition(elevator, algae));
        // xboxController.leftTrigger().onTrue(algae.loweredPosition());

        // Climb
        // xboxController.povUp().onTrue(swerve.turnCommand(Robot.onRedAlliance() ? Degrees.of(Field.redCageMid.getRotation().getDegrees()) : Degrees.of(Field.blueCageMid.getRotation().getDegrees())));
        // xboxController.povDown().onTrue(Routines.alignToMidCage(swerve).andThen(climb.deepclimb())); // Still needs to work for any cage
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
        // xboxController.y().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // xboxController.a().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // xboxController.b().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // xboxController.x().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // xboxController.rightTrigger().whileTrue(swerve.run(() -> swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(1, 0, 0, swerve.getPose().getRotation()))));

        // xboxController.y().whileTrue(swerve.characterizeDriveWheelDiameter());
    }

    private Command updateControllerState() {
        return Commands.runOnce(() -> isScoreReady = !isScoreReady, new Subsystem[0]);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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
        selected.schedule();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        swerve.stop();
        elevator.stop();
        intake.stop();
        algae.stop();
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
