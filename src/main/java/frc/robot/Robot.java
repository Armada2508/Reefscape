// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeK;
import frc.robot.Constants.ControllerK;
import frc.robot.Constants.DriveK;
import frc.robot.Constants.IntakeK;
import frc.robot.commands.Autos;
import frc.robot.lib.logging.LogUtil;
import frc.robot.lib.logging.TalonFXLogger;
import frc.robot.subsystems.Elevator;
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
    private final Elevator elevator = new Elevator();
    // private final Climb climb = new Climb();
    // private final Algae algae = new Algae();
    // private final Intake intake = new Intake();
    private final SendableChooser<Command> autoChooser;
    
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
        Command driveFieldOriented = swerve.driveCommand(
            () -> DriveK.translationalYLimiter.calculate(MathUtil.applyDeadband(-xboxController.getLeftY(), ControllerK.leftJoystickDeadband)) * DriveK.driveSpeedModifier, 
            () -> DriveK.translationalXLimiter.calculate(MathUtil.applyDeadband(-xboxController.getLeftX(), ControllerK.leftJoystickDeadband)) * DriveK.driveSpeedModifier,  
            () -> DriveK.rotationalLimiter.calculate(MathUtil.applyDeadband(-xboxController.getRightX(), ControllerK.rightJoystickDeadband)) * DriveK.rotationSpeedModifier,
            true, true
        ).withName("Swerve Drive Field Oriented");
        swerve.setDefaultCommand(driveFieldOriented);
        configureBindings();
        autoChooser = Autos.initPathPlanner(swerve);
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
        Trigger paddle1 = xboxController.back();
        Trigger paddle2 = xboxController.y();
        Trigger paddle3 = xboxController.leftStick();
        Trigger paddle4 = xboxController.rightStick();
        // Testing
        xboxController.y().onTrue(elevator.setVoltage(Volts.of(3)).andThen(Commands.idle(elevator)).finallyDo(elevator::stop));
        xboxController.a().whileTrue(elevator.setVoltage(Volts.of(-3)).andThen(Commands.idle(elevator)).finallyDo(elevator::stop));
        
        // Reset forward direction for field relative
        // xboxController.start().onTrue(swerve.runOnce(swerve::zeroGyro);

        // Zeroing
        // xboxController.back().and(xboxController.start()).onTrue(Routines.zeroAll(elevator, algae, climb));

        // // Alignment
        // xboxController.x().onTrue(Routines.alignToLeftReef(swerve));
        // xboxController.b().onTrue(Routines.alignToRightReef(swerve));
        // xboxController.a().onTrue(Routines.alignToCoralStation(swerve));

        // // Intake
        // xboxController.leftBumper().onTrue(Routines.intakeCoral(elevator, intake));

        // // Reef Levels
        // paddle2.onTrue(Routines.scoreCoralLevelOne(elevator, intake));
        // paddle1.onTrue(Routines.scoreCoralLevelTwo(elevator, intake));
        // xboxController.rightBumper().onTrue(Routines.scoreCoralLevelThree(elevator, intake));
        // xboxController.rightTrigger().onTrue(Routines.scoreCoralLevelFour(elevator, intake));

        // // Algae
        // paddle4.onTrue(Routines.algaeLowPosition(elevator, algae));
        // paddle3.onTrue(Routines.algaeHighPosition(elevator, algae));
        // xboxController.leftTrigger().onTrue(algae.loweredPosition());

        // // Climb
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
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        PathPlannerAuto auto = (PathPlannerAuto) autoChooser.getSelected();
        if (!swerve.initializedOdometryFromVision()) {
            AutoBuilder.resetOdom(auto.getStartingPose());
        }
        auto.schedule();
    }

    @Override
    public void disabledInit() {
        swerve.stop();
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
    
}
