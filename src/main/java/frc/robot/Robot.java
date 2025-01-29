// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import java.util.Map;

import java.util.Map;

import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeK;
import frc.robot.Constants.ControllerK;
import frc.robot.Constants.DriveK;
import frc.robot.Constants.IntakeK;
import frc.robot.commands.Autos;
import frc.robot.lib.logging.LogUtil;
import frc.robot.lib.logging.TalonFXLogger;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
    
    @Logged(name = "Vision")
    private final Vision vision = new Vision();
    @Logged(name = "Swerve")
    private final Swerve swerve = new Swerve(vision::getVisionResults);
    private final CommandXboxController xboxController = new CommandXboxController(ControllerK.xboxPort);
    private final SendableChooser<Command> autoChooser;

    // ===========
    public static Field2d alignmentTest = new Field2d();
    // ===========
    
    public Robot() {
        DataLog dataLog = DataLogManager.getLog();
        DriverStation.silenceJoystickConnectionWarning(true);
        Epilogue.bind(this); // Should be configured for Network Tables or DataLog
        URCL.start(Map.of(AlgaeK.sparkMaxID, "Algae Spark", IntakeK.motorLeftId, "Intake Left Spark", IntakeK.motorRightId, "Intake Right Spark")); // Can be passed dataLog to only log to DataLog
        LogUtil.logDriverStation(this); // Network Tables
        LogUtil.logCommandInterrupts(dataLog); // Network Tables & DataLog
        DriverStation.startDataLog(dataLog); // DataLog
        TalonFXLogger.refreshAllLoggedTalonFX(this, Seconds.of(kDefaultPeriod), Seconds.zero()); // Epilogue
        logGitConstants();
        Command driveFieldOriented = swerve.driveCommand(
            () -> DriveK.translationalYLimiter.calculate(MathUtil.applyDeadband(-xboxController.getLeftY(), ControllerK.leftJoystickDeadband)), 
            () -> DriveK.translationalXLimiter.calculate(MathUtil.applyDeadband(-xboxController.getLeftX(), ControllerK.leftJoystickDeadband)),  
            () -> DriveK.rotationalLimiter.calculate(MathUtil.applyDeadband(-xboxController.getRightX(), ControllerK.rightJoystickDeadband)),
            true
        ).withName("Swerve Drive Field Oriented");
        swerve.setDefaultCommand(driveFieldOriented);
        configureBindings();
        autoChooser = Autos.initPathPlanner(swerve);

        swerve.resetOdometry(new Pose2d(Field.blueReefB.getMeasureX().minus(Field.blueStationLow.getMeasureX()), Field.blueReefB.getMeasureY().minus(Field.blueStationLow.getMeasureY()), Rotation2d.fromDegrees(0)));

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
        // Reset forward direction for field relative
        xboxController.back().onTrue(swerve.runOnce(swerve::zeroGyro));
        // D-pad snap turning
        // xboxController.povUp().onTrue(swerve.turnCommand(Degrees.zero())); 
        // xboxController.povUpLeft().onTrue(swerve.turnCommand(Degrees.of(45))); 
        // xboxController.povLeft().onTrue(swerve.turnCommand(Degrees.of(90))); 
        // xboxController.povDownLeft().onTrue(swerve.turnCommand(Degrees.of(135))); 
        // xboxController.povDown().onTrue(swerve.turnCommand(Degrees.of(180))); 
        // xboxController.povDownRight().onTrue(swerve.turnCommand(Degrees.of(-135))); 
        // xboxController.povRight().onTrue(swerve.turnCommand(Degrees.of(-90))); 
        // xboxController.povUpRight().onTrue(swerve.turnCommand(Degrees.of(-45))); 

        // Alignment
        xboxController.a().onTrue(Commands.defer(swerve::alignToCoralStation, Set.of(swerve)));
        xboxController.b().onTrue(Commands.defer(swerve::alignToReef, Set.of(swerve))); //align to reef
        xboxController.povUp().onTrue(Commands.defer(swerve::alignToTopCage, Set.of(swerve))); // top cage
        xboxController.povRight().onTrue(Commands.defer(swerve::alignToMidCage, Set.of(swerve))); // mid cage
        xboxController.povDown().onTrue(Commands.defer(swerve::alignToLowCage, Set.of(swerve))); // low cage


        // SysID
        // xboxController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        // xboxController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
        // xboxController.y().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // xboxController.a().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // xboxController.b().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // xboxController.x().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // xboxController.rightTrigger().whileTrue(swerve.run(() -> swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(1, 0, 0, swerve.getPose().getRotation()))));
        // xboxController.leftTrigger().whileTrue(swerve.characterizeDriveWheelDiameter());
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putData("Field", alignmentTest);
        alignmentTest.getObject("Blue Cage Top").setPose(Field.blueCageTop);
        alignmentTest.getObject("Blue Cage Mid").setPose(Field.blueCageMid);
        alignmentTest.getObject("Blue Cage Low").setPose(Field.blueCageLow);
        alignmentTest.getObject("Blue Station Top").setPose(Field.blueStationTop); 
        alignmentTest.getObject("Blue Station Low").setPose(Field.blueStationLow); 
        alignmentTest.getObject("red Station Top").setPose(Field.redStationTop); 
        alignmentTest.getObject("red Station Low").setPose(Field.redStationLow); 


        alignmentTest.getObject("Robot Pose").setPose(swerve.getPose());
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
    
}
