// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerK;
import frc.robot.Constants.DriveK;
import frc.robot.lib.logging.TalonFXLogger;
import frc.robot.subsystems.Swerve;

@Logged
public class Robot extends TimedRobot {
    
    private final Swerve swerve = new Swerve();
    private final CommandXboxController xboxController = new CommandXboxController(ControllerK.xboxPort);
    private final SendableChooser<Command> autoChooser;
    
    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        Epilogue.bind(this);
        TalonFXLogger.refreshAllLoggedTalonFX(this, Seconds.of(kDefaultPeriod), Seconds.zero());
        Command driveFieldOriented = swerve.driveCommand(
            () -> DriveK.translationalYLimiter.calculate(MathUtil.applyDeadband(-xboxController.getLeftY(), ControllerK.leftJoystickDeadband)), 
            () -> DriveK.translationalXLimiter.calculate(MathUtil.applyDeadband(-xboxController.getLeftX(), ControllerK.leftJoystickDeadband)),  
            () -> DriveK.rotationalLimiter.calculate(MathUtil.applyDeadband(-xboxController.getRightX(), ControllerK.rightJoystickDeadband)),
            true
        ).withName("Swerve Drive Field Oriented");
        swerve.setDefaultCommand(driveFieldOriented);
        FollowPathCommand.warmupCommand().schedule();

        // var trigger = new EventTrigger("score coral").onTrue(Commands.print("Scoring CORAL!").andThen(Commands.waitSeconds(3)));
        // var trigger2 = new EventTrigger("intake coral").onTrue(Commands.print("Intaking CORAL!").andThen(Commands.waitSeconds(2)));
        NamedCommands.registerCommand("score coral", Commands.print("Scoring CORAL!").andThen(Commands.waitSeconds(2)));
        NamedCommands.registerCommand("intake coral", Commands.print("Intaking CORAL!").andThen(Commands.waitSeconds(2)));
        new EventTrigger("raise elevator").onTrue(Commands.print("Raising Elevator!"));
        autoChooser = AutoBuilder.buildAutoChooser("Score 2 CORAL");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Reset forward direction for field relative
        xboxController.back().onTrue(swerve.runOnce(swerve::zeroGyro));
        // D-pad snap turning
        xboxController.povUp().onTrue(swerve.turnCommand(Degrees.zero())); 
        xboxController.povUpLeft().onTrue(swerve.turnCommand(Degrees.of(45))); 
        xboxController.povLeft().onTrue(swerve.turnCommand(Degrees.of(90))); 
        xboxController.povDownLeft().onTrue(swerve.turnCommand(Degrees.of(135))); 
        xboxController.povDown().onTrue(swerve.turnCommand(Degrees.of(180))); 
        xboxController.povDownRight().onTrue(swerve.turnCommand(Degrees.of(-135))); 
        xboxController.povRight().onTrue(swerve.turnCommand(Degrees.of(-90))); 
        xboxController.povUpRight().onTrue(swerve.turnCommand(Degrees.of(-45))); 

        // SysID
        // xboxController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        // xboxController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
        // xboxController.y().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // xboxController.a().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // xboxController.b().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // xboxController.x().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // xboxController.rightTrigger().whileTrue(swerve.run(() -> swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(1, 0, 0, swerve.getPose().getRotation()))));
        // xboxController.leftTrigger().onTrue(new DriveWheelCharacterization(swerve, xboxController.leftBumper()));
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        autoChooser.getSelected().schedule();
    }

    @Override
    public void disabledInit() {
        swerve.stop();
    }
    
}
