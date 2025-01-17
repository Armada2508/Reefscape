// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerK;
import frc.robot.Constants.DriveK;
import frc.robot.subsystems.Swerve;

@Logged
public class Robot extends TimedRobot {
    
    private final Swerve swerve = new Swerve();
    private final CommandXboxController xboxController = new CommandXboxController(ControllerK.xboxPort);
    
    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        Epilogue.bind(this);
        configureBindings();
        Command driveFieldOriented = swerve.driveCommand(
            () -> DriveK.translationalYLimiter.calculate(MathUtil.applyDeadband(-xboxController.getLeftY(), ControllerK.leftJoystickDeadband)), 
            () -> DriveK.translationalXLimiter.calculate(MathUtil.applyDeadband(-xboxController.getLeftX(), ControllerK.leftJoystickDeadband)),  
            () -> DriveK.rotationalLimiter.calculate(MathUtil.applyDeadband(-xboxController.getRightX(), ControllerK.rightJoystickDeadband)),
            true
        ).withName("Swerve Drive Field Oriented");
        swerve.setDefaultCommand(driveFieldOriented);
    }

    private void configureBindings() {
        // Reset forward direction for field relative
        xboxController.x().and(xboxController.b()).onTrue(swerve.runOnce(swerve::zeroGyro));
        // D-pad snap turning
        xboxController.povUp().onTrue(swerve.turnCommand(Degrees.zero())); 
        xboxController.povUpLeft().onTrue(swerve.turnCommand(Degrees.of(45))); 
        xboxController.povLeft().onTrue(swerve.turnCommand(Degrees.of(90))); 
        xboxController.povDownLeft().onTrue(swerve.turnCommand(Degrees.of(135))); 
        xboxController.povDown().onTrue(swerve.turnCommand(Degrees.of(180))); 
        xboxController.povDownRight().onTrue(swerve.turnCommand(Degrees.of(-135))); 
        xboxController.povRight().onTrue(swerve.turnCommand(Degrees.of(-90))); 
        xboxController.povUpRight().onTrue(swerve.turnCommand(Degrees.of(-45))); 
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        swerve.stop();
    }
    
}
