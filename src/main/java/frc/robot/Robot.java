// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerK;
import frc.robot.Constants.DriveK;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {
    
    Swerve swerve = new Swerve();
    CommandXboxController xboxController = new CommandXboxController(ControllerK.xboxPort);
    private static Field2d reefTest = new Field2d();
    
    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        addPeriodic(() -> CommandScheduler.getInstance().run(), kDefaultPeriod);
        configureBindings();
        Command driveFieldOrientedAngularVelocity = swerve.driveCommand(
            () -> DriveK.translationalYLimiter.calculate(MathUtil.applyDeadband(-xboxController.getLeftY(), ControllerK.leftJoystickDeadband)), 
            () -> DriveK.translationalXLimiter.calculate(MathUtil.applyDeadband(-xboxController.getLeftX(), ControllerK.leftJoystickDeadband)),  
            () -> DriveK.rotationalLimiter.calculate(MathUtil.applyDeadband(-xboxController.getRightX(), ControllerK.rightJoystickDeadband)),
            false
        );

        swerve.setDefaultCommand(driveFieldOrientedAngularVelocity);

        Pose2d robotPose = swerve.getPose();
        reefTest.getObject("Robot Pose").setPose(robotPose);
        
    }

    private void configureBindings() {
        // Reset forward direction for field relative
        xboxController.x().and(xboxController.b()).onTrue(swerve.runOnce(swerve::zeroGyro)); 
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

    }
    
}
