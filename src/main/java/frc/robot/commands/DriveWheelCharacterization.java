package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveK;
import frc.robot.subsystems.Swerve;

/**
 * Taken from https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/commands/WheelRadiusCharacterization.java
 */
public class DriveWheelCharacterization extends Command {
    
    private final ChassisSpeeds speeds = new ChassisSpeeds(MetersPerSecond.zero(), MetersPerSecond.zero(), DegreesPerSecond.of(45));
    private final Swerve swerve;
    private double[] initialWheelPositions;
    private Angle initialYaw;
    
    public DriveWheelCharacterization(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        initialWheelPositions = swerve.getWheelPositions();
        initialYaw = swerve.getYaw();
    }

    @Override
    public void execute() {
        swerve.setChassisSpeeds(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        Angle accumulatedYaw = swerve.getYaw().minus(initialYaw);
        double averageWheelDelta = 0.0;
        double[] wheelPositions = swerve.getWheelPositions();
        for (int i = 0; i < wheelPositions.length; i++) {
            averageWheelDelta += Math.abs(wheelPositions[i] - initialWheelPositions[i]);
        }
        averageWheelDelta /= 4.0;
        double wheelRadiusInches = accumulatedYaw.abs(Radians) * SwerveK.driveBaseRadius.in(Inches) / averageWheelDelta;
        System.out.println("Accumulated Yaw: " + accumulatedYaw.toLongString());
        System.out.println("Drive Base Radius: " + SwerveK.driveBaseRadius.toLongString());
        System.out.println("Average Wheel Delta (Rads)" + averageWheelDelta);
        System.out.println("Found the wheel diameter to be " + wheelRadiusInches * 2 + " inches.");
    }

}
