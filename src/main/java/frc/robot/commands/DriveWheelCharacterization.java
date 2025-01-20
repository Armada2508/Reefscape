package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveK;
import frc.robot.subsystems.Swerve;

public class DriveWheelCharacterization extends Command {
    
    private final Swerve swerve;
    private final BooleanSupplier cancelCondition;
    private double[] initialWheelPositions;
    private Angle initialYaw;
    
    public DriveWheelCharacterization(Swerve swerve, BooleanSupplier cancelCondition) {
        this.swerve = swerve;
        this.cancelCondition = cancelCondition;
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        initialWheelPositions = swerve.getWheelPositions();
        initialYaw = swerve.getYaw();
    }

    @Override
    public void execute() {
        swerve.setChassisSpeeds(new ChassisSpeeds(MetersPerSecond.zero(), MetersPerSecond.zero(), DegreesPerSecond.of(45)));
    }

    @Override
    public boolean isFinished() {
        return cancelCondition.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        Angle accumulatedYaw = swerve.getYaw().minus(initialYaw);
        double averageWheelPosition = 0.0;
        double[] wheelPositions = swerve.getWheelPositions();
        for (int i = 0; i < wheelPositions.length; i++) {
            averageWheelPosition += Math.abs(wheelPositions[i] - initialWheelPositions[i]);
        }
        averageWheelPosition /= 4.0;
        double wheelRadiusInches = accumulatedYaw.abs(Radians) * SwerveK.driveBaseRadius.in(Inches) / averageWheelPosition;
        System.out.println("Accumulated Yaw: " + accumulatedYaw.toLongString());
        System.out.println("Drive Base Radius: " + SwerveK.driveBaseRadius.toLongString());
        System.out.println("Average Wheel Distance (Rads)" + averageWheelPosition);
        System.out.println("Found the wheel diameter to be " + wheelRadiusInches * 2 + " inches.");
    }

}
