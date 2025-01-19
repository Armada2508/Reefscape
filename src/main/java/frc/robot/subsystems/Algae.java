package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeK;

@Logged
public class Algae extends SubsystemBase {

    private final SparkMax sparkMax = new SparkMax(AlgaeK.sparkMaxID, MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(AlgaeK.limitSwitchID);

    public Algae() {
        SparkMaxConfig config = new SparkMaxConfig();
        // These conversion factors cause all values to be in reference to the mechanism instead of the motor
        config.encoder
            .positionConversionFactor(1.0 / AlgaeK.gearRatio) // Converts rotations of the motor into rotations of the mechanism
            .velocityConversionFactor(1.0 / (AlgaeK.gearRatio * 60.0)); // Divide by 60 to turn RPM into RPS
        config.idleMode(IdleMode.kBrake);
        config.softLimit //? The docs are unclear whether these limits are affected by the conversion factors
            .forwardSoftLimit(AlgaeK.maxPosition.in(Rotations))
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(AlgaeK.zeroPosition.in(Rotations))
            .reverseSoftLimitEnabled(true);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(AlgaeK.kP, 0, AlgaeK.kD)
            .maxMotion
            .maxVelocity(AlgaeK.maxVelocity.in(RotationsPerSecond))
            .maxAcceleration(AlgaeK.maxAcceleration.in(RotationsPerSecondPerSecond))
            .allowedClosedLoopError(AlgaeK.allowableError.in(Rotations));
        sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    /**
     * Sets the voltage of the arm motor
     * @param volts Voltage to set the arm motor to
     * @return A command that finishes immediately and sets the voltage of the arm
     */
    public Command setVoltage(Voltage volts) {
        return runOnce(() -> sparkMax.setVoltage(volts)).withName("Set Voltage");
    }

    /**
     * Commands the arm into the position to knock the algae off and waits until it's within the allowable error
     * @return A command to put the arm in clear algae position
     */
    public Command algaePosition() {
        return runOnce(() -> {
            sparkMax.getClosedLoopController().setReference(AlgaeK.algaePosition.in(Rotations), ControlType.kMAXMotionPositionControl);
        })
        .andThen(Commands.waitUntil(() -> getAngle().isNear(AlgaeK.algaePosition, AlgaeK.allowableError)))
        .withName("Clear Algae Position");
    }

    /**
     * Commands the arm into stow position and waits until it's within the allowable error
     * @return A command to put the arm in stow position
     */
    public Command stow() {
        return runOnce(() -> {
            sparkMax.getClosedLoopController().setReference(AlgaeK.stowPosition.in(Rotations), ControlType.kMAXMotionPositionControl);
        })
        .andThen(Commands.waitUntil(() -> getAngle().isNear(AlgaeK.stowPosition, AlgaeK.allowableError)))
        .withName("Stow");
    }

    /**
     * Runs the arm backwards until it hits the limit switch and zeroes the encoder
     * @return A command to zero the arm
     */
    public Command zero() {
        return setVoltage(AlgaeK.zeroingVoltage)
            .andThen(Commands.waitUntil(limitSwitch::get))
            .andThen(() -> sparkMax.getEncoder().setPosition(AlgaeK.zeroPosition.in(Rotations)))
            .finallyDo(this::stop)
            .withName("Zero");
    }

    /**
     * Stops the arm from moving
     */
    public void stop() {
        sparkMax.stopMotor();
    }

    /**
     * Returns the angle of the arm
     * @return angle of the arm
     */
    public Angle getAngle() {
        return Rotations.of(sparkMax.getEncoder().getPosition());
    }

}
