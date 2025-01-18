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
        config.encoder
            .positionConversionFactor(1.0/AlgaeK.gearRatio)
            .velocityConversionFactor(1.0/(AlgaeK.gearRatio*60.0)); // Divide by 60 to turn RPM into RPS
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(AlgaeK.kP, 0, AlgaeK.kD)
            .maxMotion
            .maxVelocity(AlgaeK.maxVelocity.in(RotationsPerSecond))
            .maxAcceleration(AlgaeK.maxAcceleration.in(RotationsPerSecondPerSecond))
            .allowedClosedLoopError(AlgaeK.allowableError.in(Rotations));
        sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public Command setVoltage(Voltage volts) {
        return runOnce(() -> sparkMax.setVoltage(volts)).withName("Set Voltage");
    }

    public Command algaePosition() {
        return runOnce(() -> {
            sparkMax.getClosedLoopController().setReference(AlgaeK.algaePosition.in(Rotations), ControlType.kMAXMotionPositionControl);
        })
        .andThen(Commands.waitUntil(() -> getAngle().isNear(AlgaeK.algaePosition, AlgaeK.allowableError)))
        .withName("Clear Algae Position");
    }

    public Command stow() {
        return runOnce(() -> {
            sparkMax.getClosedLoopController().setReference(AlgaeK.stowPosition.in(Rotations), ControlType.kMAXMotionPositionControl);
        })
        .andThen(Commands.waitUntil(() -> getAngle().isNear(AlgaeK.stowPosition, AlgaeK.allowableError)))
        .withName("Stow");
    }

    public Command zero() {
        return setVoltage(AlgaeK.zeroingVoltage)
            .andThen(Commands.waitUntil(limitSwitch::get))
            .andThen(() -> sparkMax.getEncoder().setPosition(AlgaeK.zeroPosition.in(Rotations)))
            .finallyDo(this::stop)
            .withName("Zero");
    }

    public void stop() {
        sparkMax.stopMotor();
    }

    public Angle getAngle() {
        return Rotations.of(sparkMax.getEncoder().getPosition());
    }

}
