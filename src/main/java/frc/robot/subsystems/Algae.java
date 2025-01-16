package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AlgaeK;

public class Algae extends SubsystemBase {

    private final SparkMax sparkMax = new SparkMax(0, MotorType.kBrushless);

    public Algae() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop // Need to tune all this
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(AlgaeK.kP, 0, AlgaeK.kD)
            .maxMotion
            .maxVelocity(AlgaeK.maxVelocity)
            .maxAcceleration(AlgaeK.maxAcceleration)
            .allowedClosedLoopError(0);
        sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public Command setVoltage(Voltage volts) {
        return runOnce(() -> sparkMax.setVoltage(volts));
    }

    public Command algaePosition() {
        return runOnce(() -> {
            sparkMax.getClosedLoopController().setReference(AlgaeK.algaePosition.in(Rotations), ControlType.kMAXMotionPositionControl);
        });
    }

    public Command stow() {
        return runOnce(() -> {
            sparkMax.getClosedLoopController().setReference(AlgaeK.stowPosition.in(Rotations), ControlType.kMAXMotionPositionControl);
        });
    }

    public Command zero() {
        return setVoltage(Volts.of(-3)).andThen(new WaitUntilCommand(0));
    }

}
