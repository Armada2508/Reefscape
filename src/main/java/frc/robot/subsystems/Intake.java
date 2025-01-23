package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeK;
import frc.robot.lib.util.Util;

public class Intake extends SubsystemBase {
    
    private final SparkMax motorLeft = new SparkMax(IntakeK.motorLeftId, MotorType.kBrushless);
    private final SparkMax motorRight = new SparkMax(IntakeK.motorRightId, MotorType.kBrushless);
    private final TimeOfFlight timeOfFlight = new TimeOfFlight(IntakeK.timeOfFlightId);

    public Intake() {
        configSparkMax();
    }

    public void configSparkMax() {
        SparkMaxConfig motorLeftConfig = new SparkMaxConfig();
        SparkMaxConfig motorRightConfig = new SparkMaxConfig();

        motorLeft.configure(motorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorRight.configure(motorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean isSensorTripped() {
        return Util.inRange(timeOfFlight.getRange(), IntakeK.coralDetectionRange.in(Inches)) && timeOfFlight.isRangeValid();
    }

    public Command coralIntake() {
        return runOnce(() -> {
            motorLeft.setVoltage(IntakeK.coralIntakeVolts);
            motorRight.setVoltage(IntakeK.coralIntakeVolts);
        })
        .andThen(Commands.waitUntil(this::isSensorTripped))
        .andThen(this::stop);
    }
    //^ () -> !isSensorTripped() | Supplier to check when the sensor is no longer being tripped

    public Command scoreLevelOne() {
        return runOnce(() -> {
            motorLeft.setVoltage(IntakeK.levelOneVolts); //MAKE THIS TAKE IN NEGATIVE VOLTS
            motorRight.setVoltage(IntakeK.levelOneVolts.unaryMinus()); //opposite of motorLeft
        })
        .andThen(Commands.waitUntil(() -> !isSensorTripped()))
        .andThen(this::stop);
    }

    public Command scoreLevelTwoThree() {
        return runOnce(() -> {
            motorLeft.setVoltage(IntakeK.levelTwoThreeVolts);
            motorRight.setVoltage(IntakeK.levelTwoThreeVolts);
        })
        .andThen(Commands.waitUntil(() -> !isSensorTripped()))
        .andThen(this::stop);

    }

    public Command scoreLevelFour() {
        return runOnce (() -> {
            motorLeft.setVoltage(IntakeK.levelFourVolts);
            motorRight.setVoltage(IntakeK.levelFourVolts);
        })
        .andThen(Commands.waitUntil(() -> !isSensorTripped()))
        .andThen(this::stop);
    }

    public Command setVoltage(Voltage volts) {
        return runOnce (() -> {
            motorLeft.setVoltage(volts);
            motorRight.setVoltage(volts);
        });
    }

    public void stop() {
        motorLeft.stopMotor();
        motorRight.stopMotor();
    }

}
