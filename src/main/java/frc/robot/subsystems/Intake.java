package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeK;
import frc.robot.lib.util.Util;

@Logged
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

        motorLeftConfig.smartCurrentLimit(IntakeK.currentLimit);
        motorRightConfig.smartCurrentLimit(IntakeK.currentLimit);
        motorRightConfig.inverted(true);

        motorLeft.configure(motorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorRight.configure(motorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * True when ToF sensor is tripped
     * @return When milimeters are equal to the coralDetectionRange, 
     */
    public boolean isSensorTripped() {
        Millimeters.of(1).lte(IntakeK.coralDetectionRange);
        return Util.inRange(timeOfFlight.getRange(), IntakeK.coralDetectionRange.in(Inches)) && timeOfFlight.isRangeValid();
    }

    /**
     * Stops motors
     */
    public void stop() {
        motorLeft.stopMotor();
        motorRight.stopMotor();
    }

    /**
     * Sets voltage to motors
     * @param volts Voltage to give motors
     * @return Command to set voltage, motorRight runs opposite of motorLeft
     */
    public Command setVoltage(Voltage volts) {
        return runOnce (() -> {
            motorLeft.setVoltage(volts);
            motorRight.setVoltage(volts);
        });
    }

    /**
     * Sets voltage to motors for scoring.
     * @param volts Voltage to give motors
     * @return Command to set voltage and stop when ToF is tripped
     */
    public Command score(Voltage volts) {
        return setVoltage(volts)
        .andThen(Commands.waitUntil(() -> !isSensorTripped()))
        .finallyDo(this::stop);
    }

    /**
     * Sets voltage to motors for intake.
     * @return Command to set voltage and stop when time of flight is tripped
     */
    public Command coralIntake() {
        return runOnce(() -> {
            motorLeft.setVoltage(IntakeK.coralIntakeVolts);
            motorRight.setVoltage(IntakeK.coralIntakeVolts);
        })
        .andThen(Commands.waitUntil(this::isSensorTripped))
        .andThen(this::stop);
    }

    /**
     * Sets voltage for motors scoring level one
     * @return Command to set voltage with both motors in same direction and stop when ToF is tripped
     */
    public Command scoreLevelOne() {
        return runOnce(() -> {
            motorLeft.setVoltage(IntakeK.levelOneVolts);
            motorRight.setVoltage(IntakeK.levelOneVolts.unaryMinus());
        })
        .andThen(Commands.waitUntil(this::isSensorTripped))
        .andThen(this::stop);
    }

    /**
     * Sets voltage for motors scoring levels two and three
     * @return Command to set voltage and stop when ToF is tripped
     */
    public Command scoreLevelTwoThree() {
        return score(IntakeK.levelTwoThreeVolts);
    }

    /**
     * Sets voltage for motors scoring level four
     * @return Command to set voltage and stop when ToF is tripped
     */
    public Command scoreLevelFour() {
        return score(IntakeK.levelFourVolts);
    }
}
