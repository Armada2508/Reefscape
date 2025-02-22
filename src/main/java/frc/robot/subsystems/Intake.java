package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;

import java.util.function.DoubleSupplier;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeK;
import frc.robot.lib.logging.LogUtil;
import frc.robot.lib.util.Util;

public class Intake extends SubsystemBase {
    
    private final SparkMax sparkMaxLeft = new SparkMax(IntakeK.sparkMaxLeftID, MotorType.kBrushless);
    private final SparkMax sparkMaxRight = new SparkMax(IntakeK.sparkMaxRightID, MotorType.kBrushless);
    @Logged(name = "Time of Flight")
    private final TimeOfFlight timeOfFlight = new TimeOfFlight(IntakeK.timeOfFlightId);

    public Intake() {
        configSparkMax();
        timeOfFlight.setRangingMode(RangingMode.Short, 24);
    }

    private void configSparkMax() {
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();

        leftConfig.idleMode(IdleMode.kCoast);
        rightConfig.idleMode(IdleMode.kCoast);
        leftConfig.smartCurrentLimit(IntakeK.currentLimit);
        rightConfig.smartCurrentLimit(IntakeK.currentLimit);
        leftConfig.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true).warningsAlwaysOn(true).faultsAlwaysOn(true);
        rightConfig.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true).warningsAlwaysOn(true).faultsAlwaysOn(true);
        rightConfig.inverted(true);

        sparkMaxLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        sparkMaxRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * True when ToF sensor is tripped
     * @return When milimeters are equal to the coralDetectionRange the sensor is tripped
     */
    @Logged(name = "Sensor Tripped")
    public boolean isSensorTripped() {
        return Util.inRange(timeOfFlight.getRange(), IntakeK.coralDetectionRange.in(Millimeters));
    }

    /**
     * Stops motors
     */
    public void stop() {
        sparkMaxLeft.stopMotor();
        sparkMaxRight.stopMotor();
    }

    /**
     * Sets voltage to motors
     * @param volts Voltage to give motors
     * @return Command to set voltage, motorRight runs opposite of motorLeft
     */
    public Command setVoltage(Voltage volts) {
        return runOnce (() -> {
            sparkMaxLeft.setVoltage(volts);
            sparkMaxRight.setVoltage(volts);
        })
        .withName("Set Voltage Command");
    }

    /**
     * Sets voltage to motors for scoring.
     * @param volts Voltage to give motors
     * @return Command to set voltage and stop when ToF is tripped
     */
    public Command score(Voltage volts) {
        return setVoltage(volts)
        .andThen(Commands.waitUntil(() -> !isSensorTripped()))
        .finallyDo(this::stop)
        .withName("Score Command");
    }

    /**
     * Sets voltage to motors for intake.
     * @return Command to set voltage and stop when time of flight is tripped
     */
    public Command coralIntake() {
        return setVoltage(IntakeK.coralIntakeVolts)
        .andThen(
            Commands.waitUntil(this::isSensorTripped),
            Commands.waitSeconds(0.25)
        )
        .finallyDo(this::stop)
        .withName("Coral Intake Command");
    }

    DoubleSupplier voltage1 = LogUtil.getTunableDouble("First Voltage (V)", -4);
    DoubleSupplier voltage2 = LogUtil.getTunableDouble("Second Voltage (V)", 2);
    DoubleSupplier voltage3 = LogUtil.getTunableDouble("Third Voltage (V)", -6);
    DoubleSupplier voltage4 = LogUtil.getTunableDouble("Fourth Voltage (V)", -4);
    DoubleSupplier wait1 = LogUtil.getTunableDouble("First Wait (s)", 0.04);
    DoubleSupplier wait2 = LogUtil.getTunableDouble("Second Wait (s)", 0.1);
    DoubleSupplier wait3 = LogUtil.getTunableDouble("Third Wait (s)", 1);

    /**
     * Sets voltage for motors scoring level one
     * @return Command to set voltage with both motors in same direction and stop when ToF is tripped
     */
    public Command scoreLevelOne() {
        return runOnce(() -> {
            sparkMaxLeft.setVoltage(-7.5);
            sparkMaxRight.setVoltage(-7.5);
        })
        .andThen(
            Commands.waitSeconds(0.04),
            runOnce(() -> sparkMaxRight.setVoltage(5.5))
            // Commands.waitSeconds(0)
        )
        .finallyDo(this::stop)
        .withName("Score Level One Command");
    }

    /**
     * Sets voltage for motors scoring levels two and three
     * @return Command to set voltage and stop when ToF is tripped
     */
    public Command scoreLevelTwoThree() {
        return score(IntakeK.levelTwoThreeVolts)
        .withName("Score Levels Two and Three Command");
    }

    /**
     * Sets voltage for motors scoring level four
     * @return Command to set voltage and stop when ToF is tripped
     */
    public Command scoreLevelFour() {
        return score(IntakeK.levelFourVolts)
        .withName("Score Level Four Command");
    }

    @Logged(name = "TOF Range Valid")
    public boolean isTOFRangeValid() {
        return timeOfFlight.isRangeValid();
    }
    
    @Logged(name = "Current Command")
    public String getCurrentCommandName() {
        var cmd = getCurrentCommand();
        if (cmd == null) return "None";
        return cmd.getName();
    }
    
}
