package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorK;
import frc.robot.Constants.ElevatorK.Positions;
import frc.robot.lib.util.Encoder;
import frc.robot.lib.util.Util;

@Logged
public class Elevator extends SubsystemBase {

    private final TalonFX talon = new TalonFX(ElevatorK.talonID);
    private final TalonFX talonFollow = new TalonFX(ElevatorK.talonFollowID);
    private final TimeOfFlight timeOfFlight = new TimeOfFlight(ElevatorK.tofID);
    private final Debouncer debouncer = new Debouncer(ElevatorK.currentTripTime.in(Seconds));
    private double lastSensorRead = 0;

    public Elevator() {
        configTalons();
        configMotionMagic(ElevatorK.maxVelocity, ElevatorK.maxAcceleration);
        timeOfFlight.setRangingMode(RangingMode.Short, ElevatorK.sampleTime);
    }

    @Override
    public void periodic() {
        if (debouncer.calculate(talon.getTorqueCurrent().getValue().abs(Amps) > ElevatorK.currentSpike.in(Amps))) {
            Command current = getCurrentCommand();
            if (current != null) current.cancel();
            stop();
            System.out.println("Stopping ELEVATOR because of CURRENT SPIKE!");
        }
    }

    private void configTalons() {
        Util.factoryReset(talon, talonFollow);
        Util.brakeMode(talon, talonFollow);
        talonFollow.setControl(new StrictFollower(talon.getDeviceID()));
        talon.getConfigurator().apply(ElevatorK.currentLimitsConfig);
        talonFollow.getConfigurator().apply(ElevatorK.currentLimitsConfig);
        talon.getConfigurator().apply(ElevatorK.softwareLimitConfig);
        talon.getConfigurator().apply(ElevatorK.gearRatioConfig);
        talon.getConfigurator().apply(ElevatorK.pidConfig);
        // No limit switch
        talon.setPosition(linearToAngular(ElevatorK.minHeight));
    }

    private void configMotionMagic(LinearVelocity velocity, LinearAcceleration acceleration) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = linearToAngular(velocity.times(BaseUnits.TimeUnit.one())).in(Rotations);
        motionMagicConfigs.MotionMagicAcceleration = linearToAngular(
            acceleration.times(BaseUnits.TimeUnit.one()).times(BaseUnits.TimeUnit.one())
        ).in(Rotations);
        talon.getConfigurator().apply(motionMagicConfigs);
    }

    private Optional<Distance> getInterpolatedHeight(Distance closeHeight, Distance farHeight) {
        if (!timeOfFlight.isRangeValid()) return Optional.empty();
        double range = timeOfFlight.getRange();
        if (Millimeters.of(Math.abs(range - lastSensorRead)).gte(Inches.of(0.22))) lastSensorRead = range;
        Distance interpolatedHeight = Millimeters.of(MathUtil.interpolate(
            closeHeight.in(Millimeters), 
            farHeight.in(Millimeters), 
            (lastSensorRead + ElevatorK.timeOfFlightOffset.in(Millimeters)) / ElevatorK.maxLinearDistance.in(Millimeters)
        ));
        return Optional.of(interpolatedHeight);
    }

    private void setPosition(Distance height) {
        MotionMagicVoltage request = new MotionMagicVoltage(linearToAngular(height));
        talon.setControl(request);
    }

    /**
     * Sets the position of the elevator to a specific height and end's when it's within the allowable error or after 2 seconds.
     * @param height height of the elevator to move to
     */
    public Command setPositionCommand(Distance height) {
        return runOnce(() -> setPosition(height))
            .andThen(Commands.waitUntil(() -> getPosition().isNear(height, ElevatorK.allowableError)))
            .withTimeout(2)
            .withName("Set Position " + height); 
    }

    /**
     * Sets the position of the elevator to a distance of height using the enum Positions within this class's constants file.
     * @param position position of the elevator to move to
     */
    public Command setPositionCommand(ElevatorK.Positions position) {
        return switch (position) {
            case INTAKE -> // Dynamic
                runOnce(() -> setPosition(position.close))
                .andThen(run(() -> getInterpolatedHeight(position.close, position.far).ifPresent(this::setPosition)))
                .withName("Set Dynamic Position " + position);
            case STOW, ALGAE_LOW, ALGAE_HIGH, L1, L2, L3, L4 -> // Static
                setPositionCommand(position.close)
                .withName("Set Static Position " + position);
            default -> throw new IllegalArgumentException("Invalid Position: " + position);
        };
    }

    /**
     * Sets the volts of the motors
     * @param volts speed of the motor in volts
     */
    public Command setVoltage(Voltage volts) {
        VoltageOut request = new VoltageOut(volts);
        return runOnce(() -> talon.setControl(request)).withName("Set Voltage");
    }

    /**
     * Stops both motors
     */
    public void stop() {
        talon.setControl(new NeutralOut());
    }

    /**
     * Gets the current height of the elevator
     * @return Height of the elevator as a Distance
     */
    public Distance getPosition() {
        return angularToLinear(talon.getPosition().getValue());
    }

    public boolean nearHeight(Distance height) {
        return height.isNear(getPosition(), ElevatorK.allowableError);
    }

    public boolean nearL4() {
        return nearHeight(Positions.L4.close);
    }

    @Logged(name = "TOF Reading (in.)")
    public double getTimeOfFlightDistance() {
        return timeOfFlight.getRange() / 25.4;
    }

    public boolean isTOFValid() {
        return timeOfFlight.isRangeValid();
    }

    @Logged(name = "Position (in.)")
    public double getPositionInches() {
        return getPosition().in(Inches);
    }

    @Logged(name = "Velocity (in.)")
    public double getVelocityInches() {
        return angularToLinear(Rotations.of(talon.getVelocity().getValueAsDouble())).in(Inches);
    }

    @Logged(name = "Acceleration (in.)")
    public double getAccelerationInches() {
        return angularToLinear(Rotations.of(talon.getAcceleration().getValueAsDouble())).in(Inches);
    }

    @Logged(name = "Target (in.)")
    public double getTargetInches() {
        return angularToLinear(Rotations.of(talon.getClosedLoopReference().getValue())).in(Inches);
    }

    @Logged(name = "Error (in.)")
    public double getErrorInches() {
        return angularToLinear(Rotations.of(talon.getClosedLoopError().getValue())).in(Inches);
    }

    @Logged(name = "Current Command")
    public String getCurrentCommandName() {
        var cmd = getCurrentCommand();
        if (cmd == null) return "None";
        return cmd.getName();
    }

    private Angle linearToAngular(Distance height) {
        return Encoder.linearToAngular(height.div(ElevatorK.stageCount), ElevatorK.sprocketDiameter);
    }

    private Distance angularToLinear(Angle rotations) {
        return Encoder.angularToLinear(rotations.times(ElevatorK.stageCount), ElevatorK.sprocketDiameter);
    }

}
