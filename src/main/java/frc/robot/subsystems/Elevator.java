package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorK;
import frc.robot.lib.util.Encoder;
import frc.robot.lib.util.Util;

@Logged
public class Elevator extends SubsystemBase {

    private final TalonFX talon = new TalonFX(ElevatorK.talonID);
    private final TalonFX talonFollow = new TalonFX(ElevatorK.talonFollowID);
    private boolean zeroed = false;

    public Elevator() {
        configTalons();
        configMotionMagic(ElevatorK.maxVelocity, ElevatorK.maxAcceleration);
    }

    private void configTalons() {
        Util.factoryReset(talon, talonFollow);
        Util.brakeMode(talon, talonFollow);
        talonFollow.setControl(new StrictFollower(talon.getDeviceID()));
        talon.getConfigurator().apply(ElevatorK.softwareLimitConfig);
        talon.getConfigurator().apply(ElevatorK.hardwareLimitConfig);
        talon.getConfigurator().apply(ElevatorK.gearRatioConfig);
        talon.getConfigurator().apply(ElevatorK.pidConfig);
        // No limit switch
        talon.setPosition(Encoder.linearToAngular(ElevatorK.minHeight.div(ElevatorK.stageCount), ElevatorK.sprocketDiameter));
        zeroed = true;
    }

    private void configMotionMagic(LinearVelocity velocity, LinearAcceleration acceleration) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = (velocity.in(MetersPerSecond) / ElevatorK.stageCount) / (Math.PI * ElevatorK.sprocketDiameter.in(Meters));
        motionMagicConfigs.MotionMagicAcceleration = (acceleration.in(MetersPerSecondPerSecond) / ElevatorK.stageCount) / (Math.PI * ElevatorK.sprocketDiameter.in(Meters));
        talon.getConfigurator().apply(motionMagicConfigs);
    }

    /**
     * Sets the position of the elevator to a distance of height using the enum Positions within this classes constants file.
     * @param position position of the elavator to move to
     */
    public Command setPosition(ElevatorK.Positions position) {
        MotionMagicVoltage request = new MotionMagicVoltage(Encoder.linearToAngular(position.level.div(ElevatorK.stageCount), ElevatorK.sprocketDiameter));
        return Commands.either(
            runOnce(() -> talon.setControl(request))
                .andThen(Commands.waitUntil(() -> getPosition().isNear(position.level, ElevatorK.allowableError))),
            Commands.print("Elevator not zeroed"),
            () -> zeroed)
            .withName("Set Position " + position); 
    }

    /**
     * Gets the current height of the elevator
     * @return Height of the elevator as a Distance
     */
    public Distance getPosition() {
        return Encoder.angularToLinear(talon.getPosition().getValue().times(ElevatorK.stageCount), ElevatorK.sprocketDiameter);
    }

    @Logged(name = "Position (in.)")
    public double getPositionInches() {
        return getPosition().in(Inches);
    }

    /**
     * Sets the volts of the motors
     * @param volts speed of the motor in volts
     */
    public Command setVoltage(Voltage volts) {
        return runOnce(() -> talon.setControl(new VoltageOut(volts))).withName("Set Voltage");
    }

    /**
     * Stops both motors
     */
    public void stop() {
        talon.setControl(new NeutralOut());
    }

    public Command zero() {
        return setVoltage(ElevatorK.zeroingVoltage)
            .andThen(
                Commands.waitUntil(() -> talon.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround).finallyDo(this::stop),
                runOnce(() -> zeroed = true)
            )
            .finallyDo(() -> stop());
    }

    public Command zeroManual() {
        return runOnce(() -> talon.setPosition(Encoder.linearToAngular(ElevatorK.minHeight.div(ElevatorK.stageCount), ElevatorK.sprocketDiameter)));
    }

    @Logged(name = "Current Command")
    public String getCurrentCommandName() {
        var cmd = getCurrentCommand();
        if (cmd == null) return "None";
        return cmd.getName();
    }
    
}
