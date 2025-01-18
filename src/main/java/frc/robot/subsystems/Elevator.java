package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

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

public class Elevator extends SubsystemBase {
    private final TalonFX talon = new TalonFX(ElevatorK.elevatorID);
    private final TalonFX talonFollow = new TalonFX(ElevatorK.followID);

    public Elevator() {
        configMotionMagic(ElevatorK.velocity, ElevatorK.acceleration);
        configTalon();
    }

    private void configTalon() {
        Util.factoryReset(talon);
        Util.brakeMode(talon, talonFollow);
        talonFollow.setControl(new StrictFollower(talon.getDeviceID()));
        talon.getConfigurator().apply(ElevatorK.gearRatioConfig);
        talon.getConfigurator().apply(ElevatorK.pidConfig); // applies PID constants, still need to tunee
    }

    private void configMotionMagic(LinearVelocity velocity, LinearAcceleration acceleration) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = (velocity.in(MetersPerSecond) / ElevatorK.stageCount) / (Math.PI * ElevatorK.sprocketDiameter.in(Meters));
        motionMagicConfigs.MotionMagicAcceleration = (acceleration.in(MetersPerSecondPerSecond) / ElevatorK.stageCount) / (Math.PI * ElevatorK.sprocketDiameter.in(Meters)); //! Find
        talon.getConfigurator().apply(motionMagicConfigs);
    }
    // find out if motion magic is dependent on the gear box or the motor

    /**
     * Sets the position of the elevator to a distance of height using the enum Positions within this classes constants file.
     * @param position position of the elavator to move to
     */
    public Command setPosition(ElevatorK.Positions position) {
        MotionMagicVoltage request = new MotionMagicVoltage(Encoder.toRotations(position.level, 1, ElevatorK.sprocketDiameter)); //what wheel?
        return runOnce(() -> talon.setControl(request))
        .andThen(Commands.waitUntil(() -> getPosition().isNear(position.level, ElevatorK.allowableError))); // stop when we reach set position
    }

    /**
     * Gets the current height of the elevator
     * @return Height of the elevator as a Distance
     */
    public Distance getPosition() {
        return Encoder.toDistance(talon.getPosition().getValue(), 1, ElevatorK.sprocketDiameter);
    }

    /**
     * Sets the volts of the motors
     * @param speed speed of the motor in volts
     */
    public void setVoltage(Voltage volts) {
        VoltageOut request = new VoltageOut(volts);
        talon.setControl(request);
    }

    /**
     * Stops both motors
     */
    public void stop() {
        talon.setControl(new NeutralOut());
    }
}
