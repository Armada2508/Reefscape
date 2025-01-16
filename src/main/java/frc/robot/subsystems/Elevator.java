package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorK;
import frc.robot.lib.util.Encoder;
import frc.robot.lib.util.Util;

public class Elevator extends SubsystemBase {
    TalonFX talon = new TalonFX(ElevatorK.elevatorID);

    public Elevator() {
        configMotionMagic(0, 0, 0); //! Find?
        configTalon();
    }

    public void configTalon() {
        Util.factoryReset(talon);
        // brake mode? invert?

    }

    public void configMotionMagic(double velocity, double acceleration, double jerk) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = acceleration / 360; //! Find
        motionMagicConfigs.MotionMagicCruiseVelocity = velocity / 360; //! Find
        motionMagicConfigs.MotionMagicJerk = jerk / 360; //! Find
        talon.getConfigurator().apply(motionMagicConfigs);
    }

    /**
     * Sets the position of the elevator to a distance of height using the enum Positions within this classes constants file.
     * @param position position of the elavator to move to
     */
    public Command setPosition(ElevatorK.Positions position) {
        // make this work depending on the enum above, switch case?
        MotionMagicVoltage request = new MotionMagicVoltage(Encoder.toRotations(position.level, ElevatorK.gearRatio, Inches.of(0))); //what wheel?
        return runOnce(() -> talon.setControl(request));
    }

    public Angle getPosition() {
        return Encoder.toRotations(null, ElevatorK.gearRatio, Inches.of(0)); //What wheel?
    }

    /**
     * 
     * @param speed speed of the motor in volts
     */
    public void setSpeed(Voltage speed) {
        VoltageOut request = new VoltageOut(speed);
        talon.setControl(request);
    }

    public void stop() {
        talon.setControl(new NeutralOut());
    }
}
