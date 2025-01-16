package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorK;
import frc.robot.lib.util.Encoder;
import frc.robot.lib.util.Util;

public class Elevator extends SubsystemBase {
    TalonFX talon = new TalonFX(ElevatorK.elevatorID);

    public Elevator() {
        configMotionmagic();
        configTalon();
    }

    public void configTalon() {
        Util.factoryReset(talon);
        // brake mode? invert?

    }

    public void configMotionmagic() {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 0; 
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicJerk = 0;
        talon.getConfigurator().apply(motionMagicConfigs);
    }

    enum Positions {
        L1(Inches.of(18)), //! verify 
        L2(Inches.of(31.825)),
        L3(Inches.of(47.625)),
        L4(Inches.of(72)),
        ALGAE_LOW(L2.level.minus(Inches.of(6.25))),
        ALGAE_HIGH(L3.level.minus(Inches.of(6.25))),
        STOW(Inches.of(0)); //! find

        public final Distance level;

        Positions(Distance position) {
            this.level = position;
        }
    }

    /**
     * 
     * @param position position of the elavator to move to
     */
    public void setPosition(Positions position) {
        // make this work depending on the enum above, switch case?
        MotionMagicVoltage request = new MotionMagicVoltage(Encoder.toRotations(position.level, ElevatorK.gearRatio, Inches.of(0))); //what wheel?
        talon.setControl(request);
    }

    public void getPosition() {
        
    }

    /**
     * 
     * @param speed speed of the motor in volts
     */
    public void setSpeed(Voltage speed) {
        VoltageOut request = new VoltageOut(speed);
        talon.setControl(request);
    }
}
