package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbK;
import frc.robot.lib.util.Util;

@Logged
public class Climb extends SubsystemBase {

    private final TalonFX talon = new TalonFX(ClimbK.talonID);
    private final TalonFX talonFollow = new TalonFX(ClimbK.talonFollowID);
    private final Debouncer homingDebouncer = new Debouncer(ClimbK.homingTime.in(Seconds));
    private final Servo servoL = new Servo(ClimbK.servoLID);
    private final Servo servoR = new Servo(ClimbK.servoRID);
    private final BangBangController bangBang = new BangBangController(ClimbK.allowableError.in(Degree));
    private boolean isZeroed = false;

    public Climb() {
        configMotionMagic(ClimbK.maxVelocity, ClimbK.maxAcceleration); 
        configTalons();
        bangBang.setSetpoint(ClimbK.bangBangSetpoint.in(Degrees));
    } 

    private void configTalons() {
        Util.factoryReset(talon, talonFollow);
        talon.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        talon.getConfigurator().apply(ClimbK.softLimitConfigs);
        talon.getConfigurator().apply(ClimbK.gearRatioConfig);
        talon.getConfigurator().apply(ClimbK.pidconfig);
        talon.getConfigurator().apply(ClimbK.currentConfigs);
        talonFollow.getConfigurator().apply(ClimbK.currentConfigs);
        talonFollow.setControl(new StrictFollower(ClimbK.talonID));
        Util.brakeMode(talon, talonFollow);
    }

    private void configMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = velocity.in(RotationsPerSecond); 
        motionMagicConfigs.MotionMagicAcceleration = acceleration.in(RotationsPerSecondPerSecond); 
        talon.getConfigurator().apply(motionMagicConfigs);
    }

    public Command servoCoast() {
        return runOnce(() -> {
            servoL.set(0.25);
            servoR.set(0);
        }).andThen(Commands.waitSeconds(0.5)).withName("Servo Coast");
    }

    public Command servoRatchet() {
        return runOnce(() -> {
            servoL.set(0);
            servoR.set(0.25);
        }).andThen(Commands.waitSeconds(0.5)).withName("Servo Ratchet");
    }

    /**
     * Sets the voltage of the motors.
     * @param volts voltage of the motor in volts.
     */
    public Command setVoltage(Voltage volts) {
        VoltageOut request = new VoltageOut(volts);
        return runOnce(() -> talon.setControl(request)).withName("Set Voltage");
    }
    
    /**
     * Climbs the deep cage using voltage
     */
     public Command climb() {
        return servoRatchet()
            .andThen(
                setVoltage(ClimbK.climbVoltage),
                Commands.waitUntil(() -> talon.getFault_ReverseSoftLimit().getValue())
            ).finallyDo(this::stop)
            .withName("Deep Climb");
    }

    /**
     * Preps the arm for climbing
     */
    public Command prep() {
        return servoCoast()
            .andThen(
                setVoltage(ClimbK.prepVoltage),
                Commands.waitUntil(() -> talon.getFault_ForwardSoftLimit().getValue())
            )
            .finallyDo(this::stop)
            .withName("Prep Climb");
    }

    public double getPositionDegrees() {
        return talon.getPosition().getValue().in(Degrees);
    }

    public Command stow() {
        VoltageOut request = new VoltageOut(Volts.of(1));
        return servoCoast().andThen(run(() -> {
            System.out.println(getPositionDegrees());
            double v = bangBang.calculate(getPositionDegrees()) == 1 ? 1 : -1;
            System.out.println(v);
            talon.setControl(request.withOutput(v));
        })).until(() -> bangBang.atSetpoint())
        .finallyDo(this::stop).withName("Stow");
    }
    
    /**
     * Climbs the deep cage using motion magic
     */
    // public Command deepClimbMotionMagic() {
    //     servo.set(1);
    //     servo.setAngle(ClimbK.servoActiveAngle.in(Degree));
    //     MotionMagicVoltage request = new MotionMagicVoltage(ClimbK.maxAngle);
    //     return Commands.either(
    //         Commands.runOnce(() -> talon.setControl(request))
    //         .andThen(Commands.waitUntil(() -> talon.getPosition().getValue().isNear(ClimbK.maxAngle, ClimbK.allowableError))),
    //         Commands.print("Climb not zeroed!"),
    //         () -> isZeroed
    //     )
    //     .withName("Deep Climb Motion Magic");
        
    // }

    /** 
     * Runs the climber motors back to the hard stop and zeroes them
     */
    // public Command zero() {
    //     return setVoltage(ClimbK.zeroingVoltage)
    //     .andThen(Commands.waitUntil(() -> homingDebouncer.calculate(talon.getSupplyCurrent().getValue().gte(ClimbK.homingSpike))))
    //     .andThen(() -> { 
    //         talon.setPosition(ClimbK.minAngle);
    //         isZeroed = true;
    //     })
    //     .finallyDo(this::stop)
    //     .withName("Zero");
    // }

    /** Motion magic version of the Release command.
     */ 
    // public Command releaseMotionMagic() {
    //     MotionMagicVoltage request = new MotionMagicVoltage(ClimbK.minAngle);
    //     return runOnce(() -> armMotor.setControl(request))
    //     .andThen(Commands.waitUntil(() -> armMotor.getPosition().getValue().isNear(ClimbK.minAngle, ClimbK.allowableError)))
    //     .withName("Release Motion Magic");
    // }

    /**
     * Stops the climb motors
     */
    public void stop() {
        talon.setControl(new NeutralOut());
    }

    @Logged(name = "Current Command")
    public String getCurrentCommandName() {
        var cmd = getCurrentCommand();
        if (cmd == null) return "None";
        return cmd.getName();
    }

}
