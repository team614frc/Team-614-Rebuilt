package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Feeder extends SubsystemBase {
  public static final AngularVelocity FEED_SPEED = RPM.of(5000);

  // Motor Behavior
  public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

  // Voltage Limits

  // Current Limits
  public static final Current STATOR_CURRENT_LIMIT = Amps.of(120);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);

  // Feedforward / voltage
  public static final Voltage MAX_VOLTAGE = Volts.of(12.0);

  // PID Constants
  public static final int PID_SLOT = 0;
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV =
      MAX_VOLTAGE.in(Volts) / KrakenX60.kFreeSpeed.in(RotationsPerSecond);

  public enum Speed {
    FEED(FEED_SPEED.in(Rotations.per(Minute)));

    private final double rpm;

    private Speed(double rpm) {
      this.rpm = rpm;
    }

    public AngularVelocity angularVelocity() {
      return RPM.of(rpm);
    }
  }

  private final TalonFX motor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public Feeder() {
    motor = new TalonFX(Ports.kFeeder, Ports.kRoboRioCANBus);

    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKV(kV));

    motor.getConfigurator().apply(config);
    SmartDashboard.putData(this);
  }

  public void set(Speed speed) {
    motor.setControl(velocityRequest.withVelocity(speed.angularVelocity()));
  }

  public void setPercentOutput(double percentOutput) {
    motor.setControl(voltageRequest.withOutput(Volts.of(percentOutput * MAX_VOLTAGE.in(Volts))));
  }

  public Command feedCommand() {
    return startEnd(() -> set(Speed.FEED), () -> setPercentOutput(0));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty(
        "Command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    builder.addDoubleProperty("RPM", () -> motor.getVelocity().getValue().in(RPM), null);
    builder.addDoubleProperty(
        "Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
    builder.addDoubleProperty(
        "Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
  }
}
