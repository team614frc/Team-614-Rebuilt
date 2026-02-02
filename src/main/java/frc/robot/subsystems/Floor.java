package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Floor extends SubsystemBase {

  // Speed Constants
  public static final double STOP_PERCENT_OUTPUT = 0.0;
  public static final double FLOOR_PERCENT_OUTPUT = 0.83;

  // Voltage Limits
  public static final Voltage MAX_VOLTAGE = Volts.of(12.0);
  public static final Voltage VOLTAGE_OUT = Volts.of(0.0);

  // Current Limits
  public static final Current STATOR_CURRENT_LIMIT = Amps.of(120);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);

  public enum Speed {
    STOP(STOP_PERCENT_OUTPUT),
    FEED(FLOOR_PERCENT_OUTPUT);

    private final double percentOutput;

    private Speed(double percentOutput) {
      this.percentOutput = percentOutput;
    }

    public Voltage voltage() {
      return Volts.of(percentOutput * MAX_VOLTAGE.in(Volts));
    }
  }

  private final TalonFX motor;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public Floor() {
    motor = new TalonFX(Ports.kFloor, Ports.kRoboRioCANBus);

    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true));

    motor.getConfigurator().apply(config);
    SmartDashboard.putData(this);
  }

  public void set(Speed speed) {
    motor.setControl(voltageRequest.withOutput(speed.voltage()));
  }

  public Command feedCommand() {
    return startEnd(() -> set(Speed.FEED), () -> set(Speed.STOP));
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
