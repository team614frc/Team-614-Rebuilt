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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.FloorConstants;

public class Floor extends SubsystemBase {
    public enum Speed {
        STOP(FloorConstants.STOP_PERCENT_OUTPUT),
        FEED(FloorConstants.FLOOR_PERCENT_OUTPUT);

    private final double percentOutput;

    private Speed(double percentOutput) {
      this.percentOutput = percentOutput;
    }

        public Voltage voltage() {
            return Volts.of(percentOutput * FloorConstants.MAX_VOLTAGE);
        }
    }

    private final TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(FloorConstants.VOLTAGE_OUT);

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
                    .withStatorCurrentLimit(FloorConstants.STATOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(FloorConstants.SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
            );

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
