package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
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
import org.littletonrobotics.junction.Logger;

public class Floor extends SubsystemBase {
  private static final Voltage MAX_VOLTAGE = Volts.of(12.0);
  private static final Current STATOR_CURRENT_LIMIT = Amps.of(120);
  private static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);

  private static final double FEED_RPM = 5900;

  private static final double KP = 0.5;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double KV =
      MAX_VOLTAGE.in(Volts) / KrakenX60.kFreeSpeed.in(RotationsPerSecond);

  private static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(100);

  private final TalonFX motor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public Floor() {
    motor = new TalonFX(Ports.kFloor, Ports.kRoboRioCANBus);

    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withVoltage(new VoltageConfigs().withPeakReverseVoltage(Volts.of(0)))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs().withKP(KP).withKI(KI).withKD(KD).withKV(KV));

    motor.getConfigurator().apply(config);
    SmartDashboard.putData(this);
  }

  public void feed() {
    motor.setControl(velocityRequest.withVelocity(RPM.of(FEED_RPM)));
  }

  public void stop() {
    motor.setControl(voltageRequest.withOutput(0));
  }

  public boolean isAtSetpoint() {
    final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
    final AngularVelocity currentVelocity = motor.getVelocity().getValue();
    final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
    return isInVelocityMode && currentVelocity.isNear(targetVelocity, VELOCITY_TOLERANCE);
  }

  public Command feedCommand() {
    return startEnd(this::feed, this::stop);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Floor/VelocityRPM", motor.getVelocity().getValue().in(RPM));
    Logger.recordOutput("Floor/TargetRPM", velocityRequest.getVelocityMeasure().in(RPM));
    Logger.recordOutput("Floor/AtSetpoint", isAtSetpoint());
    Logger.recordOutput("Floor/StatorCurrentAmps", motor.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput("Floor/SupplyCurrentAmps", motor.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput("Floor/AppliedVoltage", motor.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput("Floor/Temperature", motor.getDeviceTemp().getValue().in(Celsius));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty(
        "Command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    builder.addDoubleProperty("RPM", () -> motor.getVelocity().getValue().in(RPM), null);
    builder.addDoubleProperty(
        "Target RPM", () -> velocityRequest.getVelocityMeasure().in(RPM), null);
    builder.addBooleanProperty("At Setpoint", this::isAtSetpoint, null);
    builder.addDoubleProperty(
        "Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
    builder.addDoubleProperty(
        "Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
  }
}
