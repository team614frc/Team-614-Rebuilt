package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
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
import frc.robot.Ports;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private static final Current STATOR_CURRENT_LIMIT = Amps.of(120);
  private static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
  private static final Voltage MAX_VOLTAGE = Volts.of(12.0);

  // Default PIDF gains — adjustable live via SmartDashboard
  private static final double DEFAULT_KP = 0.5;
  private static final double DEFAULT_KI = 0.0;
  private static final double DEFAULT_KD = 0.0;
  private static final double DEFAULT_KV = 0.1195;

  public enum Speed {
    FEED(RPM.of(6500)); // 5500

    private final AngularVelocity velocity;

    private Speed(AngularVelocity velocity) {
      this.velocity = velocity;
    }

    public AngularVelocity angularVelocity() {
      return this.velocity;
    }
  }

  private final TalonFX motor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final NeutralOut neutralRequest = new NeutralOut();

  // Live-tunable PIDF gains
  private double kP = DEFAULT_KP;
  private double kI = DEFAULT_KI;
  private double kD = DEFAULT_KD;
  private double kV = DEFAULT_KV;

  // Track last-applied gains to avoid spamming CAN bus
  private double[] lastApplied = {-1, -1, -1, -1}; // [kP, kI, kD, kV]

  // SmartDashboard test toggle
  private boolean testEnabled = false;
  private double dashboardTargetRPM = Speed.FEED.angularVelocity().in(RPM);

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
            .withSlot0(
                new Slot0Configs()
                    .withKP(DEFAULT_KP)
                    .withKI(DEFAULT_KI)
                    .withKD(DEFAULT_KD)
                    .withKV(DEFAULT_KV));

    motor.getConfigurator().apply(config);
    SmartDashboard.putData(this);
  }

  public void set(Speed speed) {
    motor.setControl(velocityRequest.withVelocity(speed.angularVelocity()));
  }

  public void setPercentOutput(double percentOutput) {
    motor.setControl(voltageRequest.withOutput(MAX_VOLTAGE.times(percentOutput)));
  }

  public void feed() {
    set(Speed.FEED);
  }

  public void stop() {
    motor.setControl(neutralRequest);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command feedCommand() {
    return startEnd(this::feed, this::stop);
  }

  /** Applies updated Slot0 gains only when values have changed. */
  private void applyGainsIfChanged() {
    if (kP != lastApplied[0]
        || kI != lastApplied[1]
        || kD != lastApplied[2]
        || kV != lastApplied[3]) {
      motor.getConfigurator().apply(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKV(kV));
      lastApplied[0] = kP;
      lastApplied[1] = kI;
      lastApplied[2] = kD;
      lastApplied[3] = kV;
    }
  }

  @Override
  public void periodic() {
    // Apply any gain changes from SmartDashboard
    applyGainsIfChanged();

    // Test mode: run at dashboard RPM when no command is scheduled
    if (getCurrentCommand() == null) {
      if (testEnabled) {
        motor.setControl(velocityRequest.withVelocity(RPM.of(dashboardTargetRPM)));
      } else {
        motor.setControl(neutralRequest);
      }
    }

    Logger.recordOutput("Feeder/VelocityRPM", motor.getVelocity().getValue().in(RPM));
    Logger.recordOutput("Feeder/TargetVelocityRPM", velocityRequest.Velocity * 60.0);
    Logger.recordOutput("Feeder/StatorCurrentAmps", motor.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput("Feeder/SupplyCurrentAmps", motor.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput("Feeder/AppliedVoltage", motor.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput("Feeder/TemperatureCelsius", motor.getDeviceTemp().getValue().in(Celsius));
    Logger.recordOutput("Feeder/TestEnabled", testEnabled);
    Logger.recordOutput("Feeder/CommandActive", getCurrentCommand() != null);
    if (getCurrentCommand() != null) {
      Logger.recordOutput("Feeder/CommandName", getCurrentCommand().getName());
    }
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

    // Test controls
    builder.addBooleanProperty("Test Enable", () -> testEnabled, v -> testEnabled = v);
    builder.addDoubleProperty(
        "Test Target RPM", () -> dashboardTargetRPM, v -> dashboardTargetRPM = v);

    // Live PIDF tuning
    builder.addDoubleProperty("kP", () -> kP, v -> kP = v);
    builder.addDoubleProperty("kI", () -> kI, v -> kI = v);
    builder.addDoubleProperty("kD", () -> kD, v -> kD = v);
    builder.addDoubleProperty("kV", () -> kV, v -> kV = v);
  }
}
