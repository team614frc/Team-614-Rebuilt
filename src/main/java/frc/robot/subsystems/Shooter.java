package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;
import java.util.List;

public class Shooter extends SubsystemBase {

  private static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(100);
  private static final Voltage MAX_VOLTAGE = Volts.of(12.0);
  private static final Current STATOR_CURRENT_LIMIT = Amps.of(120);
  private static final Current SUPPLY_CURRENT_LIMIT = Amps.of(70);
  private static final double kP = 0.5;
  private static final double kI = 2.0;
  private static final double kD = 0.0;
  private static final double kV =
      MAX_VOLTAGE.in(Volts) / KrakenX60.kFreeSpeed.in(RotationsPerSecond);
  private final TalonFX leftMotor, middleMotor, rightMotor;
  private final List<TalonFX> motors;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private double dashboardTargetRPM = 0.0;

  public Shooter() {
    leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kRoboRioCANBus);
    middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kRoboRioCANBus);
    rightMotor = new TalonFX(Ports.kShooterRight, Ports.kRoboRioCANBus);
    motors = List.of(leftMotor, middleMotor, rightMotor);

    configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive);
    configureMotor(middleMotor, InvertedValue.Clockwise_Positive);
    configureMotor(rightMotor, InvertedValue.Clockwise_Positive);

    SmartDashboard.putData(this);
  }

  private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withVoltage(new VoltageConfigs().withPeakReverseVoltage(Volts.of(0)))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKV(kV));

    motor.getConfigurator().apply(config);
  }

  public void setRPM(double rpm) {
    for (final TalonFX motor : motors) {
      motor.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
    }
  }

  public void setPercentOutput(double percentOutput) {
    for (final TalonFX motor : motors) {
      motor.setControl(voltageRequest.withOutput(MAX_VOLTAGE.times(percentOutput)));
    }
  }

  public void stop() {
    setPercentOutput(0.0);
  }

  public Command spinUpCommand(double rpm) {
    return runOnce(() -> setRPM(rpm)).andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
  }

  public Command dashboardSpinUpCommand() {
    return defer(() -> spinUpCommand(dashboardTargetRPM));
  }

  public boolean isVelocityWithinTolerance() {
    return motors.stream()
        .allMatch(
            motor -> {
              final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
              final AngularVelocity currentVelocity = motor.getVelocity().getValue();
              final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
              return isInVelocityMode && currentVelocity.isNear(targetVelocity, VELOCITY_TOLERANCE);
            });
  }

  private void initSendable(SendableBuilder builder, TalonFX motor, String name) {
    builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
    builder.addDoubleProperty(
        name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
    builder.addDoubleProperty(
        name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    initSendable(builder, leftMotor, "Left");
    initSendable(builder, middleMotor, "Middle");
    initSendable(builder, rightMotor, "Right");
    builder.addStringProperty(
        "Command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    builder.addDoubleProperty(
        "Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
    builder.addDoubleProperty(
        "Target RPM", () -> velocityRequest.getVelocityMeasure().in(RPM), null);
  }

  public LinearVelocity getExitVelocity() {
    double wheelRadiusMeters = 0.05;
    double wheelRPS = leftMotor.getVelocity().getValue().in(RotationsPerSecond);
    double mps = 2.0 * Math.PI * wheelRadiusMeters * wheelRPS;

    // NEW: use constructor with value and Units constant
    return Units.MetersPerSecond.of(mps);
  }
}
