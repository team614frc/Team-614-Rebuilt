package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;
import java.util.List;
import org.littletonrobotics.junction.Logger;

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

  @Override
  public void periodic() {
    // Log individual motor data
    Logger.recordOutput("Shooter/Left/VelocityRPM", leftMotor.getVelocity().getValue().in(RPM));
    Logger.recordOutput(
        "Shooter/Left/StatorCurrentAmps", leftMotor.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Left/SupplyCurrentAmps", leftMotor.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Left/TemperatureCelsius", leftMotor.getDeviceTemp().getValue().in(Celsius));

    Logger.recordOutput("Shooter/Middle/VelocityRPM", middleMotor.getVelocity().getValue().in(RPM));
    Logger.recordOutput(
        "Shooter/Middle/StatorCurrentAmps", middleMotor.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Middle/SupplyCurrentAmps", middleMotor.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Middle/TemperatureCelsius", middleMotor.getDeviceTemp().getValue().in(Celsius));

    Logger.recordOutput("Shooter/Right/VelocityRPM", rightMotor.getVelocity().getValue().in(RPM));
    Logger.recordOutput(
        "Shooter/Right/StatorCurrentAmps", rightMotor.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Right/SupplyCurrentAmps", rightMotor.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Right/TemperatureCelsius", rightMotor.getDeviceTemp().getValue().in(Celsius));

    // Log aggregate data
    Logger.recordOutput("Shooter/TargetVelocityRPM", velocityRequest.Velocity * 60.0);
    Logger.recordOutput("Shooter/AtSetpoint", isVelocityWithinTolerance());
    Logger.recordOutput("Shooter/ExitVelocityMPS", getExitVelocity().in(Units.MetersPerSecond));

    // Log control state
    Logger.recordOutput("Shooter/CommandActive", getCurrentCommand() != null);
    if (getCurrentCommand() != null) {
      Logger.recordOutput("Shooter/CommandName", getCurrentCommand().getName());
    }
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
    Distance wheelRadius = Meters.of(0.05);
    AngularVelocity wheelAngularVelocity = leftMotor.getVelocity().getValue();
    LinearVelocity mps =
        MetersPerSecond.of(
            2.0 * Math.PI * wheelRadius.in(Meters) * wheelAngularVelocity.in(RotationsPerSecond));

    return mps;
  }
}
