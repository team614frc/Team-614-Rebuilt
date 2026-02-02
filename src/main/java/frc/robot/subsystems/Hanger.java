package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangerConstants;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Hanger extends SubsystemBase {
  public enum Position {
    HOMED(HangerConstants.HOMED.in(Inches)),
    EXTEND_HOPPER(HangerConstants.EXTEND_HOPPER.in(Inches)),
    HANGING(HangerConstants.HANGING.in(Inches)),
    HUNG(HangerConstants.HUNG.in(Inches));

    private final double inches;

    private Position(double inches) {
      this.inches = inches;
    }

    public Angle motorAngle() {
      final Measure<AngleUnit> angleMeasure =
          Inches.of(inches).divideRatio(HangerConstants.HANGER_EXTENSION_PER_MOTOR_ANGLE);
      return Rotations.of(angleMeasure.in(Rotations)); // Promote from Measure<AngleUnit> to Angle
    }
  }

  private final TalonFX motor;
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private boolean isHomed = false;

  public Hanger() {
    motor = new TalonFX(Ports.kHanger, Ports.kRoboRioCANBus);

    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(HangerConstants.STATOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(HangerConstants.SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(KrakenX60.kFreeSpeed)
                    .withMotionMagicAcceleration(KrakenX60.kFreeSpeed.per(Second)))
            .withSlot0(
                new Slot0Configs()
                    .withKP(HangerConstants.kP)
                    .withKI(HangerConstants.kI)
                    .withKD(HangerConstants.kD)
                    .withKV(
                        HangerConstants.MAX_VOLTAGE.in(Volts)
                            / KrakenX60.kFreeSpeed.in(
                                RotationsPerSecond)) // 12 volts when requesting max RPS
                );

    motor.getConfigurator().apply(config);
    SmartDashboard.putData(this);
  }

  public void set(Position position) {
    motor.setControl(motionMagicRequest.withPosition(position.motorAngle()));
  }

  public void setPercentOutput(double percentOutput) {
    motor.setControl(
        voltageRequest.withOutput(Volts.of(percentOutput * HangerConstants.MAX_VOLTAGE.in(Volts))));
  }

  public Command positionCommand(Position position) {
    return runOnce(() -> set(position))
        .andThen(Commands.waitUntil(this::isExtensionWithinTolerance));
  }

  public Command homingCommand() {
    return Commands.sequence(
            runOnce(() -> setPercentOutput(HangerConstants.HOMING_VOLTAGE)),
            Commands.waitUntil(
                () ->
                    motor.getSupplyCurrent().getValue().in(Amps)
                        > HangerConstants.HOMING_CURRENT_THRESHOLD.in(Amps)),
            runOnce(
                () -> {
                  motor.setPosition(Position.HOMED.motorAngle());
                  isHomed = true;
                  set(Position.EXTEND_HOPPER);
                }))
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public boolean isHomed() {
    return isHomed;
  }

  private boolean isExtensionWithinTolerance() {
    final Distance currentExtension = motorAngleToExtension(motor.getPosition().getValue());
    final Distance targetExtension = motorAngleToExtension(motionMagicRequest.getPositionMeasure());
    return currentExtension.isNear(targetExtension, HangerConstants.EXTENSION_TOLERANCE);
  }

  private Distance motorAngleToExtension(Angle motorAngle) {
    final Measure<DistanceUnit> extensionMeasure =
        motorAngle.timesRatio(HangerConstants.HANGER_EXTENSION_PER_MOTOR_ANGLE);
    return Inches.of(extensionMeasure.in(Inches)); // Promote from Measure<DistanceUnit> to Distance
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty(
        "Command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    builder.addDoubleProperty(
        "Extension (inches)",
        () -> motorAngleToExtension(motor.getPosition().getValue()).in(Inches),
        null);
    builder.addDoubleProperty(
        "Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
  }
}
