package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Hanger extends SubsystemBase {

  // Position Constants
  public static final Distance HANGER_HOMED = Inches.of(0);
  public static final Distance HANGER_EXTEND_HOPPER = Inches.of(2);
  public static final Distance HANGER_HANGING = Inches.of(6);
  public static final Distance HANGER_HUNG = Inches.of(0.2);

  // Voltage Limits
  public static final int INITIAL_SETPOINT = 0;
  public static final int NEW_SLOT = 0;

  public static final Voltage VOLTAGE_OUT = Volts.of(0.0);
  public static final Voltage MAX_VOLTAGE = Volts.of(12.0);

  // Current Limits
  public static final Current STATOR_CURRENT_LIMIT = Amps.of(20);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(70);

  // PID Constants
  public static final double kP = 10.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 12;

  // Homing Constants
  public static final double HOMING_PERCENT_OUTPUT = -0.05 * 12.0;
  public static final Current HOMING_CURRENT_THRESHOLD = Amps.of(0.4);

  public static Per<DistanceUnit, AngleUnit> HANGER_EXTENSION_PER_MOTOR_ANGLE =
      Inches.of(6).div(Rotations.of(142));
  public static Distance EXTENSION_TOLERANCE = Inches.of(1);

  public enum Position {
    HOMED(HANGER_HOMED),
    EXTEND_HOPPER(HANGER_EXTEND_HOPPER),
    HANGING(HANGER_HANGING),
    HUNG(HANGER_HUNG);

    private final Distance distance;

    private Position(Distance inches) {
      this.distance = inches;
    }

    public Angle motorAngle() {
      final Measure<AngleUnit> angleMeasure =
         distance.divideRatio(HANGER_EXTENSION_PER_MOTOR_ANGLE);
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
                    .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(KrakenX60.kFreeSpeed)
                    .withMotionMagicAcceleration(KrakenX60.kFreeSpeed.per(Second)))
            .withSlot0(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKV(kV));

    motor.getConfigurator().apply(config);
    SmartDashboard.putData(this);
  }

  public void set(Position position) {
    motor.setControl(motionMagicRequest.withPosition(position.motorAngle()));
  }

  public void setPercentOutput(double percentOutput) {
    motor.setControl(voltageRequest.withOutput(Volts.of(percentOutput * MAX_VOLTAGE.in(Volts))));
  }

  public Command positionCommand(Position position) {
    return runOnce(() -> set(position))
        .andThen(Commands.waitUntil(this::isExtensionWithinTolerance));
  }

  public Command homingCommand() {
    return Commands.sequence(
            runOnce(() -> setPercentOutput(HOMING_PERCENT_OUTPUT)),
            Commands.waitUntil(
                () ->
                    motor.getSupplyCurrent().getValue().in(Amps)
                        > HOMING_CURRENT_THRESHOLD.in(Amps)),
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
    return currentExtension.isNear(targetExtension, EXTENSION_TOLERANCE);
  }

  private Distance motorAngleToExtension(Angle motorAngle) {
    final Measure<DistanceUnit> extensionMeasure =
        motorAngle.timesRatio(HANGER_EXTENSION_PER_MOTOR_ANGLE);
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
