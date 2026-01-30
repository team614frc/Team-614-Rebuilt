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
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;
import frc.robot.Constants.HangerConstants;

public class Hanger extends SubsystemBase {
    public enum Position {
        HOMED(HangerConstants.HOMED),
        EXTEND_HOPPER(HangerConstants.EXTEND_HOPPER),
        HANGING(HangerConstants.HANGING),
        HUNG(HangerConstants.HUNG);

    private final double inches;

    private Position(double inches) {
      this.inches = inches;
    }

    public Angle motorAngle() {
      final Measure<AngleUnit> angleMeasure =
          Inches.of(inches).divideRatio(kHangerExtensionPerMotorAngle);
      return Rotations.of(angleMeasure.in(Rotations)); // Promote from Measure<AngleUnit> to Angle
    }
  }

    private static final Per<DistanceUnit, AngleUnit> kHangerExtensionPerMotorAngle = Inches.of(HangerConstants.MOTOR_ANGLE_DISTANCE).div(Rotations.of(HangerConstants.ROTATIONS));
    private static final Distance kExtensionTolerance = Inches.of(HangerConstants.EXTENSION_TOLERANCE);

    private final TalonFX motor;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(HangerConstants.INITIAL_SETPOINT).withSlot(HangerConstants.NEW_SLOT);
    private final VoltageOut voltageRequest = new VoltageOut(HangerConstants.VOLTAGE_OUT);

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
                    .withSupplyCurrentLimitEnable(true)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(KrakenX60.kFreeSpeed)
                    .withMotionMagicAcceleration(KrakenX60.kFreeSpeed.per(Second)))
            .withSlot0(
                new Slot0Configs()
                    .withKP(HangerConstants.kP)
                    .withKI(HangerConstants.kI)
                    .withKD(HangerConstants.kD)
                    .withKV(HangerConstants.MAX_VOLTAGE / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );

    motor.getConfigurator().apply(config);
    SmartDashboard.putData(this);
  }

  public void set(Position position) {
    motor.setControl(motionMagicRequest.withPosition(position.motorAngle()));
  }

    public void setPercentOutput(double percentOutput) {
        motor.setControl(
            voltageRequest
                .withOutput(Volts.of(percentOutput * HangerConstants.MAX_VOLTAGE))
        );
    }

  public Command positionCommand(Position position) {
    return runOnce(() -> set(position))
        .andThen(Commands.waitUntil(this::isExtensionWithinTolerance));
  }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPercentOutput(HangerConstants.HOMING_VOLTAGE)),
            Commands.waitUntil(() -> motor.getSupplyCurrent().getValue().in(Amps) > HangerConstants.HOMING_CURRENT_THRESHOLD),
            runOnce(() -> {
                motor.setPosition(Position.HOMED.motorAngle());
                isHomed = true;
                set(Position.EXTEND_HOPPER);
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public boolean isHomed() {
    return isHomed;
  }

  private boolean isExtensionWithinTolerance() {
    final Distance currentExtension = motorAngleToExtension(motor.getPosition().getValue());
    final Distance targetExtension = motorAngleToExtension(motionMagicRequest.getPositionMeasure());
    return currentExtension.isNear(targetExtension, kExtensionTolerance);
  }

  private Distance motorAngleToExtension(Angle motorAngle) {
    final Measure<DistanceUnit> extensionMeasure =
        motorAngle.timesRatio(kHangerExtensionPerMotorAngle);
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
