package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Intake extends SubsystemBase {

  private static final double INTAKE_PERCENT_OUTPUT = 0.6;
  private static final Voltage MAX_VOLTAGE = Volts.of(12.0);
  private static final Voltage PIVOT_VOLTAGE_REQUEST = Volts.of(0);
  private static final Voltage MOTION_MAGIC_VOLTAGE = Volts.of(0);
  private static final Voltage ROLLER_VOLTAGE_REQUEST = Volts.of(0);
  private static final Angle HOMED_ANGLE = Degrees.of(110);
  private static final Angle STOWED_ANGLE = Degrees.of(100);
  private static final Angle INTAKE_ANGLE = Degrees.of(-4);
  private static final Angle INTAKE_AGITATE = Degrees.of(20);
  private static final Angle PIVOT_REDUCTION = Degrees.of(50.0);
  private static final double PIVOT_PERCENT_OUTPUT = 0.1;
  private static final Current STATOR_CURRENT_LIMIT = Amps.of(120);
  private static final Current SUPPLY_CURRENT_LIMIT = Amps.of(70);
  private static final Current HOMING_CURRENT_THRESHOLD = Amps.of(6);
  private static final double kPivotReduction = PIVOT_REDUCTION.in(Degrees);
  private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
  private static final Angle kPositionTolerance = Degrees.of(5);
  private static final double KP = 300.0;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double KV = 12.0 / kMaxPivotSpeed.in(RotationsPerSecond);
  private static final int NEW_SLOT = 0;

  public enum Speed {
    STOP(0.0),
    INTAKE(INTAKE_PERCENT_OUTPUT);

    private final double percentOutput;

    private Speed(double percentOutput) {
      this.percentOutput = percentOutput;
    }

    public Voltage voltage() {
        return MAX_VOLTAGE.times(percentOutput);
    }
  }

  public enum Position {
    HOMED(HOMED_ANGLE),
    STOWED(STOWED_ANGLE),
    INTAKE(INTAKE_ANGLE),
    AGITATE(INTAKE_AGITATE);

    private final Angle angle;

    private Position(Angle angle) {
      this.angle = angle;
    }

    public Angle angle() {
      return angle;
    }
  }

  private final TalonFX pivotMotor, rollerMotor;
  private final VoltageOut pivotVoltageRequest = new VoltageOut(PIVOT_VOLTAGE_REQUEST);
  private final MotionMagicVoltage pivotMotionMagicRequest =
      new MotionMagicVoltage(MOTION_MAGIC_VOLTAGE.in(Volts)).withSlot(NEW_SLOT);
  private final VoltageOut rollerVoltageRequest = new VoltageOut(ROLLER_VOLTAGE_REQUEST);

  private boolean isHomed = false;

  public Intake() {
    pivotMotor = new TalonFX(Ports.kIntakePivot, Ports.kCANivoreCANBus);
    rollerMotor = new TalonFX(Ports.kIntakeRollers, Ports.kRoboRioCANBus);
    configurePivotMotor();
    configureRollerMotor();
    SmartDashboard.putData(this);
  }

  private void configurePivotMotor() {
    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(STATOR_CURRENT_LIMIT.in(Amps)))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(SUPPLY_CURRENT_LIMIT.in(Amps)))
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
                    .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second)))
            .withSlot0(new Slot0Configs().withKP(KP).withKI(KI).withKD(KD).withKV(KV));
    pivotMotor.getConfigurator().apply(config);
  }

  private void configureRollerMotor() {
    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(STATOR_CURRENT_LIMIT.in(Amps)))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(SUPPLY_CURRENT_LIMIT.in(Amps)))
                    .withSupplyCurrentLimitEnable(true));
    rollerMotor.getConfigurator().apply(config);
  }

  private boolean isPositionWithinTolerance() {
    final Angle currentPosition = pivotMotor.getPosition().getValue();
    final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
    return currentPosition.isNear(targetPosition, kPositionTolerance);
  }

  private void setPivotPercentOutput(double percentOutput) {
    pivotMotor.setControl(pivotVoltageRequest.withOutput(MAX_VOLTAGE.times(percentOutput)));
  }

  public void set(Position position) {
    pivotMotor.setControl(pivotMotionMagicRequest.withPosition(position.angle()));
  }

  public void set(Speed speed) {
    rollerMotor.setControl(rollerVoltageRequest.withOutput(speed.voltage()));
  }

  public Command intakeCommand() {
    return startEnd(
        () -> {
          set(Position.INTAKE);
          set(Speed.INTAKE);
        },
        () -> set(Speed.STOP));
  }

  public Command agitateCommand() {
    return runOnce(() -> set(Speed.INTAKE))
        .andThen(
            Commands.sequence(
                    runOnce(() -> set(Position.AGITATE)),
                    Commands.waitUntil(this::isPositionWithinTolerance),
                    runOnce(() -> set(Position.INTAKE)),
                    Commands.waitUntil(this::isPositionWithinTolerance))
                .repeatedly())
        .handleInterrupt(
            () -> {
              set(Position.INTAKE);
              set(Speed.STOP);
            });
  }

  public Command homingCommand() {
    return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(PIVOT_PERCENT_OUTPUT)),
            Commands.waitUntil(
                () -> pivotMotor.getSupplyCurrent().getValue().gt(HOMING_CURRENT_THRESHOLD)),
            runOnce(
                () -> {
                  pivotMotor.setPosition(Position.HOMED.angle());
                  isHomed = true;
                  set(Position.STOWED);
                }))
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty(
        "Command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    builder.addDoubleProperty(
        "Angle (degrees)", () -> pivotMotor.getPosition().getValue().in(Degrees), null);
    builder.addDoubleProperty("RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);
    builder.addDoubleProperty(
        "Pivot Supply Current", () -> pivotMotor.getSupplyCurrent().getValue().in(Amps), null);
    builder.addDoubleProperty(
        "Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
  }
}
