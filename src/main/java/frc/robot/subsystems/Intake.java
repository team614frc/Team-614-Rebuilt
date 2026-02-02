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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Intake extends SubsystemBase {
  public enum Speed {
    STOP(IntakeConstants.STOP_PERCENT_OUTPUT),
    INTAKE(IntakeConstants.INTAKE_PERCENT_OUTPUT);

    private final double percentOutput;

    private Speed(double percentOutput) {
      this.percentOutput = percentOutput;
    }

    public Voltage voltage() {
      return Volts.of(percentOutput * IntakeConstants.MAX_VOLTAGE.in(Volts));
    }
  }

  public enum Position {
    HOMED(IntakeConstants.HOMED_ANGLE),
    STOWED(IntakeConstants.STOWED_ANGLE),
    INTAKE(IntakeConstants.INTAKE_ANGLE),
    AGITATE(IntakeConstants.AGITATE);

    private final Angle angle;

    private Position(Angle angle) {
      this.angle = angle;
    }

    public Angle angle() {
      return angle;
    }
  }

  private static final double kPivotReduction = IntakeConstants.PIVOT_REDUCTION.in(Degrees);
  private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
  private static final Angle kPositionTolerance = Degrees.of(IntakeConstants.POSITION_TOLERANCE);

  private final TalonFX pivotMotor, rollerMotor;
  private final VoltageOut pivotVoltageRequest =
      new VoltageOut(IntakeConstants.PIVOT_VOLTAGE_REQUEST);
  private final MotionMagicVoltage pivotMotionMagicRequest =
      new MotionMagicVoltage(IntakeConstants.MOTION_MAGIC_VOLTAGE.in(Volts))
          .withSlot(IntakeConstants.NEW_SLOT);
  private final VoltageOut rollerVoltageRequest =
      new VoltageOut(IntakeConstants.ROLLER_VOLTAGE_REQUEST);

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
                    .withStatorCurrentLimit(Amps.of(IntakeConstants.STATOR_CURRENT_LIMIT.in(Amps)))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(IntakeConstants.SUPPLY_CURRENT_LIMIT.in(Amps)))
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
                    .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second)))
            .withSlot0(
                new Slot0Configs()
                    .withKP(IntakeConstants.KP)
                    .withKI(IntakeConstants.KI)
                    .withKD(IntakeConstants.KD)
                    .withKV(
                        IntakeConstants.MAX_VOLTAGE.in(Volts)
                            / kMaxPivotSpeed.in(
                                RotationsPerSecond)) // 12 volts when requesting max RPS
                );
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
                    .withStatorCurrentLimit(Amps.of(IntakeConstants.STATOR_CURRENT_LIMIT.in(Amps)))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(IntakeConstants.SUPPLY_CURRENT_LIMIT.in(Amps)))
                    .withSupplyCurrentLimitEnable(true));
    rollerMotor.getConfigurator().apply(config);
  }

  private boolean isPositionWithinTolerance() {
    final Angle currentPosition = pivotMotor.getPosition().getValue();
    final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
    return currentPosition.isNear(targetPosition, kPositionTolerance);
  }

  private void setPivotPercentOutput(double percentOutput) {
    pivotMotor.setControl(
        pivotVoltageRequest.withOutput(
            Volts.of(percentOutput * IntakeConstants.MAX_VOLTAGE.in(Volts))));
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
            runOnce(() -> setPivotPercentOutput(IntakeConstants.PIVOT_PERCENT_OUTPUT)),
            Commands.waitUntil(() -> pivotMotor.getSupplyCurrent().getValue().in(Amps) > 6),
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
