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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Ports;
import java.util.List;

public class Shooter extends SubsystemBase {
    private static final AngularVelocity kVelocityTolerance = RPM.of(ShooterConstants.VELOCITY_TOLERANCE);

    private final TalonFX leftMotor, middleMotor, rightMotor;
    private final List<TalonFX> motors;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(ShooterConstants.VELOCITY_VOLTAGE_SLOT).withSlot(ShooterConstants.NEW_SLOT);
    private final VoltageOut voltageRequest = new VoltageOut(ShooterConstants.VOLTAGE_OUT);

    private double dashboardTargetRPM = ShooterConstants.DASHBOARD_TARGET_RPM;

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
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(ShooterConstants.PEAK_REVERSE_VOLTAGE))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(ShooterConstants.STATOR_CURRENT_LIMIT))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(ShooterConstants.SUPPLY_CURRENT_LIMIT))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(ShooterConstants.kP)
                    .withKI(ShooterConstants.kI)
                    .withKD(ShooterConstants.kD)
                    .withKV(ShooterConstants.MAX_VOLTAGE / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        
        motor.getConfigurator().apply(config);
    }

  public void setRPM(double rpm) {
    for (final TalonFX motor : motors) {
      motor.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
    }
  }

    public void setPercentOutput(double percentOutput) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                voltageRequest
                    .withOutput(Volts.of(percentOutput * ShooterConstants.MAX_VOLTAGE))
            );
        }
    }

    public void stop() {
        setPercentOutput(ShooterConstants.STOP_PERCENT_OUTPUT);
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
              return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
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
}
