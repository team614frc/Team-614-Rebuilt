package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  // ── Tolerances & limits ──────────────────────────────────────────────────

  // Velocity must be within this of setpoint to be considered "in tolerance"
  // (switches to torque-current bang-bang, and counts as atGoal).
  // Mirrors 6328's torqueCurrentControlTolerance (default 20 rad/s ≈ 191 RPM).
  private static final double TORQUE_CURRENT_TOLERANCE_RPM = 100.0;

  // How long (seconds) velocity must stay OUT of tolerance before we switch
  // back to duty-cycle. kFalling debounce prevents brief oscillation dips from
  // dropping out of torque-current mode. Mirrors 6328's torqueCurrentControlDebounce.
  private static final double TORQUE_CURRENT_DEBOUNCE_SECONDS = 0.025;

  // How long (seconds) velocity must stay in tolerance before atGoal is true.
  // Mirrors 6328's atGoalDebounce (default 0.2s).
  private static final double AT_GOAL_DEBOUNCE_SECONDS = 0.2;

  // Peak torque-current during torque-current bang-bang (Amps).
  // Tune: too low = sluggish idle hold; too high = violent spikes.
  private static final double IDLE_PEAK_TORQUE_AMPS = 40.0;

  // ── Bang-bang kP ─────────────────────────────────────────────────────────
  private static final double BANG_BANG_KP = 999999.0;

  // ── Signal refresh rates ─────────────────────────────────────────────────
  private static final double VELOCITY_UPDATE_FREQ_HZ = 100.0;
  private static final double CURRENT_UPDATE_FREQ_HZ = 100.0;
  private static final double SUPPLY_CURRENT_UPDATE_FREQ_HZ = 50.0;
  private static final double TEMP_UPDATE_FREQ_HZ = 4.0;

  // ── Hardware ─────────────────────────────────────────────────────────────
  private final TalonFX leftMotor, middleMotor, rightMotor;
  private final List<TalonFX> motors;

  // ── Control requests ─────────────────────────────────────────────────────
  private final VelocityDutyCycle leftIdleRequest = new VelocityDutyCycle(0).withSlot(1);
  private final VelocityDutyCycle middleIdleRequest = new VelocityDutyCycle(0).withSlot(1);
  private final VelocityDutyCycle rightIdleRequest = new VelocityDutyCycle(0).withSlot(1);

  private static final double IDLE_RPM = 2000.0;

  private final VelocityTorqueCurrentFOC leftTorqueRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final VelocityTorqueCurrentFOC middleTorqueRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final VelocityTorqueCurrentFOC rightTorqueRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  private final NeutralOut neutralRequest = new NeutralOut();

  // ── Debouncers ───────────────────────────────────────────────────────────
  // torque debouncers: kFalling — switches TO torque-current immediately when in
  // tolerance, but only switches BACK to duty-cycle after being out of tolerance
  // for TORQUE_CURRENT_DEBOUNCE_SECONDS. Brief oscillation dips stay in torque mode.
  private final Debouncer leftTorqueDebouncer =
      new Debouncer(TORQUE_CURRENT_DEBOUNCE_SECONDS, DebounceType.kFalling);
  private final Debouncer middleTorqueDebouncer =
      new Debouncer(TORQUE_CURRENT_DEBOUNCE_SECONDS, DebounceType.kFalling);
  private final Debouncer rightTorqueDebouncer =
      new Debouncer(TORQUE_CURRENT_DEBOUNCE_SECONDS, DebounceType.kFalling);

  // atGoal debouncers: kFalling — atGoal goes true quickly when in tolerance,
  // stays true briefly after going out (prevents flickering on the robot side).
  private final Debouncer leftAtGoalDebouncer =
      new Debouncer(AT_GOAL_DEBOUNCE_SECONDS, DebounceType.kFalling);
  private final Debouncer middleAtGoalDebouncer =
      new Debouncer(AT_GOAL_DEBOUNCE_SECONDS, DebounceType.kFalling);
  private final Debouncer rightAtGoalDebouncer =
      new Debouncer(AT_GOAL_DEBOUNCE_SECONDS, DebounceType.kFalling);

  // ── State ─────────────────────────────────────────────────────────────────
  private double leftTargetRPM = 0;
  private double middleTargetRPM = 0;
  private double rightTargetRPM = 0;

  // Mirrors 6328's lastTorqueCurrentControl — detects transitions out of torque
  // mode (i.e., a ball was launched) to increment ballCount.
  private boolean leftLastTorque = false;
  private boolean middleLastTorque = false;
  private boolean rightLastTorque = false;

  private long ballCount = 0;

  private boolean leftAtGoal = false;
  private boolean middleAtGoal = false;
  private boolean rightAtGoal = false;

  private double dashboardTargetRPM = 3750;

  private boolean leftTestEnabled = false;
  private boolean middleTestEnabled = false;
  private boolean rightTestEnabled = false;

  public Shooter() {
    leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kRoboRioCANBus);
    middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kRoboRioCANBus);
    rightMotor = new TalonFX(Ports.kShooterRight, Ports.kRoboRioCANBus);
    motors = List.of(leftMotor, middleMotor, rightMotor);

    configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive);
    configureMotor(middleMotor, InvertedValue.CounterClockwise_Positive);
    configureMotor(rightMotor, InvertedValue.Clockwise_Positive);

    SmartDashboard.putData(this);
  }

  // ── Configuration ────────────────────────────────────────────────────────

  private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withPeakForwardDutyCycle(1.0)
                    .withPeakReverseDutyCycle(0.0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(140))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(IDLE_PEAK_TORQUE_AMPS)
                    .withPeakReverseTorqueCurrent(0.0))
            .withSlot0(new Slot0Configs().withKP(BANG_BANG_KP)) // bang-bang for commanded shots
            .withSlot1(
                new Slot1Configs() // gentle PID for idle
                    .withKP(0.035)
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKV(0.0103)); // feedforward — tune: 0.12 is a starting point for Falcon

    motor.getConfigurator().apply(config);

    motor.getVelocity().setUpdateFrequency(VELOCITY_UPDATE_FREQ_HZ);
    motor.getStatorCurrent().setUpdateFrequency(CURRENT_UPDATE_FREQ_HZ);
    motor.getSupplyCurrent().setUpdateFrequency(SUPPLY_CURRENT_UPDATE_FREQ_HZ);
    motor.getDeviceTemp().setUpdateFrequency(TEMP_UPDATE_FREQ_HZ);
    motor.optimizeBusUtilization();
  }

  // ── Core velocity control (mirrors 6328's runVelocity) ───────────────────

  /**
   * Runs one motor using the same two-mode bang-bang scheme as 6328: torque-current when in
   * tolerance, duty-cycle otherwise. Returns the new torqueCurrentControl state for lastTorque
   * tracking.
   */
  private boolean runMotorVelocity(
      TalonFX motor,
      VelocityDutyCycle dutyReq,
      VelocityTorqueCurrentFOC torqueReq,
      Debouncer torqueDebouncer,
      Debouncer atGoalDebouncer,
      boolean lastTorque,
      double targetRPM) {

    final double currentRPM = motor.getVelocity().getValue().in(RPM);
    final boolean inTolerance = Math.abs(currentRPM - targetRPM) <= TORQUE_CURRENT_TOLERANCE_RPM;

    final boolean torqueCurrentControl = torqueDebouncer.calculate(inTolerance);
    atGoalDebouncer.calculate(inTolerance);

    // A transition out of torque-current means a ball was just launched
    if (!torqueCurrentControl && lastTorque) {
      ballCount++;
    }

    if (torqueCurrentControl) {
      motor.setControl(torqueReq.withVelocity(RPM.of(targetRPM)));
    } else {
      motor.setControl(dutyReq.withVelocity(RPM.of(targetRPM)));
    }

    return torqueCurrentControl;
  }

  public void setRPM(double leftRPM, double middleRPM, double rightRPM) {
    leftTargetRPM = leftRPM;
    middleTargetRPM = middleRPM;
    rightTargetRPM = rightRPM;
  }

  public void setRPM(double rpm) {
    setRPM(rpm, rpm, rpm);
  }

  public void stop() {
    leftTargetRPM = middleTargetRPM = rightTargetRPM = 0;
    for (TalonFX m : motors) m.setControl(neutralRequest);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public boolean isAtGoal() {
    return leftAtGoal && middleAtGoal && rightAtGoal;
  }

  public boolean isVelocityWithinTolerance() {
    return leftLastTorque && middleLastTorque && rightLastTorque;
  }

  public long getBallCount() {
    return ballCount;
  }

  public Command spinUpCommand(double rpm) {
    return spinUpCommand(rpm, rpm, rpm);
  }

  public Command spinUpCommand(double leftRPM, double middleRPM, double rightRPM) {
    return runOnce(() -> setRPM(leftRPM, middleRPM, rightRPM))
        .andThen(Commands.waitUntil(this::isAtGoal));
  }

  public Command dashboardSpinUpCommand() {
    return defer(() -> spinUpCommand(dashboardTargetRPM));
  }

  public Command shuttleCommand() {
    return defer(() -> spinUpCommand(4500));
  }

  public LinearVelocity getExitVelocity() {
    double wheelRadiusMeters = 0.05;
    double wheelRPS = leftMotor.getVelocity().getValue().in(RotationsPerSecond);
    return Units.MetersPerSecond.of(2.0 * Math.PI * wheelRadiusMeters * wheelRPS);
  }

  public Command punchThroughCommand() {
    return startEnd(
        () -> setRPM(6000), // full send — ball goes through regardless
        this::stop);
  }

  @Override
  public void periodic() {
    if (getCurrentCommand() == null) {
      leftTargetRPM = leftTestEnabled ? dashboardTargetRPM : 0;
      middleTargetRPM = middleTestEnabled ? dashboardTargetRPM : 0;
      rightTargetRPM = rightTestEnabled ? dashboardTargetRPM : 0;
    }

    // Left
    if (leftTargetRPM == 0) {
      leftTorqueDebouncer.calculate(false);
      leftAtGoalDebouncer.calculate(false);
      leftMotor.setControl(leftIdleRequest.withVelocity(RPM.of(IDLE_RPM))); // gentle PID idle
      leftLastTorque = false;
      leftAtGoal = false;
    } else {
      leftLastTorque =
          runMotorVelocity(
              leftMotor,
              leftIdleRequest,
              leftTorqueRequest,
              leftTorqueDebouncer,
              leftAtGoalDebouncer,
              leftLastTorque,
              leftTargetRPM);
      leftAtGoal = leftLastTorque; // atGoal = currently in torque-current mode
    }

    // Middle
    if (middleTargetRPM == 0) {
      middleTorqueDebouncer.calculate(false);
      middleAtGoalDebouncer.calculate(false);
      middleMotor.setControl(middleIdleRequest.withVelocity(RPM.of(IDLE_RPM)));
      middleLastTorque = false;
      middleAtGoal = false;
    } else {
      middleLastTorque =
          runMotorVelocity(
              middleMotor,
              middleIdleRequest,
              middleTorqueRequest,
              middleTorqueDebouncer,
              middleAtGoalDebouncer,
              middleLastTorque,
              middleTargetRPM);
      middleAtGoal = middleLastTorque;
    }

    // Right
    if (rightTargetRPM == 0) {
      rightTorqueDebouncer.calculate(false);
      rightAtGoalDebouncer.calculate(false);
      rightMotor.setControl(rightIdleRequest.withVelocity(RPM.of(IDLE_RPM)));
      rightLastTorque = false;
      rightAtGoal = false;
    } else {
      rightLastTorque =
          runMotorVelocity(
              rightMotor,
              rightIdleRequest,
              rightTorqueRequest,
              rightTorqueDebouncer,
              rightAtGoalDebouncer,
              rightLastTorque,
              rightTargetRPM);
      rightAtGoal = rightLastTorque;
    }

    // Logging
    Logger.recordOutput("Shooter/Left/VelocityRPM", leftMotor.getVelocity().getValue().in(RPM));
    Logger.recordOutput(
        "Shooter/Left/StatorCurrentAmps", leftMotor.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Left/SupplyCurrentAmps", leftMotor.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Left/TemperatureCelsius", leftMotor.getDeviceTemp().getValue().in(Celsius));
    Logger.recordOutput("Shooter/Left/TargetRPM", leftTargetRPM);
    Logger.recordOutput("Shooter/Left/TorqueCurrentMode", leftLastTorque);
    Logger.recordOutput("Shooter/Left/AtGoal", leftAtGoal);

    Logger.recordOutput("Shooter/Middle/VelocityRPM", middleMotor.getVelocity().getValue().in(RPM));
    Logger.recordOutput(
        "Shooter/Middle/StatorCurrentAmps", middleMotor.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Middle/SupplyCurrentAmps", middleMotor.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Middle/TemperatureCelsius", middleMotor.getDeviceTemp().getValue().in(Celsius));
    Logger.recordOutput("Shooter/Middle/TargetRPM", middleTargetRPM);
    Logger.recordOutput("Shooter/Middle/TorqueCurrentMode", middleLastTorque);
    Logger.recordOutput("Shooter/Middle/AtGoal", middleAtGoal);

    Logger.recordOutput("Shooter/Right/VelocityRPM", rightMotor.getVelocity().getValue().in(RPM));
    Logger.recordOutput(
        "Shooter/Right/StatorCurrentAmps", rightMotor.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Right/SupplyCurrentAmps", rightMotor.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Right/TemperatureCelsius", rightMotor.getDeviceTemp().getValue().in(Celsius));
    Logger.recordOutput("Shooter/Right/TargetRPM", rightTargetRPM);
    Logger.recordOutput("Shooter/Right/TorqueCurrentMode", rightLastTorque);
    Logger.recordOutput("Shooter/Right/AtGoal", rightAtGoal);

    Logger.recordOutput("Shooter/AtGoal", isAtGoal());
    Logger.recordOutput("Shooter/BallCount", ballCount);
    Logger.recordOutput("Shooter/ExitVelocityMPS", getExitVelocity().in(Units.MetersPerSecond));
    Logger.recordOutput("Shooter/CommandActive", getCurrentCommand() != null);
    if (getCurrentCommand() != null)
      Logger.recordOutput("Shooter/CommandName", getCurrentCommand().getName());
  }

  // ── Sendable ─────────────────────────────────────────────────────────────

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Left RPM", () -> leftMotor.getVelocity().getValue().in(RPM), null);
    builder.addDoubleProperty(
        "Middle RPM", () -> middleMotor.getVelocity().getValue().in(RPM), null);
    builder.addDoubleProperty("Right RPM", () -> rightMotor.getVelocity().getValue().in(RPM), null);

    builder.addDoubleProperty("Left Target RPM", () -> leftTargetRPM, null);
    builder.addDoubleProperty("Middle Target RPM", () -> middleTargetRPM, null);
    builder.addDoubleProperty("Right Target RPM", () -> rightTargetRPM, null);

    builder.addBooleanProperty("Left Torque Mode", () -> leftLastTorque, null);
    builder.addBooleanProperty("Middle Torque Mode", () -> middleLastTorque, null);
    builder.addBooleanProperty("Right Torque Mode", () -> rightLastTorque, null);

    builder.addBooleanProperty("Left At Goal", () -> leftAtGoal, null);
    builder.addBooleanProperty("Middle At Goal", () -> middleAtGoal, null);
    builder.addBooleanProperty("Right At Goal", () -> rightAtGoal, null);

    builder.addBooleanProperty("At Goal", this::isAtGoal, null);
    builder.addDoubleProperty("Ball Count", () -> ballCount, null);
    builder.addDoubleProperty(
        "Dashboard RPM", () -> dashboardTargetRPM, v -> dashboardTargetRPM = v);

    builder.addBooleanProperty("Left Test Enable", () -> leftTestEnabled, v -> leftTestEnabled = v);
    builder.addBooleanProperty(
        "Middle Test Enable", () -> middleTestEnabled, v -> middleTestEnabled = v);
    builder.addBooleanProperty(
        "Right Test Enable", () -> rightTestEnabled, v -> rightTestEnabled = v);
  }
}
