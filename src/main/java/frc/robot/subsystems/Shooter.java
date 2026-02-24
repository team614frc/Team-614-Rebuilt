package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
import frc.robot.Ports;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(100);
  private static final Voltage MAX_VOLTAGE = Volts.of(12.0);
  private static final Current STATOR_CURRENT_LIMIT = Amps.of(120);
  private static final Current SUPPLY_CURRENT_LIMIT = Amps.of(70);

  // With TorqueCurrentFOC, kP units = Amps per RPS error, kV = Amps per RPS
  // These will need retuning — start here and adjust via SmartDashboard
  private static final double LEFT_KP = 0.785, LEFT_KI = 0.0, LEFT_KD = 0.0;
  private static final double MIDDLE_KP = 0.785, MIDDLE_KI = 0.0, MIDDLE_KD = 0.0;
  private static final double RIGHT_KP = 0.785, RIGHT_KI = 0.0, RIGHT_KD = 0.0;

  // kV: Amps per RPS — spin freely, read stator current at target RPS, then kV = current / RPS
  private static final double kV = 0.1225;
  private static final double TEST_RPM = 3590.0;

  // Signal refresh rates (Hz).
  // On RoboRIO CAN bus: keep velocity <= 100 Hz to avoid starving other devices.
  // On CANivore: safe to bump velocity and current up to 250 Hz.
  private static final double VELOCITY_UPDATE_FREQ_HZ = 100.0;
  private static final double CURRENT_UPDATE_FREQ_HZ = 100.0;
  private static final double SUPPLY_CURRENT_UPDATE_FREQ_HZ = 50.0;
  private static final double TEMP_UPDATE_FREQ_HZ = 4.0;

  private final TalonFX leftMotor, middleMotor, rightMotor;
  private final List<TalonFX> motors;

  // FOC velocity control requests — output is torque current (Amps), not voltage
  private final VelocityTorqueCurrentFOC leftVelocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final VelocityTorqueCurrentFOC middleVelocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final VelocityTorqueCurrentFOC rightVelocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  // Use NeutralOut to coast to stop (replaces VoltageOut at 0)
  private final NeutralOut neutralRequest = new NeutralOut();

  private double dashboardTargetRPM = 3500;

  // Individual motor test toggles (SmartDashboard checkboxes)
  private boolean leftTestEnabled = false;
  private boolean middleTestEnabled = false;
  private boolean rightTestEnabled = false;

  // Per-motor live-tunable PIDF gains
  private double leftKP = LEFT_KP, leftKI = LEFT_KI, leftKD = LEFT_KD;
  private double middleKP = MIDDLE_KP, middleKI = MIDDLE_KI, middleKD = MIDDLE_KD;
  private double rightKP = RIGHT_KP, rightKI = RIGHT_KI, rightKD = RIGHT_KD;
  private double leftKV = kV, middleKV = kV, rightKV = kV;

  // Track last-applied gains to avoid spamming CAN bus
  private final double[] leftApplied = {-1, -1, -1, -1}; // [kP, kI, kD, kV]
  private final double[] middleApplied = {-1, -1, -1, -1};
  private final double[] rightApplied = {-1, -1, -1, -1};

  public Shooter() {
    leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kRoboRioCANBus);
    middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kRoboRioCANBus);
    rightMotor = new TalonFX(Ports.kShooterRight, Ports.kRoboRioCANBus);
    motors = List.of(leftMotor, middleMotor, rightMotor);

    configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive, LEFT_KP, LEFT_KI, LEFT_KD);
    configureMotor(
        middleMotor, InvertedValue.CounterClockwise_Positive, MIDDLE_KP, MIDDLE_KI, MIDDLE_KD);
    configureMotor(rightMotor, InvertedValue.Clockwise_Positive, RIGHT_KP, RIGHT_KI, RIGHT_KD);

    SmartDashboard.putData(this);
  }

  private void configureMotor(
      TalonFX motor, InvertedValue invertDirection, double kP, double kI, double kD) {
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
            // With TorqueCurrentFOC: kP = Amps/RPS error, kV = Amps/RPS, kA = Amps/(RPS/s)
            .withSlot0(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKV(kV));

    motor.getConfigurator().apply(config);

    // Configure signal refresh rates AFTER applying the full config.
    // Call setUpdateFrequency on every signal read in periodic() or logging
    // BEFORE optimizeBusUtilization(), which silences all un-configured signals.
    motor.getVelocity().setUpdateFrequency(VELOCITY_UPDATE_FREQ_HZ);
    motor.getStatorCurrent().setUpdateFrequency(CURRENT_UPDATE_FREQ_HZ);
    motor.getSupplyCurrent().setUpdateFrequency(SUPPLY_CURRENT_UPDATE_FREQ_HZ);
    motor.getDeviceTemp().setUpdateFrequency(TEMP_UPDATE_FREQ_HZ);

    // Disable all signals not explicitly configured above to free up CAN bandwidth
    motor.optimizeBusUtilization();
  }

  /**
   * Applies updated Slot0 gains to a motor only when values have changed, then runs or stops the
   * motor based on the test toggle. Called from periodic().
   */
  private void handleMotorTest(
      TalonFX motor,
      VelocityTorqueCurrentFOC request,
      boolean enabled,
      double kP,
      double kI,
      double kD,
      double kVval,
      double[] lastApplied /* [kP, kI, kD, kV] */) {

    // Only apply config when a gain has actually changed
    if (kP != lastApplied[0]
        || kI != lastApplied[1]
        || kD != lastApplied[2]
        || kVval != lastApplied[3]) {
      motor
          .getConfigurator()
          .apply(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKV(kVval));
      lastApplied[0] = kP;
      lastApplied[1] = kI;
      lastApplied[2] = kD;
      lastApplied[3] = kVval;
    }

    if (enabled) {
      motor.setControl(request.withVelocity(RPM.of(TEST_RPM)));
    } else {
      motor.setControl(neutralRequest);
    }
  }

  public void setRPM(double leftRPM, double middleRPM, double rightRPM) {
    leftMotor.setControl(leftVelocityRequest.withVelocity(RPM.of(leftRPM)));
    middleMotor.setControl(middleVelocityRequest.withVelocity(RPM.of(middleRPM)));
    rightMotor.setControl(rightVelocityRequest.withVelocity(RPM.of(rightRPM)));
  }

  /** Convenience overload: same RPM for all motors. */
  public void setRPM(double rpm) {
    setRPM(rpm, rpm, rpm);
  }

  public void setPercentOutput(double percentOutput) {
    for (final TalonFX motor : motors) {
      // Approximate percent output via voltage for manual control
      motor.setControl(
          new com.ctre.phoenix6.controls.VoltageOut(MAX_VOLTAGE.in(Volts) * percentOutput));
    }
  }

  public void stop() {
    for (final TalonFX motor : motors) {
      motor.setControl(neutralRequest);
    }
  }

  public Command spinUpCommand(double rpm) {
    return spinUpCommand(rpm, rpm, rpm);
  }

  public Command spinUpCommand(double leftRPM, double middleRPM, double rightRPM) {
    return runOnce(() -> setRPM(leftRPM, middleRPM, rightRPM))
        .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
  }

  public Command dashboardSpinUpCommand() {
    return defer(() -> spinUpCommand(dashboardTargetRPM));
  }

  public boolean isVelocityWithinTolerance() {
    return isMotorAtTarget(leftMotor, leftVelocityRequest)
        && isMotorAtTarget(middleMotor, middleVelocityRequest)
        && isMotorAtTarget(rightMotor, rightVelocityRequest);
  }

  /**
   * Checks if a motor's actual velocity is within tolerance of the request's target velocity. Uses
   * velocity comparison only — no control mode check — so it works correctly across control mode
   * transitions and with FOC requests.
   */
  private boolean isMotorAtTarget(TalonFX motor, VelocityTorqueCurrentFOC request) {
    final AngularVelocity currentVelocity = motor.getVelocity().getValue();
    final AngularVelocity targetVelocity = request.getVelocityMeasure();
    return currentVelocity.isNear(targetVelocity, VELOCITY_TOLERANCE);
  }

  @Override
  public void periodic() {
    // Handle individual motor test mode (only active when no command is running)
    if (getCurrentCommand() == null) {
      handleMotorTest(
          leftMotor,
          leftVelocityRequest,
          leftTestEnabled,
          leftKP,
          leftKI,
          leftKD,
          leftKV,
          leftApplied);
      handleMotorTest(
          middleMotor,
          middleVelocityRequest,
          middleTestEnabled,
          middleKP,
          middleKI,
          middleKD,
          middleKV,
          middleApplied);
      handleMotorTest(
          rightMotor,
          rightVelocityRequest,
          rightTestEnabled,
          rightKP,
          rightKI,
          rightKD,
          rightKV,
          rightApplied);
    }

    Logger.recordOutput("Shooter/Left/VelocityRPM", leftMotor.getVelocity().getValue().in(RPM));
    Logger.recordOutput(
        "Shooter/Left/StatorCurrentAmps", leftMotor.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Left/SupplyCurrentAmps", leftMotor.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Left/TemperatureCelsius", leftMotor.getDeviceTemp().getValue().in(Celsius));
    Logger.recordOutput("Shooter/Left/TargetRPM", leftVelocityRequest.getVelocityMeasure().in(RPM));
    Logger.recordOutput("Shooter/Left/AtSetpoint", isMotorAtTarget(leftMotor, leftVelocityRequest));

    Logger.recordOutput("Shooter/Middle/VelocityRPM", middleMotor.getVelocity().getValue().in(RPM));
    Logger.recordOutput(
        "Shooter/Middle/StatorCurrentAmps", middleMotor.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Middle/SupplyCurrentAmps", middleMotor.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Middle/TemperatureCelsius", middleMotor.getDeviceTemp().getValue().in(Celsius));
    Logger.recordOutput(
        "Shooter/Middle/TargetRPM", middleVelocityRequest.getVelocityMeasure().in(RPM));
    Logger.recordOutput(
        "Shooter/Middle/AtSetpoint", isMotorAtTarget(middleMotor, middleVelocityRequest));

    Logger.recordOutput("Shooter/Right/VelocityRPM", rightMotor.getVelocity().getValue().in(RPM));
    Logger.recordOutput(
        "Shooter/Right/StatorCurrentAmps", rightMotor.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Right/SupplyCurrentAmps", rightMotor.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Shooter/Right/TemperatureCelsius", rightMotor.getDeviceTemp().getValue().in(Celsius));
    Logger.recordOutput(
        "Shooter/Right/TargetRPM", rightVelocityRequest.getVelocityMeasure().in(RPM));
    Logger.recordOutput(
        "Shooter/Right/AtSetpoint", isMotorAtTarget(rightMotor, rightVelocityRequest));

    Logger.recordOutput("Shooter/AtSetpoint", isVelocityWithinTolerance());
    Logger.recordOutput("Shooter/ExitVelocityMPS", getExitVelocity().in(Units.MetersPerSecond));
    Logger.recordOutput("Shooter/CommandActive", getCurrentCommand() != null);
    if (getCurrentCommand() != null) {
      Logger.recordOutput("Shooter/CommandName", getCurrentCommand().getName());
    }
  }

  private void addMotorSendable(
      SendableBuilder builder, TalonFX motor, String name, VelocityTorqueCurrentFOC request) {
    builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
    builder.addDoubleProperty(
        name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
    builder.addDoubleProperty(
        name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    builder.addDoubleProperty(
        name + " Target RPM", () -> request.getVelocityMeasure().in(RPM), null);
    builder.addBooleanProperty(name + " At Setpoint", () -> isMotorAtTarget(motor, request), null);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    addMotorSendable(builder, leftMotor, "Left", leftVelocityRequest);
    addMotorSendable(builder, middleMotor, "Middle", middleVelocityRequest);
    addMotorSendable(builder, rightMotor, "Right", rightVelocityRequest);

    builder.addDoubleProperty(
        "Dashboard RPM", () -> dashboardTargetRPM, v -> dashboardTargetRPM = v);
    builder.addStringProperty(
        "Command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    builder.addBooleanProperty("At Setpoint", this::isVelocityWithinTolerance, null);

    // --- Individual motor test toggles (checkbox = spin at TEST_RPM) ---
    builder.addBooleanProperty("Left Test Enable", () -> leftTestEnabled, v -> leftTestEnabled = v);
    builder.addBooleanProperty(
        "Middle Test Enable", () -> middleTestEnabled, v -> middleTestEnabled = v);
    builder.addBooleanProperty(
        "Right Test Enable", () -> rightTestEnabled, v -> rightTestEnabled = v);

    // --- Per-motor live PIDF tuning ---
    builder.addDoubleProperty("Left kP", () -> leftKP, v -> leftKP = v);
    builder.addDoubleProperty("Left kI", () -> leftKI, v -> leftKI = v);
    builder.addDoubleProperty("Left kD", () -> leftKD, v -> leftKD = v);
    builder.addDoubleProperty("Left kV", () -> leftKV, v -> leftKV = v);

    builder.addDoubleProperty("Middle kP", () -> middleKP, v -> middleKP = v);
    builder.addDoubleProperty("Middle kI", () -> middleKI, v -> middleKI = v);
    builder.addDoubleProperty("Middle kD", () -> middleKD, v -> middleKD = v);
    builder.addDoubleProperty("Middle kV", () -> middleKV, v -> middleKV = v);

    builder.addDoubleProperty("Right kP", () -> rightKP, v -> rightKP = v);
    builder.addDoubleProperty("Right kI", () -> rightKI, v -> rightKI = v);
    builder.addDoubleProperty("Right kD", () -> rightKD, v -> rightKD = v);
    builder.addDoubleProperty("Right kV", () -> rightKV, v -> rightKV = v);
  }

  public LinearVelocity getExitVelocity() {
    double wheelRadiusMeters = 0.05;
    double wheelRPS = leftMotor.getVelocity().getValue().in(RotationsPerSecond);
    double mps = 2.0 * Math.PI * wheelRadiusMeters * wheelRPS;
    return Units.MetersPerSecond.of(mps);
  }
}
