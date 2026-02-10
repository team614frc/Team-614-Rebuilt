package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  private static final Distance SERVO_LENGTH = Millimeters.of(100);
  private static final LinearVelocity MAX_SERVO_SPEED = Millimeters.per(Second).of(20);
  private static final double MIN_POSITION = 0.01;
  private static final double MAX_POSITION = 0.77;
  private static final double POSITION_TOLERANCE = 0.01;
  private static final int MAX = 2000;
  private static final int DEADBAND_MAX = 1800;
  private static final int CENTER = 1500;
  private static final int DEADBAND_MIN = 1200;
  private static final int MIN = 1000;
  private final Servo leftServo;
  private final Servo rightServo;
  private double currentPosition = 0.5;
  private double targetPosition = 0.5;
  private Time lastUpdateTime = Seconds.of(0);

  public Hood() {
    leftServo = new Servo(Ports.kHoodLeftServo);
    rightServo = new Servo(Ports.kHoodRightServo);
    leftServo.setBoundsMicroseconds(MAX, DEADBAND_MAX, CENTER, DEADBAND_MIN, MIN);
    rightServo.setBoundsMicroseconds(MAX, DEADBAND_MAX, CENTER, DEADBAND_MIN, MIN);
    setPosition(currentPosition);
    SmartDashboard.putData(this);
  }

  /** Expects a position between 0.0 and 1.0 */
  public void setPosition(double position) {
    final double clampedPosition = MathUtil.clamp(position, MIN_POSITION, MAX_POSITION);
    leftServo.set(clampedPosition);
    rightServo.set(clampedPosition);
    targetPosition = clampedPosition;
  }

  /** Expects a position between 0.0 and 1.0 */
  public Command positionCommand(double position) {
    return runOnce(() -> setPosition(position))
        .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
  }

  public boolean isPositionWithinTolerance() {
    return MathUtil.isNear(targetPosition, currentPosition, POSITION_TOLERANCE);
  }

  private void updateCurrentPosition() {
    final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
    final Time elapsedTime = currentTime.minus(lastUpdateTime);
    lastUpdateTime = currentTime;

    if (isPositionWithinTolerance()) {
      currentPosition = targetPosition;
      return;
    }

    final Distance maxDistanceTraveled = MAX_SERVO_SPEED.times(elapsedTime);
    final double maxPercentageTraveled = maxDistanceTraveled.div(SERVO_LENGTH).in(Value);
    currentPosition =
        targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
  }

  @Override
  public void periodic() {
    updateCurrentPosition();

    // Log hood state
    Logger.recordOutput("Hood/CurrentPosition", currentPosition);
    Logger.recordOutput("Hood/TargetPosition", targetPosition);
    Logger.recordOutput("Hood/AngleDegrees", getAngle().in(Degrees));
    Logger.recordOutput("Hood/AtSetpoint", isPositionWithinTolerance());
    Logger.recordOutput("Hood/LeftServoPosition", leftServo.get());
    Logger.recordOutput("Hood/RightServoPosition", rightServo.get());

    Logger.recordOutput("Hood/CommandActive", getCurrentCommand() != null);
    if (getCurrentCommand() != null) {
      Logger.recordOutput("Hood/CommandName", getCurrentCommand().getName());
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty(
        "Command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    builder.addDoubleProperty("Current Position", () -> currentPosition, null);
    builder.addDoubleProperty("Target Position", () -> targetPosition, value -> setPosition(value));
  }

  public Angle getAngle() {
    double minAngleDeg = 10.0;
    double maxAngleDeg = 60.0;
    double angleDeg = minAngleDeg + (maxAngleDeg - minAngleDeg) * currentPosition;

    return Degrees.of(angleDeg);
  }
}
