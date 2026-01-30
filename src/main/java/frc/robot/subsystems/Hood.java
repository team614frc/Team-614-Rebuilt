package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.HoodConstants;
import frc.robot.Ports;

public class Hood extends SubsystemBase {
  private static final Distance kServoLength = Millimeters.of(HoodConstants.SERVO_LENGTH);
  private static final LinearVelocity kMaxServoSpeed =
      Millimeters.of(HoodConstants.SERVO_SPEED).per(Second);
  private static final double kMinPosition = HoodConstants.MIN_POSITION;
  private static final double kMaxPosition = HoodConstants.MAX_POSITION;
  private static final double kPositionTolerance = HoodConstants.POSITION_TOLERANCE;

  private final Servo leftServo;
  private final Servo rightServo;

  private double currentPosition = HoodConstants.CURRENT_POSITION;
  private double targetPosition = HoodConstants.TARGET_POSITION;
  private Time lastUpdateTime = Second.of(HoodConstants.LAST_UPDATE_TIME);

  public Hood() {
    leftServo = new Servo(Ports.kHoodLeftServo);
    rightServo = new Servo(Ports.kHoodRightServo);
    leftServo.setBoundsMicroseconds(
        HoodConstants.MAX,
        HoodConstants.DEADBAND_MAX,
        HoodConstants.CENTER,
        HoodConstants.DEADBAND_MIN,
        HoodConstants.MIN);
    rightServo.setBoundsMicroseconds(
        HoodConstants.MAX,
        HoodConstants.DEADBAND_MAX,
        HoodConstants.CENTER,
        HoodConstants.DEADBAND_MIN,
        HoodConstants.MIN);
    setPosition(currentPosition);
    SmartDashboard.putData(this);
  }

  /** Expects a position between 0.0 and 1.0 */
  public void setPosition(double position) {
    final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
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
    return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
  }

  private void updateCurrentPosition() {
    final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
    final Time elapsedTime = currentTime.minus(lastUpdateTime);
    lastUpdateTime = currentTime;

    if (isPositionWithinTolerance()) {
      currentPosition = targetPosition;
      return;
    }

    final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
    final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
    currentPosition =
        targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
  }

  @Override
  public void periodic() {
    updateCurrentPosition();
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
}
