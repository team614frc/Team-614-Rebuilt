package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
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

  private final Servo leftServo;
  private final Servo rightServo;

  private double currentPosition = 0.5;
  private double targetPosition = 0.5;
  private Time lastUpdateTime = Seconds.of(0);

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
    final double clampedPosition =
        MathUtil.clamp(position, HoodConstants.MIN_POSITION, HoodConstants.MAX_POSITION);
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
    return MathUtil.isNear(targetPosition, currentPosition, HoodConstants.POSITION_TOLERANCE);
  }

  private void updateCurrentPosition() {
    final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
    final Time elapsedTime = currentTime.minus(lastUpdateTime);
    lastUpdateTime = currentTime;

    if (isPositionWithinTolerance()) {
      currentPosition = targetPosition;
      return;
    }

    final Distance maxDistanceTraveled = HoodConstants.MAX_SERVO_SPEED.times(elapsedTime);
    final double maxPercentageTraveled =
        maxDistanceTraveled.div(HoodConstants.SERVO_LENGTH).in(Millimeters);
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
