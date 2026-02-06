package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  private final CANdle candle;

  public LEDs() {
    // CHANGE CAN ID LATER
    candle = new CANdle(0, "rio");
  }

  private SolidColor SolidColorWithColor(int red, int green, int blue) {
    RGBWColor color = new RGBWColor(red, green, blue);
    return new SolidColor(8, 399).withColor(color);
  }

  public void hubActive() {
    // Solid green = OUR HUB ACTIVE
    candle.setControl(SolidColorWithColor(0, 255, 0));
  }

  public void opponentHub() {
    // Solid red = OPPONENT HUB ACTIVE
    candle.setControl(SolidColorWithColor(255, 0, 0));
  }

  public void hubStartingSoon() {
    // Yellow = 5 second warning before OUR hub
    candle.setControl(SolidColorWithColor(255, 255, 0));
  }

  public void hubEndingSoon() {
    // Purple = 5 second warning before OUR hub ends
    candle.setControl(SolidColorWithColor(160, 0, 255));
  }

  public void transition() {
    // Blue during transition
    candle.setControl(SolidColorWithColor(0, 0, 255));
  }

  public void disabled() {
    // Dim white when disabled
    candle.setControl(SolidColorWithColor(40, 40, 40));
  }
}
