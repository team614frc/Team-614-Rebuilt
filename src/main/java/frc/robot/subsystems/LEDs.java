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

  public void setColor(HubState state) {
    candle.setControl(new SolidColor(8, 399).withColor(new RGBWColor(state.r, state.g, state.b)));
  }

  public enum HubState {
    HUB_ACTIVE(0, 255, 0), // Green
    OPPONENT_HUB(255, 0, 0), // Red
    HUB_STARTING_SOON(255, 255, 0), // Yellow
    HUB_ENDING_SOON(160, 0, 255), // Purple
    TRANSITION(0, 0, 255), // Blue
    DISABLED(40, 40, 40); // Dim white

    public final int r;
    public final int g;
    public final int b;

    HubState(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }
}
