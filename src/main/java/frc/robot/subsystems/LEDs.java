package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;

public class LEDs extends SubsystemBase {

    private final CANdle candle;

    public LEDs() {
        // CHANGE CAN ID LATER
        candle = new CANdle(0);
    }

    public void hubActive() {
    // Solid green = OUR HUB ACTIVE
    candle.setLEDs(0, 255, 0);
}

public void opponentHub() {
    // Solid red = OPPONENT HUB ACTIVE
    candle.setLEDs(255, 0, 0);
}

public void hubStartingSoon() {
    // Yellow = 5 second warning before OUR hub
    candle.setLEDs(255, 255, 0);
}

public void hubEndingSoon() {
    // Purple = 5 second warning before OUR hub ends
    candle.setLEDs(160, 0, 255);
}

public void transition() {
    // Blue during transition
    candle.setLEDs(0, 0, 255);
}

public void disabled() {
    // Dim white when disabled
    candle.setLEDs(40, 40, 40);
}
}
