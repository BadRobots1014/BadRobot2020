package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLED;


public class LEDSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  public LEDSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }
}
