package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLED;


public class LEDSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private AddressableLED m_LED;
  private AddressableLEDBuffer m_LEDBuffer;
  private int m_rainbowFirstPixelHue;
  private int m_blinkCounter;
  public enum LEDState
	{
		kRED, kBLUE, k1014COLOR, kRAINBOW, kLOW_BATTERY, kOFF;
	}

  public LEDSubsystem(AddressableLED LED, AddressableLEDBuffer LEDBuffer) {
    m_LED = LED;
    m_LEDBuffer = LEDBuffer;
    
    m_LED.setLength(m_LEDBuffer.getLength());

    // Set the data
    m_LED.setData(m_LEDBuffer);
    m_LED.start();
  }

  public void setLightsRGB(int red, int green, int blue) {
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values
      m_LEDBuffer.setRGB(i, red, green, blue);
   }
   
   m_LED.setData(m_LEDBuffer);
  }

  public void setLightsHSV(int hue, int saturation, int value) {
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      // Sets the specified LED to the HSV values
      m_LEDBuffer.setRGB(i, hue, saturation, value);
   }
   
   m_LED.setData(m_LEDBuffer);
  }

  public void setLightsRainbow() { //should be put inside loop
    // For every pixel
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LEDBuffer.getLength())) % 180;
      // Set the value
      m_LEDBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  public void setLightsBlink(int red, int green, int blue) { //must be put inside loop
    m_blinkCounter++;
    if (m_blinkCounter < 20 && m_blinkCounter >= 0) setLightsRGB(red, green, blue);

    else if (m_blinkCounter < 40) setLightsPattern(LEDState.kOFF);

    else m_blinkCounter = 0;
  }

  public void setLightsPattern(LEDState state) {
    switch (state) {
      case kRED:
        setLightsRGB(255, 0, 0);
        break;
      case kBLUE:
        setLightsRGB(0, 0, 255);
        break;
      case k1014COLOR:
        setLightsRGB(255, 255, 0); //just a regular yellow
        break;
      case kOFF:
        setLightsRGB(255, 255, 255); //Does this turn off?
        break;
      case kLOW_BATTERY:
        setLightsBlink(255, 0, 0);
        break;
      case kRAINBOW:
        setLightsRainbow();
        break;
    }
  }
}
