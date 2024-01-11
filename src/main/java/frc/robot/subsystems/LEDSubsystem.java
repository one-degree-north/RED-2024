package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private LEDState state = LEDState.STATIC;

  // Dynamic LED Mode Variables

  private int rainbowFirstPixelHue;

  private int flowDimPixelIndex;
  private int flowStartIndex;
  private int flowEndIndex;

  private int percentStartIndex;
  private int percentEndIndex;
  private Supplier<Double> percentSupp;

  private LEDState lastState = LEDState.STATIC;

  public LEDSubsystem(int port, int length) {

    m_led = new AddressableLED(port);
    m_ledBuffer = new AddressableLEDBuffer(length);

    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public LEDState getState() {
    return state;
  }

  public void rainbow() {
    if (lastState != LEDState.RAINBOW)
    state = LEDState.RAINBOW;
  }

  public void flow(int startIndex, int endIndex) {
    if (lastState != LEDState.FLOW){
      state = LEDState.FLOW;
      this.flowStartIndex = startIndex;
      this.flowEndIndex = endIndex;
      this.flowDimPixelIndex = flowStartIndex;
    }
  }

  public void percentage(Supplier<Double> percentage, int startIndex, int endIndex) {
    if (lastState != LEDState.PERCENTAGE){
      state = LEDState.PERCENTAGE;
      this.percentStartIndex = startIndex;
      this.percentEndIndex = endIndex;
      this.percentSupp = percentage;
    }
  }

  public void setAllStatic(Color color) {
    state = LEDState.STATIC;
    for (int i = 0; i < m_ledBuffer.getLength(); i++)
      m_ledBuffer.setLED(i, color);
    
    m_led.setData(m_ledBuffer);
  }

  public void setRangeStatic(Color color, int startIndex, int endIndex) {
    state = LEDState.STATIC;
    for (int i = startIndex; i <= endIndex; i++)
      m_ledBuffer.setLED(i, color);

    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    switch(state) {
      case RAINBOW:
        // For every pixel
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
        break;

      case FLOW:
        int rangeFlow = flowEndIndex - flowStartIndex + 1;
        int maxDistance = (rangeFlow / 2) + (rangeFlow % 2);

        for (int i=flowStartIndex; i<=flowEndIndex; i++) {
          int distanceRight = (i-flowDimPixelIndex + rangeFlow) % rangeFlow;
          int distanceLeft = (flowDimPixelIndex-i + rangeFlow) % rangeFlow;
          int distance = Math.min(distanceLeft, distanceRight);

          int intensity = (int) (((double)distance/(double)maxDistance) * 255.0);
          m_ledBuffer.setRGB(i, intensity, 0, 0);
        }

        flowDimPixelIndex += 1;
        if (flowDimPixelIndex > flowEndIndex) 
          flowDimPixelIndex = flowStartIndex;
        break;
      
      case PERCENTAGE:
        int rangePercent = percentEndIndex-percentStartIndex+1;
        int numberOfLeds = (int) (percentSupp.get() * rangePercent);
        for (int i=percentStartIndex; i < percentStartIndex+numberOfLeds; i++) {
          m_ledBuffer.setRGB(i, 255, 0, 0);
        }
        for (int i = percentStartIndex+numberOfLeds; i<=percentEndIndex; i++) {
          m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        break;

      default:
        break;
    }

    if (state != LEDState.STATIC) m_led.setData(m_ledBuffer);
    
    lastState = getState();
  }

  public enum LEDState {
    STATIC,
    RAINBOW,
    FLOW,
    PERCENTAGE
  }
}