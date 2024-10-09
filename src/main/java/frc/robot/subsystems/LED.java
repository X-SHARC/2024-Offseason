// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;
  int[] rgb = new int[60];
  private int blinkFrequency = 10;
  private int beginning;
  int j;

  /** Creates a new LED. */
  public LED(int dataPort, int ledLength) {
    m_led = new AddressableLED(dataPort);
    m_ledBuffer = new AddressableLEDBuffer(ledLength);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
  }

  public void setColor(int r, int g, int b) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
  }

  public void blink(int r, int g, int b) {
    if (blinkFrequency != 0) {
      if (j % blinkFrequency == 0) {
        setColor(0, 0, 0);
        j = 1;
      } else if (j % (blinkFrequency / 2) == 0) {
        setColor(r, g, b);
      }
      j++;
    } else {
      setColor(r, g, b);
    }
  }

  public void sliding(Color color) {
    for (var i = 30; i < m_ledBuffer.getLength(); i++) {
      if (i >= beginning && i <= beginning + 8) {
        m_ledBuffer.setLED(i, color);
        m_ledBuffer.setLED(60 - i, color);
      } else {
        m_ledBuffer.setRGB(i, 0, 0, 0);
        m_ledBuffer.setRGB(60 - i, 0, 0, 0);
      }
    }
    m_led.setData(m_ledBuffer);
    beginning++;
    beginning %= 60;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
