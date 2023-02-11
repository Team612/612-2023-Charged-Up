// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class led extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  /** Creates a new led. */
  public led() {
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(300); //300 should be the length
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start(); //starts the leds
  }
//different LED settings for different occasions
  public void resetled(){ //turns all the leds off
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
      
   }
    m_led.setData(m_ledBuffer);
      }
  
  public void led1(){ //green
    System.out.println("here"); 
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 255, 0);
   }
    m_led.setData(m_ledBuffer);
  }

  public void led2(){ //red
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 0, 0);
   }
   m_led.setData(m_ledBuffer);
    
    
  }
  
  public void led3(int t){ //Red, green, blue cycle
    if (t%3 == 0) {
      m_ledBuffer.setRGB(t, 255, 0, 0);
    }
    else if (t%3 == 1) {
      m_ledBuffer.setRGB(t, 0, 255, 0);
    }
    else {
      m_ledBuffer.setRGB(t, 0, 0, 255);

    }
    m_led.setData(m_ledBuffer);
    
    
    m_led.start();
  }
  
  public void led4(int t, int r,int g, int b){ //rainbow gradient
   m_ledBuffer.setRGB(t,r,g,b);
   m_led.setData(m_ledBuffer);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
