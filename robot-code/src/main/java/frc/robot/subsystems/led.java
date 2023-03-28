// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
public class led extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private Timer timer;
  /** Creates a new led. */
  public led() {
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(300); //300 should be the length
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start(); //starts the leds
     timer = new Timer();
  }

  public int getLength(){
    return m_ledBuffer.getLength();
  }
  //-------------------------------------THEMES----------------------------------------------------------//
  public void resetled(){ //turns all the leds off
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
   }
    m_led.setData(m_ledBuffer);
      }

  public void setLed(int r, int g, int b){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setLedSpecific(int i, int r, int g, int b) {
    m_ledBuffer.setRGB(i,r,g,b);
    m_led.setData(m_ledBuffer);
  }

  public void yellow(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 255, 0);
   }
    m_led.setData(m_ledBuffer);
  }

  public void purple(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 0, 255);
   }
    m_led.setData(m_ledBuffer);
  }
  
  //chantilly theme
  public void ChantillyTheme(){ //Chantilly color pattern; purple and white
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      if(i % 2 == 0){
        m_ledBuffer.setHSV(i, 136, 0, 60);
      } else {
        m_ledBuffer.setRGB(i, 75, 0, 130);
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public void SixTwelveTheme(){ //612 color pattern; blue, yellow, and white
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      if(i % 3 == 0){
        m_ledBuffer.setHSV(i, 136, 0, 60);
      } else if(i % 3 == 1){
        m_ledBuffer.setRGB(i, 255, 0, 255);
      }else{
        m_ledBuffer.setRGB(i, 0, 0, 255);
      }
    }
    m_led.setData(m_ledBuffer);
  }
  
  //yellow sparkle
  

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}