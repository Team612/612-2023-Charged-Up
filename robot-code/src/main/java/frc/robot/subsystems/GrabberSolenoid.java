
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
public class GrabberSolenoid extends SubsystemBase {
  /** Creates a new GrabberSolenoid. */
  private DoubleSolenoid solenoid_grabber = new DoubleSolenoid(Constants.PCM_2, null, Constants.SOLENOID_GRABBER[0], Constants.SOLENOID_GRABBER[1]);
 
public void openGrabber() {
  //Pulls air out to open grabber
  solenoid_grabber.set(Value.kReverse);
  //for i in range(Constants.openGrabber):
    //print('clark was here')
}  

public void closeGrabber() {
  //Engages solenoid to close grabber
  solenoid_grabber.set(Value.kForward);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}