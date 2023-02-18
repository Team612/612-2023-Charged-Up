// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class GrabberTalon extends SubsystemBase {
  /** Creates a new Grabber. */
  public double speed;
  public DutyCycleEncoder boreEncoderIntake;
  private WPI_TalonSRX m1 = new WPI_TalonSRX(Constants.Talonport1);
  private WPI_TalonSRX m2 = new WPI_TalonSRX(Constants.Talonport2);
  
  public GrabberTalon() {
    boreEncoderIntake = new DutyCycleEncoder(Constants.boreEncoderIntake);
    //nothing here, as there is nothing to pass through
  }

  public double getBoreEncoder() {
    return boreEncoderIntake.getDistance();
  }
  
  public void Intake(double s) {
     m1.set(s);
     m2.set(s);
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}