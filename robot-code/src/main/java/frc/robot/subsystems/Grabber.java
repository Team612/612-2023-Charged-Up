// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.SparkPorts;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  private CANSparkMax grabber;
  static Grabber instance = null;
  private final DutyCycleEncoder boreEncoderArm;

  public Grabber() {
    grabber = new CANSparkMax(SparkPorts.grabber, MotorType.kBrushless);
    boreEncoderArm = new DutyCycleEncoder(EncoderConstants.boreEncoderIntake);
  }

  public void grab(double rotate) {
    grabber.set(rotate);
  }

  public double getGrabEncoder() {
    return boreEncoderArm.getDistance();
  }

  public static Grabber getInstance(){
    if (instance == null) {
      instance = new Grabber();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
