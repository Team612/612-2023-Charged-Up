// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.SparkPorts;

public class Telescope extends SubsystemBase {
  private CANSparkMax telescope;
  private double currTelescope = 0.0;
  static Telescope instance = null;


  /** Creates a new Telescope. */
  public Telescope() {
    telescope = new CANSparkMax(SparkPorts.tele_arm, MotorType.kBrushless);
  }
  
  public void moveTelescope(double speed) {
    telescope.set(speed);
    currTelescope = speed;
  }
  public double getTeleEncoder() {
    return telescope.getEncoder().getPosition();
  }
  public boolean isRetracted() {
    return getTeleEncoder() <= EncoderConstants.tele_in;

  }

  public boolean isExtended(){
    return getTeleEncoder() >= EncoderConstants.tele_out;
  }
  public boolean telescopeCondition() {
    return currTelescope > 0;
  }

  public static Telescope getInstance(){
    if (instance == null) {
      instance = new Telescope();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
