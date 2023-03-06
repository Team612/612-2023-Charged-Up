// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Telescope extends SubsystemBase {
  private CANSparkMax telescope;
  private double currTelescope = 0.0;
  DutyCycleEncoder boreEncoderArm;


  /** Creates a new Telescope. */
  public Telescope() {
    telescope = new CANSparkMax(7, MotorType.kBrushless);
    //boreEncoderArm = new DutyCycleEncoder(Constants.boreEncoderIntake);
  }
  
  public void rotateTelescope(double rotate) {
    telescope.set(rotate);
    currTelescope = rotate;
  }
  public boolean telescopeCondition() {
    return currTelescope > 0;
  }
  /* 
  public boolean ifBorePassesLimit() {
    if (boreEncoderArm.getDistance() > 1 || boreEncoderArm.getDistance() < -1)
      return true;
    return false;
  }
  */

  public double getTelescopeEncoder() {
    return telescope.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
