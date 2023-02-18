// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private CANSparkMax pivot1;
  private CANSparkMax pivot2;
  private CANSparkMax telescope;

  //private WPI_TalonSRX talon1;
  //private WPI_TalonSRX talon2;
  //private WPI_TalonSRX talon3;

  private double currTelescope = 0.0;
  private double currPivot1 = 0;
  private double currPivot2 = 0;
  DutyCycleEncoder boreEncoderArm;

  /** Creates a new Arm. */
  public Arm() {
    pivot1 = new CANSparkMax(0, MotorType.kBrushless);
    pivot2 = new CANSparkMax(1, MotorType.kBrushless);
    telescope = new CANSparkMax(2, MotorType.kBrushless);
    boreEncoderArm = new DutyCycleEncoder(Constants.boreEncoderIntake);
    
    
    //talon1 = new WPI_TalonSRX(0);
    //talon2 = new WPI_TalonSRX(1);
    //talon3 = new WPI_TalonSRX(2);
  }
  
  //rename method
  public void rotateTelescope(double rotate) {
    telescope.set(rotate);
    currTelescope = rotate;
  }
  public void rotatePivot1(double rotate) {
    pivot1.set(rotate);
    currPivot1 = rotate;
  }
  public void rotatePivot2(double rotate) {
    pivot2.set(rotate);
    currPivot2 = rotate;
  }
  public boolean telescopeCondition() {
    return currTelescope > 0;
  }
   
  public double getArmDistance() {
    return boreEncoderArm.getDistance();
  }

  /*public void setTalon1(double speed) {
    if(Math.abs(speed) < 0.1) 
      speed = 0;
    talon1.set(speed);
  }
  public void setTalon2(double speed) {
    if(Math.abs(speed) < 0.1) 
      speed = 0;
    talon2.set(speed);
  }
  public void setTalon3(double speed) {
    if(Math.abs(speed) < 0.1) 
      speed = 0;
    talon3.set(speed);
  } */


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
