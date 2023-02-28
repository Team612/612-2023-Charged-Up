// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Pivot;

public class Arm extends SubsystemBase {
  private CANSparkMax pivot;

  //private WPI_TalonSRX talon1;
  //private WPI_TalonSRX talon2;
  //private WPI_TalonSRX talon3;

  private double currPivot1 = 0;
  private double currPivot2 = 0;
  DutyCycleEncoder boreEncoderArm;
  static Arm instance = null;

  /** Creates a new Arm. */
  public Arm() {
    pivot = new CANSparkMax(6, MotorType.kBrushless);
    boreEncoderArm = new DutyCycleEncoder(Constants.boreEncoderIntake);
    System.out.println(pivot);
    
    
    //talon1 = new WPI_TalonSRX(0);
    //talon2 = new WPI_TalonSRX(1);
    //talon3 = new WPI_TalonSRX(2);
  }
  
  //rename method
 
  public void rotatePivot(double rotate) {
    pivot.set(rotate);
    currPivot1 = rotate;
  }
 
  public boolean ifBorePassesLimit() {
    if (boreEncoderArm.getDistance() > 1 || boreEncoderArm.getDistance() < -1)
      return true;
    return false;
  }

  public double getPivotEncoder() {
    return pivot.getEncoder().getPosition();
  }

  public static Arm getInstance(){
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
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
