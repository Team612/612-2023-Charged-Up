// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.SparkPorts;
import frc.robot.commands.StaticPivot;

public class Arm extends SubsystemBase {
  private CANSparkMax pivot;
  static Arm instance = null;


  public void initDefaultCommand() { //aka when nothing is running
    // Set the default command for a subsystem here.
    Constants.EncoderConstants.staticValue = getPivotEncoder();
    setDefaultCommand(new StaticPivot(getInstance()));
}
  /** Creates a new Arm. */
  public Arm() {
    pivot = new CANSparkMax(SparkPorts.pivotID, MotorType.kBrushless);
    pivot.setIdleMode(IdleMode.kBrake);
  }

  
  //rename method
  public void rotatePivot(double rotate) {
    pivot.set(rotate);
  }

  public double getPivotEncoder() {
    return pivot.getEncoder().getPosition();
  }

  public Boolean withinThresh(){
    if(getPivotEncoder() > EncoderConstants.arm_lower && getPivotEncoder() < EncoderConstants.arm_upper){
      return true;
    }
    return false;
  }

  public double getVoltage(){
    return pivot.getBusVoltage();
  }

  public static Arm getInstance(){
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
