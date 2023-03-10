// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.controls.ControlMap;
import frc.robot.subsystems.Arm;

public class Pivot extends CommandBase {
  private final Arm m_arm;
 
  /** Creates a new Pivot. */
  public Pivot(Arm arm) {
    m_arm = arm;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  public boolean isGoingUp(){
    if(ControlMap.gunner.getRawAxis(1) < 0) return true;
    return false;
  }

  // Called every time the scheduler runs while the command is scheduled.

  /*
   * 
   * If rawAxis 1 gives us negative --> we want to go up state
   * If rawAxis 1 gives up postiive --> we want to go down  state
   */
  @Override
  public void execute() {
    // if(m_arm.withinThresh() || (!(EncoderConstants.arm_upper == 0.0 && EncoderConstants.arm_lower == 0.0))){ //test
    //   m_arm.rotatePivot(ControlMap.gunner.getRawAxis(1) * MotorSpeeds.pivot_speed);

    // }
    // else if((isGoingUp() && m_arm.getPivotEncoder() > EncoderConstants.arm_lower) || (!isGoingUp() && m_arm.getPivotEncoder() < EncoderConstants.arm_upper)){
    //   m_arm.rotatePivot(ControlMap.gunner.getRawAxis(1) * MotorSpeeds.pivot_speed);

    // }
    // else{
    //   m_arm.rotatePivot(0);
    // }
    m_arm.rotatePivot(ControlMap.gunner.getRawAxis(1) * MotorSpeeds.pivot_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.rotatePivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
