// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ZAlignWithTag extends CommandBase {
  private final Drivetrain m_Drivetrain;
  private final PhotonCamera cam;
  private double rotationSpeed = 0;
  private double yaw = 0;
  private PhotonPipelineResult result = new PhotonPipelineResult();
  /** Creates a new YAlignWithTag. */
  public ZAlignWithTag(Drivetrain d, PhotonCamera c) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Drivetrain = d;
    cam = c;
    addRequirements(m_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    result = cam.getLatestResult();
    if(result.hasTargets()){
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      yaw = bestTarget.getYaw();
      rotationSpeed = -Constants.VisionConstants.rotationController.calculate(yaw, 0);
      
    }
    else{
      rotationSpeed = 0;
    }

    m_Drivetrain.driveMecanum(0, 0, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.driveMecanum(0,0,0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(result.hasTargets()){
      if(yaw < 0.03 && yaw > -0.03){
        return true;
      } 
    }
    return false;
  }
}
