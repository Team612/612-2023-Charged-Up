// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class YAlignWithTag extends CommandBase {
  private final Drivetrain m_Drivetrain;
  private final PhotonCamera cam;
  private double strafeSpeed = 0;
  private double strafe = 0;
  private PhotonPipelineResult result = new PhotonPipelineResult();
  /** Creates a new YAlignWithTag. */
  public YAlignWithTag(Drivetrain d, PhotonCamera c) {
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
      
      Transform3d transformation = bestTarget.getBestCameraToTarget();
      strafe = transformation.getY();
      strafeSpeed = Constants.VisionConstants.strafeController.calculate(Units.metersToFeet(strafe), 0);
    }
    else{
      strafeSpeed = 0;
    }

    m_Drivetrain.driveMecanum(0, strafeSpeed, 0);
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
      if(strafe < 0.03 && strafe > -0.03){
        return true;
      } 
    }
    return false;
  }
}
