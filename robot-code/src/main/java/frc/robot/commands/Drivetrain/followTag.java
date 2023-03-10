// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class followTag extends CommandBase {
  private final Drivetrain m_Drivetrain;
  private final PhotonCamera cam;
  private double forwardSpeed = 0;
  private double rotationSpeed = 0;
  private double strafeSpeed = 0;
  /** Creates a new followTag. */
  public followTag(Drivetrain d, PhotonCamera c) {
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
    PhotonPipelineResult result = cam.getLatestResult();
    if(result.hasTargets()){
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      Transform3d transform3d = bestTarget.getBestCameraToTarget();
        rotationSpeed = -Constants.VisionConstants.rotationController.calculate(bestTarget.getYaw(), 0);
      double range = PhotonUtils.calculateDistanceToTargetMeters(Constants.VisionConstants.CAMERA_HEIGHT_METERS, 
                   Constants.VisionConstants.TARGET_HEIGHT_METERS,
                   Constants.VisionConstants.CAMERA_PITCH_RADIANS,
                   Units.degreesToRadians(bestTarget.getPitch())); 
      double range2 = transform3d.getX();
      double strafe = transform3d.getY();
      // System.out.println(strafe);

      forwardSpeed = -Constants.VisionConstants.forwardController.calculate(range2, Constants.VisionConstants.GOAL_RANGE_METERS);
      strafeSpeed = Constants.VisionConstants.strafeController.calculate(Units.metersToFeet(strafe), 0);
    }
    else{
      forwardSpeed = 0;
      rotationSpeed = 0;
      strafeSpeed = 0;
    }
    m_Drivetrain.RobotOrientedDrive(forwardSpeed, strafeSpeed, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
