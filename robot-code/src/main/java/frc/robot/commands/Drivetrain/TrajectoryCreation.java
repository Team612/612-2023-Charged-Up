
package frc.robot.commands.Drivetrain;

import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants  ;
import frc.robot.subsystems.Vision;

/** Add your docs here. */
public class TrajectoryCreation {

    public TrajectoryConfig config = new TrajectoryConfig(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq)
        .setKinematics(Constants.DrivetrainConstants.kDriveKinematics);

    public Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(2,0), new Translation2d(2,-2), new Translation2d(0, -2)),
        new Pose2d(0,0, new Rotation2d(0)), 
        config); 
    
    public Trajectory meterForward = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(0.5, 0)),
        new Pose2d(1,0, new Rotation2d(0)),
        config);

    public Trajectory meterForwardY = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(0, 0.5)),
        new Pose2d(0,1, new Rotation2d(0)),
        config);
    
    public Trajectory return_Trajectory(PhotonCamera camera, Vision m_vision, Pose3d finalPose){
        if (camera.getLatestResult().hasTargets()){
           
            Pose3d initialPose = m_vision.return_camera_pose_tag(camera.getLatestResult().getBestTarget().getFiducialId(), camera.getLatestResult());
            
            return TrajectoryGenerator.generateTrajectory(
                new Pose2d(initialPose.getX(), initialPose.getY(), new Rotation2d(initialPose.getZ())), 
                List.of(new Translation2d( initialPose.getX() + ((finalPose.getX() - initialPose.getX())/2), initialPose.getY() + (finalPose.getY() - initialPose.getY())/2)),
                new Pose2d(finalPose.getX(), finalPose.getY(), new Rotation2d(finalPose.getZ())),
                config
            );
        }
        else{ 
            System.out.println("doesn't work, Arjun sucks");
            return TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(-0.5,0)),new Pose2d(-1,0, new Rotation2d(0)), config);
        }
    }
}