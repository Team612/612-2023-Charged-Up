// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private static AprilTagFieldLayout aprilTagFieldLayout;
  private static Transform3d robotToCam;
  private static PhotonPoseEstimator m_PoseEstimator;

  /** Creates a new Vision. 
   * @throws IOException **/
  public Vision(PhotonCamera camera) {
    //tag 1
    final Translation3d translation1 = new Translation3d(15.513558, 1.071626, 0.462788);
    final Quaternion q1 = new Quaternion(0, 0, 0, 1);
    final Rotation3d rotation1 = new Rotation3d(q1);
    final AprilTag tag1 = new AprilTag(1, new Pose3d(translation1, rotation1));

    //tag 2
    final Translation3d translation2 = new Translation3d(15.513558, 2.748026, 0.462788);
    final Quaternion q2 = new Quaternion(0, 0, 0, 1);
    final Rotation3d rotation2 = new Rotation3d(q2);
    final AprilTag tag2 = new AprilTag(2, new Pose3d(translation2, rotation2));

    //tag 3
    final Translation3d translation3 = new Translation3d(15.513558, 3.738626, 0.462788);
    final Quaternion q3 = new Quaternion(0,0,0,1);
    final Rotation3d rotation3 = new Rotation3d(q3);
    final AprilTag tag3 = new AprilTag(3, new Pose3d(translation3, rotation3));

    //tag 4
    final Translation3d translation4 = new Translation3d(16.178784, 6.749796, 0.695452);
    final Quaternion q4 = new Quaternion(0,0,0,1);
    final Rotation3d rotation4 = new Rotation3d(q4);
    final AprilTag tag4 = new AprilTag(4, new Pose3d(translation4, rotation4));

    //tag 5
    final Translation3d translation5 = new Translation3d(0.36195, 6.749796, 0.695452);
    final Quaternion q5 = new Quaternion(1, 0, 0, 0);
    final Rotation3d rotation5 = new Rotation3d(q5);
    final AprilTag tag5 = new AprilTag(5, new Pose3d(translation5, rotation5));

    //tag 6
    final Translation3d translation6 = new Translation3d(1.02743, 3.738626, 0.462788);
    final Quaternion q6 = new Quaternion(1, 0, 0, 0);
    final Rotation3d rotation6 = new Rotation3d(q6);
    final AprilTag tag6 = new AprilTag(6, new Pose3d(translation6, rotation6));

    //tag 7
    final Translation3d translation7 = new Translation3d(1.02743, 2.748026, 0.462788);
    final Quaternion q7 = new Quaternion(1, 0, 0, 0);
    final Rotation3d rotation7 = new Rotation3d(q7);
    final AprilTag tag7 = new AprilTag(7, new Pose3d(translation7, rotation7));

    //tag 8
    final Translation3d translation8 = new Translation3d(1.02743, 1.071626, 0.462788);
    final Quaternion q8 = new Quaternion(1, 0, 0, 0);
    final Rotation3d rotation8 = new Rotation3d(q8);
    final AprilTag tag8 = new AprilTag(8, new Pose3d(translation8, rotation8));
    ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
    atList.add(tag1);
    atList.add(tag2);
    atList.add(tag3);
    atList.add(tag4);
    atList.add(tag5);
    atList.add(tag6);
    atList.add(tag7);
    atList.add(tag8);
    aprilTagFieldLayout = new AprilTagFieldLayout(atList, 16.4592, 8.2296);
    robotToCam = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d());
    m_PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
  }

  //return tag pose
  public Pose3d return_tag_pose(int id){
    Optional<Pose3d> pose_of_tag = aprilTagFieldLayout.getTagPose(id);
    return pose_of_tag.get();
  }

  //self calculations
  public Pose3d return_camera_pose_tag(int id, PhotonPipelineResult results){
    System.out.println("works");
    Optional<Pose3d> pose_of_tag = aprilTagFieldLayout.getTagPose(id);
    Pose3d tag_pose = pose_of_tag.get();
    Transform3d cameraTransform = results.getBestTarget().getBestCameraToTarget();
    return tag_pose.plus(cameraTransform);
  }

  //photonvision pose estimator
  public static Optional<EstimatedRobotPose> return_photon_pose(Pose2d latestPose){
    m_PoseEstimator.setReferencePose(latestPose);
    return m_PoseEstimator.update();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
