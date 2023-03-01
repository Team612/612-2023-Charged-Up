// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

public class ShuffleBoardButtons {
    ShuffleboardTab m_smartdashboard;
    GenericEntry NavxAngle;

    GenericEntry AprilTagAngle;
    GenericEntry AprilTagX;
    GenericEntry AprilTagY;

    
    GenericEntry PoseEstimatorAngle;
    GenericEntry PoseEstimatorX;
    GenericEntry PoseEstimatorY;

    public void initButtons(){
        m_smartdashboard = Shuffleboard.getTab("SmartDashboard");
        NavxAngle = m_smartdashboard.add("NavX angle", 0.0).getEntry();

        AprilTagAngle = m_smartdashboard.add("TagPose Angle", 0.0).getEntry();
        AprilTagX = m_smartdashboard.add("TagPose X", 0).getEntry();
        AprilTagY = m_smartdashboard.add("TagPose Y", 0).getEntry();

        PoseEstimatorAngle = m_smartdashboard.add("PoseEstimator Angle", 0.0).getEntry();
        PoseEstimatorX = m_smartdashboard.add("PoseEstimator X", 0.0).getEntry();
        PoseEstimatorY = m_smartdashboard.add("PoseEstimator Y", 0.0).getEntry();

    }

    public void updateButtons(){
        Pose2d vision = Vision.getVisionInstance().getTagPose();
        Drivetrain drivetrain = Drivetrain.getInstance();
        Pose2d estimator = PoseEstimator.getPoseEstimatorInstance().getCurrentPose();
        
        NavxAngle.setDouble(drivetrain.getNavxAngle().getDegrees()); 

        AprilTagX.setDouble(vision.getX());
        AprilTagY.setDouble(vision.getY());
        AprilTagAngle.setDouble(vision.getRotation().getDegrees());  
        
        PoseEstimatorAngle.setDouble(estimator.getRotation().getDegrees());
        PoseEstimatorX.setDouble(estimator.getX());
        PoseEstimatorY.setDouble(estimator.getY());
    }
    
}