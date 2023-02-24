// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class ShuffleBoardButtons {
    ShuffleboardTab m_smartdashboard;
    GenericEntry NavxAngle;
    GenericEntry EncoderPosX;
    GenericEntry EncoderPosY;

    GenericEntry AprilTagAngle;
    GenericEntry AprilTagX;
    GenericEntry AprilTagY;

    public void initButtons(){
        m_smartdashboard = Shuffleboard.getTab("SmartDashboard");
        NavxAngle = m_smartdashboard.add("navx angle", 0.0).getEntry();
        EncoderPosX = m_smartdashboard.add("Encoder X Position", 0).getEntry();
        EncoderPosY = m_smartdashboard.add("Encoder Y Position", 0).getEntry();
       
       
        AprilTagAngle = m_smartdashboard.add("TagPose Angle", 0.0).getEntry();
        AprilTagX = m_smartdashboard.add("TagPose X", 0).getEntry();
        AprilTagY = m_smartdashboard.add("TagPose Y", 0).getEntry();

    }

    public void updateButtons(){
        Vision vision = Vision.getVisionInstance();
        Pose2d tagPose = vision.getTagPose();
        Pose2d tagToRobotPose = vision.getRobotPose();

        Pose2d drivey = Drivetrain.getInstance().getPose();


        NavxAngle.setDouble(drivey.getRotation().getDegrees());
        EncoderPosX.setDouble(drivey.getX());
        EncoderPosY.setDouble(drivey.getY());    
        
        AprilTagX.setDouble(tagPose.getX());
        AprilTagY.setDouble(tagPose.getY());
        AprilTagAngle.setDouble(tagPose.getRotation().getDegrees());    
        
    }
    
}