// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Vision;

public class ShuffleBoardButtons {
    ShuffleboardTab m_smartdashboard;

    GenericEntry NavxAngle;
    GenericEntry EncoderPosX;
    GenericEntry EncoderPosY;

    GenericEntry AprilTagAngle;
    GenericEntry AprilTagX;
    GenericEntry AprilTagY;

    
    GenericEntry PoseEstimatorAngle;
    GenericEntry PoseEstimatorX;
    GenericEntry PoseEstimatorY;
    GenericEntry fodState;

    GenericEntry grabberCurrentGraph;
    GenericEntry telescopeCurrentGraph;


    ShuffleboardTab m_encoderTab;
    GenericEntry BoreEncoders;
    GenericEntry pivotEntry;
    GenericEntry telescopeEntry;
    GenericEntry telescopeEncoderRate;




    

    public void initButtons(){
        m_smartdashboard = Shuffleboard.getTab("SmartDashboard");
        NavxAngle = m_smartdashboard.add("NavX angle", 0.0).getEntry();
        EncoderPosX = m_smartdashboard.add("Drivetrain Encoder X Position", 0).getEntry();
        EncoderPosY = m_smartdashboard.add("Drivetrain Encoder Y Position", 0).getEntry();
       
        AprilTagAngle = m_smartdashboard.add("TagPose Angle", 0.0).getEntry();
        AprilTagX = m_smartdashboard.add("TagPose X", 0).getEntry();
        AprilTagY = m_smartdashboard.add("TagPose Y", 0).getEntry();

        PoseEstimatorAngle = m_smartdashboard.add("PoseEstimator Angle", 0.0).getEntry();
        PoseEstimatorX = m_smartdashboard.add("PoseEstimator X", 0.0).getEntry();
        PoseEstimatorY = m_smartdashboard.add("PoseEstimator Y", 0.0).getEntry();


        fodState = m_smartdashboard.add("FOD state?", false).getEntry();

        m_encoderTab = Shuffleboard.getTab("Encoder");
        pivotEntry = m_encoderTab.add("Pivot Encoder", 0.0).getEntry();
        telescopeEntry = m_encoderTab.add("Telescope Encoder", 0.0).getEntry();
        BoreEncoders = m_encoderTab.add("Bore Encoder",0.0).getEntry();
        telescopeEncoderRate = m_encoderTab.add("Telescope Encoder Rate", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();

        grabberCurrentGraph = m_smartdashboard.add("Grabber Current vs Time", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
        telescopeCurrentGraph = m_smartdashboard.add("Telescope Current vs Time", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
    }

    public void updateButtons(){
        Pose2d vision = Vision.getVisionInstance().getTagPose();
        Drivetrain drivetrain = Drivetrain.getInstance();
        Pose2d estimator = PoseEstimator.getPoseEstimatorInstance().getCurrentPose();
        Grabber grabber = Grabber.getInstance();
        Telescope telescope = Telescope.getInstance();
        Arm arm = Arm.getInstance();
        
        NavxAngle.setDouble(drivetrain.getPose().getRotation().getDegrees());
        EncoderPosX.setDouble(drivetrain.getPose().getX());
        EncoderPosY.setDouble(drivetrain.getPose().getY());    

        AprilTagX.setDouble(vision.getX());
        AprilTagY.setDouble(vision.getY());
        AprilTagAngle.setDouble(vision.getRotation().getDegrees());  
        
        PoseEstimatorAngle.setDouble(estimator.getRotation().getDegrees());
        PoseEstimatorX.setDouble(estimator.getX());
        PoseEstimatorY.setDouble(estimator.getY());

        fodState.setBoolean(drivetrain.getFodState());

        pivotEntry.setDouble(arm.getPivotEncoder());
        telescopeEntry.setDouble(telescope.getTeleEncoder());
        BoreEncoders.setDouble(grabber.getGrabEncoder());
        grabberCurrentGraph.setDouble(grabber.getCurrent());
        telescopeCurrentGraph.setDouble(telescope.getCurrent());
        telescopeEncoderRate.setDouble(telescope.getTeleEncoderRate());

    }
    
}