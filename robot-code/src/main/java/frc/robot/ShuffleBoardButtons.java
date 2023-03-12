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
public class ShuffleBoardButtons {
    ShuffleboardTab m_driverTab;
    ShuffleboardTab m_encoderTab;
    ShuffleboardTab m_graphTab;
    ShuffleboardTab m_debugTab;
    
    GenericEntry NavxAngle;
    GenericEntry PoseEstimatorAngle;
    GenericEntry PoseEstimatorX;
    GenericEntry PoseEstimatorY;
    GenericEntry fodState;
    GenericEntry grabberCurrentGraph;
    GenericEntry telescopeCurrentGraph;
    GenericEntry BoreEncoders;
    GenericEntry pivotEntry;
    GenericEntry telescopeEntry;
    GenericEntry telescopeEncoderRate;
    GenericEntry isGrabbing;
    GenericEntry isReleasing;

    //accessable entires
    public static GenericEntry grabberSpikeTresh;
    public static GenericEntry teleEncoderRateThresh;
    public static GenericEntry teleSpikeThresh;


    public void initButtons(){
        m_driverTab = Shuffleboard.getTab("DriverTab");
        m_encoderTab = Shuffleboard.getTab("Encoder");
        m_graphTab = Shuffleboard.getTab("Graphs");
        m_debugTab = Shuffleboard.getTab("Debug Tab");

        //debug entries

        NavxAngle = m_debugTab.add("NavX angle", 0.0).getEntry();
        PoseEstimatorAngle = m_debugTab.add("PoseEstimator Angle", 0.0).getEntry();
        PoseEstimatorX = m_debugTab.add("PoseEstimator X", 0.0).getEntry();
        PoseEstimatorY = m_debugTab.add("PoseEstimator Y", 0.0).getEntry();

        //smartdashboard entries

        fodState = m_driverTab.add("FOD state?", false).getEntry();
        isGrabbing = m_driverTab.add("isGrabbing", false).getEntry();
        isReleasing = m_driverTab.add("isReleasing", false).getEntry();


        // encoder entries
        pivotEntry = m_encoderTab.add("Pivot Encoder", 0.0).getEntry();
        BoreEncoders = m_encoderTab.add("Bore Encoder",0.0).getEntry();

        //graphing entries
        grabberSpikeTresh = m_graphTab.add("GrabberSpikeTresh",0.0).getEntry();
        grabberCurrentGraph = m_graphTab.add("Grabber Current vs Time", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
        telescopeCurrentGraph = m_graphTab.add("Telescope Current vs Time", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
        telescopeEncoderRate = m_graphTab.add("Telescope Encoder Rate", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
        telescopeEntry = m_graphTab.add("Telescope Encoder", 0.0).getEntry();
        teleEncoderRateThresh = m_graphTab.add("TeleEncoderThresh", 0.0).getEntry();
        teleSpikeThresh = m_graphTab.add("TeleSpikeThresh", 0.0).getEntry();


    }

    public void updateButtons(){
        Drivetrain drivetrain = Drivetrain.getInstance();
        Pose2d estimator = PoseEstimator.getPoseEstimatorInstance().getCurrentPose();
        Grabber grabber = Grabber.getInstance();
        Telescope telescope = Telescope.getInstance();
        Arm arm = Arm.getInstance();

        
        NavxAngle.setDouble(drivetrain.getPose().getRotation().getDegrees());
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
        isGrabbing.setBoolean(grabber.getBooleanGrabber());
        
    }
    
}