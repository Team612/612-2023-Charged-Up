// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Drivetrain;

public class ShuffleBoardButtons {
    ShuffleboardTab m_smartdashboard;
    GenericEntry NavxAngle;

    public void initButtons(){
        m_smartdashboard = Shuffleboard.getTab("SmartDashboard");
        NavxAngle = m_smartdashboard.add("navx angle", 0.0).getEntry();
    }

    public void updateButtons(){
        NavxAngle.setDouble(Drivetrain.getInstance().getNavxAngle().getDegrees());
        
    }
    
}