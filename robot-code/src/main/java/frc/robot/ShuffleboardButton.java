// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Grabber;

/** Add your docs here. */
public class ShuffleboardButton {
    ShuffleboardTab m_encoderTab;
    GenericEntry BoreEncoders;
    GenericEntry pivotEntry;
    GenericEntry telescopeEntry;

    public void initButtons(){
        m_encoderTab = Shuffleboard.getTab("Encoder");
        pivotEntry = m_encoderTab.add("Pivot Encoder", 0.0).getEntry();
        telescopeEntry = m_encoderTab.add("Telescope Encoder", 0.0).getEntry();
        BoreEncoders = m_encoderTab.add("Bore Encoder",0.0).getEntry();

    }

    public void updateButtons(){
        pivotEntry.setDouble(Arm.getInstance().getPivotEncoder());
        telescopeEntry.setDouble(Telescope.getInstance().getTeleEncoder());
        BoreEncoders.setDouble(Grabber.getInstance().getGrabEncoder());
    }
}


