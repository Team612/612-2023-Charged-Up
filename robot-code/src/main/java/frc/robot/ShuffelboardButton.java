// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.Pivot;
import frc.robot.subsystems.Arm;

/** Add your docs here. */
public class ShuffelboardButton {
    ShuffleboardTab m_encoderTab;

    GenericEntry pivotEntry;

public void initButtons(){
m_encoderTab = Shuffleboard.getTab("Encoder");
pivotEntry = m_encoderTab.add("pivotEncoder",0.0).getEntry();

}

public void updateButtons(){
pivotEntry.setDouble(Arm.getInstance().getPivotEncoder());
}
}


