// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */

public class SwerveModule {
    private CANSparkMax sModule;
    private int moduleNumber;
    private double velocity;

    public SwerveModule(int mn, int port, MotorType t){
        moduleNumber = mn;
        sModule = new CANSparkMax(port, t);
    }

    public void setDesiredState(SwerveModuleState state){
        setSpeed(state);
    }

    public void setSpeed(SwerveModuleState state){
        velocity = state.speedMetersPerSecond / Constants.SwerveConstants.SwerveMaxSpeed;
        sModule.set(velocity); //.set is a percentage of speed
    }

    public void setAngle(){

    }

    public int getModuleNumber(){
        return moduleNumber;
    }
}
