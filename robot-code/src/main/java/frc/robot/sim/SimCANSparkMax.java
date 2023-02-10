// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.RobotBase;


/** Add your docs here. */
//SIMULATION TIME !!!! :3
// In Team 612 we are safe as one. We will not tolerate laziness.

// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612 we are safe as one. We will not tolerate laziness.
// In Team 612, We brighten the future for our leaders and student technologists within our team, school, and community through fun and engaging methods of learning and teaching while shining a light on the importance of STEAM.
// We don't use kids to build robots, we use robots to build kids.


public class SimCANSparkMax extends CANSparkMax {
    private double m_setpoint = 0.0;
    public SimCANSparkMax(int deviceID, MotorType type){
        super(deviceID, type);
    }

    @Override
    public void set(double speed){
        if(RobotBase.isSimulation()){
            this.m_setpoint = speed;
          }
          else{
            super.set(speed);
          }
    }

    @Override
    public double get(){
        if(RobotBase.isSimulation()){
            return this.m_setpoint;
          }
          else{
            return super.get();
          }
    }
} 