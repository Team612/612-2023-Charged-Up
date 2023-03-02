// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Trapezoid extends SubsystemBase {
  /** Creates a new Trapazoid. */
  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State state;
  private TrapezoidProfile.State end;
  public Trapezoid(double[] c, double[] e, double[] s) {
    constraints = new TrapezoidProfile.Constraints(c[0], c[1]);
    state = new TrapezoidProfile.State(s[0], s[1]); //position in meters
    end = new TrapezoidProfile.State(e[0],e[1]); //position in meters
  }
  //creates a trapazoid profile. 
  public TrapezoidProfile createProfile(){
    TrapezoidProfile profile = new TrapezoidProfile(constraints,end,state);
    return profile;
  }
  //Calculates what velocity the robot needs to go at in order to reach it's goal at a certain time.
  public TrapezoidProfile.State calculate(TrapezoidProfile p, double t) {
    TrapezoidProfile.State result = p.calculate(t);
    return result;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
