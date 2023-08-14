// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
public class Swerve extends SubsystemBase {
  private Translation2d fl;
  private Translation2d fr;
  private Translation2d bl;
  private Translation2d br;
  private SwerveDriveKinematics swerve_kinemtics;
  /** Creates a new Swerve. */
  public Swerve(Translation2d fl,Translation2d fr,Translation2d bl,Translation2d br) {
    this.fl = fl;
    this.fr = fr;
    this.bl = bl;
    this.br = br;
    swerve_kinemtics = new SwerveDriveKinematics(this.fl,this.fr,this.bl,this.br);
  }


  public void drive(double x, double y, double angle){ //Swerve comes with its own code for field relative pos
    ChassisSpeeds speed = new ChassisSpeeds(x,y,angle);
    SwerveModuleState[] moduleStates = swerve_kinemtics.toSwerveModuleStates(speed);
    //swerve_kinemtics.desaturateWheelSpeeds(moduleStates, speed, x, x, y);
   


  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
