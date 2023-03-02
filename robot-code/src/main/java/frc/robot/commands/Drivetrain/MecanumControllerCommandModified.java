// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.subsystems.PoseEstimator;

public class MecanumControllerCommandModified extends MecanumControllerCommand{

  public MecanumControllerCommandModified(Trajectory trajectory, Supplier<Pose2d> pose,
      SimpleMotorFeedforward feedforward, MecanumDriveKinematics kinematics, PIDController xController,
      PIDController yController, ProfiledPIDController thetaController, double maxWheelVelocityMetersPerSecond,
      PIDController frontLeftController, PIDController rearLeftController, PIDController frontRightController,
      PIDController rearRightController, Supplier<MecanumDriveWheelSpeeds> currentWheelSpeeds,
      Consumer<MecanumDriveMotorVoltages> outputDriveVoltages, PoseEstimator estimator) {
    super(trajectory, pose, feedforward, kinematics, xController, yController, thetaController,
        maxWheelVelocityMetersPerSecond, frontLeftController, rearLeftController, frontRightController, rearRightController,
        currentWheelSpeeds, outputDriveVoltages, estimator);
    //TODO Auto-generated constructor stub
  }

  public boolean isFinished() {
    return false;
  }  
}
