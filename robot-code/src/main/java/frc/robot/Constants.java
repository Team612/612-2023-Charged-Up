// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DrivetrainConstants{
     //Spark constants
     public final static int SPARK_FL = 2;
     public final static int SPARK_FR = 1;
     public final static int SPARK_BL = 4;
     public final static int SPARK_BR = 3;
 
     //wheel diameter
     public static final double kWheelDiameterMeters = 0.1524; 
     
     //Distance between centers of right and left wheels on robot
     public static final double kTrackWidth = 0.5969; // for phoenix 0.668655
 
     //Distance between centers of front and back wheels on robot
     public static final double kWheelBase = 0.676275; // for phoenix .5334
 
 
     public static final double kEncoderCPR = 1; 
     public static final double kGearReduction = 16;
     
     //Finding Distance per pulse
     public static final double kEncoderDistancePerPulse =
         ((kWheelDiameterMeters * Math.PI)) / (kGearReduction);
  
     //Feedforward gains for system dynamics 
     public static final double kS = 0.12141; 
     public static final double kV = 4.1481;  
     public static final double kA = 0.58895;
    

     //Angular gains
     public static final double kV_Angular = 4.218; // do not touch
     public static final double kA_Angular = 0.15213; // do not touch
 
     //position controllers

     //have to tune manually
     public static final double kPXController = 1; // ~ 1cm error
     public static final double kPYController = 1;
     public static final double kPThetaController = 4.5; 
 
     
    //Velocity controllers

    private static double x = 5.3755;

     public static final double kPFrontLeftVel = x; //0.34004 //5.3755
     public static final double kPRearLeftVel = x;
     public static final double kPFrontRightVel = x;
     public static final double kPRearRightVel = x;

 
     //Converting chassis velocity into individual wheel velocities
     public static final MecanumDriveKinematics kDriveKinematics =
         new MecanumDriveKinematics(
             new Translation2d(kWheelBase / 2, kTrackWidth / 2),
             new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
             new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
             new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
     );
     
     //trajectory constraints
     public static final double kMaxVelocityMetersPerSecond = 2.5;
     public static final double maxAccelerationMetersPerSecondSq = 1.0;

     public static final double kMaxAngularVelocity = Math.PI;
     public static final double kMaxAngularAcceleration = Math.PI;
 
     //angular constraints
     public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
         new TrapezoidProfile.Constraints(kMaxAngularVelocity, kMaxAngularAcceleration);
 
     //Feedforward 
     public static final SimpleMotorFeedforward kFeedforward =
         new SimpleMotorFeedforward(Constants.DrivetrainConstants.kS, Constants.DrivetrainConstants.kV, Constants.DrivetrainConstants.kA);
     
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class VisionConstants{
    public static String cameraName = "Microsoft_LifeCam_HD-3000";

    //constraints
    public static final TrapezoidProfile.Constraints ThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    public static final TrapezoidProfile.Constraints PControllerConstraints =
        new TrapezoidProfile.Constraints(1.5,1.5);
    public static final TrapezoidProfile.Constraints StrafeControllerConstaints = 
        new TrapezoidProfile.Constraints(1.5, 1.5);
    
    //controllers
    public static final ProfiledPIDController rotationController = 
      new ProfiledPIDController(.01, 0, 0, ThetaControllerConstraints);
    public static final ProfiledPIDController forwardController = 
      new ProfiledPIDController(.5, 0, 0, PControllerConstraints);
    public static final ProfiledPIDController strafeController = 
      new ProfiledPIDController(.4, 0, 0, StrafeControllerConstaints);

    //other camera constants
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(13.5);
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(14.25);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(12);
    public static final double GOAL_RANGE_METERS = 1;
    
    public static final Transform3d CAMERA_TO_Robot = new Transform3d();
  }
}
