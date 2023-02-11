// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
     public static final double kTrackWidth = 0.5969; 
 
     //Distance between centers of front and back wheels on robot
     public static final double kWheelBase = 0.676275; 
 
 
     public static final double kEncoderCPR = 1; 
     public static final double kGearReduction = 16;
     
     //Finding Distance per pulse
     public static final double kEncoderDistancePerPulse =
         ((kWheelDiameterMeters * Math.PI)) / (kGearReduction);
  
     //Feedforward gains for system dynamics 
    //  public static final double kS = 0.11858; 
    //  public static final double kV = 4.2016;  
    //  public static final double kA = 0.32842;
    

     public static final double kS = 0.096645; 
     public static final double kV = 4.1049;  
     public static final double kA = 1.1677;

     //Angular gains
     public static final double kV_Angular = 4.218; // do not touch
     public static final double kA_Angular = 0.15213; // do not touch
 
     //position controllers
 
     //have to tune manually
    //  public static final double kPXController = .03; // ~ 1cm error
    public static final double kPXController = .035; // ~ 1cm error

     public static final double kPYController = .6;
     public static final double kPThetaController = 4.5; 
 
     
    //  //Velocity controllers
    //  public static final double kPFrontLeftVel = 4.6504; 
    //  public static final double kPRearLeftVel = 4.6504;
    //  public static final double kPFrontRightVel = 4.6504;
    //  public static final double kPRearRightVel = 4.6504;
    public static final double kPFrontLeftVel = 0.74928; 
     public static final double kPRearLeftVel = 0.74928;
     public static final double kPFrontRightVel = 0.74928;
     public static final double kPRearRightVel = 0.74928;

 
     //Converting chassis velocity into individual wheel velocities
     public static final MecanumDriveKinematics kDriveKinematics =
         new MecanumDriveKinematics(
             new Translation2d(kWheelBase / 2, kTrackWidth / 2),
             new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
             new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
             new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
     );
     
     //trajectory constraints
     public static final int kMaxVelocityMetersPerSecond = 2;
     public static final int maxAccelerationMetersPerSecondSq = 1;
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
        new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    public static final TrapezoidProfile.Constraints StrafeControllerConstaints = 
        new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    
    //controllers
    public static final ProfiledPIDController rotationController = 
      new ProfiledPIDController(.01, 0, 0, ThetaControllerConstraints);
    public static final ProfiledPIDController forwardController = 
      new ProfiledPIDController(.1, 0, 0, PControllerConstraints);
    public static final ProfiledPIDController strafeController = 
      new ProfiledPIDController(.1, 0, 0, StrafeControllerConstaints);

    //other camera constants
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(13.5);
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(14.25);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(12);
    public static final double GOAL_RANGE_METERS = 1;
    
  }
}
