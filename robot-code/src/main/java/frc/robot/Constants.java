// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

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
    //multipler for slowmo
    public static double slowmo = 1;
     //wheel diameter
     public static final double kWheelDiameterMeters = 0; 
     
     //Distance between centers of right and left wheels on robot
     public static final double kTrackWidth = 0; 
 
     //Distance between centers of front and back wheels on robot
     public static final double kWheelBase = 0; 
 
 
     public static final double kEncoderCPR = 0; 
     public static final double kGearReduction = 0;
     
     //Finding Distance per pulse
     public static final double kEncoderDistancePerPulse =
         ((kWheelDiameterMeters * Math.PI)) / (kGearReduction);
  
     //Feedforward gains for system dynamics 
     public static final double kS = 0.11858; 
     public static final double kV = 4.2016;  
     public static final double kA = 0.32842;
     
     //Angular gains
     public static final double kV_Angular = 1; // do not touch
     public static final double kA_Angular = 1; // do not touch
    //thresholds
    public static final int offbalancepositive = 15;
    public static final int offbalancepositivehalf = 5; //might need to change later
    public static final int offbalancepositivehalfneg = -5;
     
     //position controllers
 
     //have to tune manually
     public static final double kPXController = .0165;
     public static final double kPYController = .15;
     public static final double kPThetaController = .5; 
 
     
     //Velocity controllers
     public static final double kPFrontLeftVel = 4.6504; 
     public static final double kPRearLeftVel = 4.6504;
     public static final double kPFrontRightVel = 4.6504;
     public static final double kPRearRightVel = 4.6504;
 
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
    public static int kGunnerControllerPort = 1;
  }

  public static class SparkPorts {
    public static final int pivotID = 6;
    public static final int tele_arm = 7;
    public static final int grabber = 5;
  }

  public static class MotorSpeeds {
    public static final double pivot_speed = 1;
    public static final double tele_arm_speed = 0.5;
    public static double grabber_speed = 0.3;
  }

  public static class EncoderConstants{
    public static int boreEncoderIntake = 1;
    public static double staticValue = 0;
    //all of these are subject to change
    public static double arm_lower; // = -99.0;
    public static double arm_upper; // = 130.0;
    public static double tele_in; // = -3.0;
    public static double tele_out; // = 84.0;
    public static double tele_extension_rate = 5.0;
    public static double tele_extension_rate_thresh = 1.0;
    public static double tele_motor_voltage = 5.0;
    public static double tele_motor_voltage_thresh = 5.0;
 
  }
  public static String cameraName = "Microsoft_LifeCam_HD-3000";
}
