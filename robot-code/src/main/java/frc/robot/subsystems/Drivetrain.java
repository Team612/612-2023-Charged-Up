// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax spark_fl;
  private final CANSparkMax spark_fr;
  private final CANSparkMax spark_bl;
  private final CANSparkMax spark_br;

  static Drivetrain instance = null;
  
  private final double DEADZONE = 0.1;

  private MecanumDrive drivetrain;
  public double vel = Constants.DrivetrainConstants.kEncoderDistancePerPulse / 60; //velocity is in rpm so we need to get it into rps
 
  private static AHRS navx;
  private Field2d m_field;

  private Rotation2d navxAngleOffset;  

 
  MecanumDriveOdometry m_odometry;
  
  private Vision m_visionOdometry;
  
  

  public Drivetrain() {
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);
    spark_fl = new CANSparkMax(Constants.DrivetrainConstants.SPARK_FL, MotorType.kBrushless);

    spark_fr = new CANSparkMax(Constants.DrivetrainConstants.SPARK_FR, MotorType.kBrushless);
    spark_bl = new CANSparkMax(Constants.DrivetrainConstants.SPARK_BL, MotorType.kBrushless);
    spark_br = new CANSparkMax(Constants.DrivetrainConstants.SPARK_BR, MotorType.kBrushless);
    
    navx = new AHRS(I2C.Port.kMXP); //TO BE CHANGED WE DON'T KNOW THIS YET
    navxAngleOffset = new Rotation2d();

    m_odometry = new MecanumDriveOdometry(Constants.DrivetrainConstants.kDriveKinematics, navx.getRotation2d(), getMecanumDriveWheelPositions());
    m_visionOdometry = Vision.getVisionInstance();
    resetVisionOdometry();

    spark_fr.setInverted(true);
    spark_br.setInverted(true);

    spark_fl.setInverted(false);
    spark_bl.setInverted(false);

    spark_fl.setIdleMode(IdleMode.kBrake);
    spark_fr.setIdleMode(IdleMode.kBrake);
    spark_bl.setIdleMode(IdleMode.kBrake);
    spark_br.setIdleMode(IdleMode.kBrake);

    spark_fr.getEncoder().setPositionConversionFactor(Constants.DrivetrainConstants.kEncoderDistancePerPulse);
    spark_fl.getEncoder().setPositionConversionFactor(Constants.DrivetrainConstants.kEncoderDistancePerPulse);
    spark_br.getEncoder().setPositionConversionFactor(Constants.DrivetrainConstants.kEncoderDistancePerPulse);
    spark_bl.getEncoder().setPositionConversionFactor(Constants.DrivetrainConstants.kEncoderDistancePerPulse);
    spark_fr.getEncoder().setVelocityConversionFactor(vel);
    spark_fl.getEncoder().setVelocityConversionFactor(vel);
    spark_br.getEncoder().setVelocityConversionFactor(vel);
    spark_bl.getEncoder().setVelocityConversionFactor(vel);

    drivetrain = new MecanumDrive(spark_fl, spark_bl, spark_fr, spark_br);
    
    resetOdometry(new Pose2d(4,0,navx.getRotation2d()));
    resetEncoders();
    navx.reset();
    navx.calibrate();


  }

  //singleton structure so to make an instance you call Drivetrain.getInstance()
  
  public static Drivetrain getInstance(){
    if(instance == null){
      instance = new Drivetrain();
    }
    return instance;
  }

  //getters for encoder velocity

  public double getFLEncoderVelocity(){
    return spark_fl.getEncoder().getVelocity();
  }
  public double getFREncoderVelocity(){
    return spark_fr.getEncoder().getVelocity();
  }
  public double getBLEncoderVelocity(){
    return spark_bl.getEncoder().getVelocity();
  }
  public double getBREncoderVelocity(){
    return spark_br.getEncoder().getVelocity();
  }

  //Robot oriented drive

  public void RobotOrientedDrive(double y, double x, double zRot){
    if(Math.abs(x) < DEADZONE) x = 0;
    if(Math.abs(y) < DEADZONE) y = 0;
    if(Math.abs(zRot) < DEADZONE) zRot = 0;
    drivetrain.driveCartesian(y, x, zRot);
  }

  //Field Oriented Drive

  public void FieldOrientedDrive(double x, double y, double zRotation){
    if(Math.abs(x) < DEADZONE) x = 0;
    if(Math.abs(y) < DEADZONE) y = 0;
    if(Math.abs(zRotation) < DEADZONE) zRotation = 0;
    drivetrain.driveCartesian(x, y, zRotation, getNavxAngle().unaryMinus().minus(navxAngleOffset.unaryMinus()));

  }
  
  //For setting individual speeds to each motor
  
  public void driveMecanum(double fl, double bl, double fr, double br){
    spark_fl.set(fl);
    spark_bl.set(bl);
    spark_fr.set(fr);
    spark_br.set(br);
  }

  //For setting individual volts to each motor

  public void mecanumVolts(MecanumDriveMotorVoltages volts){
    spark_fl.setVoltage(volts.frontLeftVoltage);
    spark_fr.setVoltage(volts.frontRightVoltage);
    spark_bl.setVoltage(volts.rearLeftVoltage);
    spark_br.setVoltage(volts.rearRightVoltage);
  }

  //Getting the MecanumDriveWheelPositions 
  public MecanumDriveWheelPositions getMecanumDriveWheelPositions(){
    return new MecanumDriveWheelPositions(
        spark_fl.getEncoder().getPosition(), 
        spark_fr.getEncoder().getPosition(), 
        spark_bl.getEncoder().getPosition(),
        spark_br.getEncoder().getPosition()
      );
  }

  //Getting the pose from the odometry
  public Pose2d getPose(){ 
    return m_odometry.getPoseMeters();
  }

  

  //resetting the odometry
  public void resetOdometry(Pose2d pose){
    m_odometry.resetPosition(navx.getRotation2d(), getMecanumDriveWheelPositions(), pose);
    // m_odometry.resetPosition(getNavxAngle(), getMecanumDriveWheelPositions(), getPose());
  }


  public Pose2d getVisionRobotPose(){
    return m_visionOdometry.getRobotPose();
  }

  public void resetVisionOdometry(){
    m_visionOdometry.resetRobotPose();
  }
  
  //resetting the encoders  

  public void resetEncoders(){
    spark_fl.getEncoder().setPosition(0);
    spark_bl.getEncoder().setPosition(0);
    spark_fr.getEncoder().setPosition(0);
    spark_br.getEncoder().setPosition(0);
  }

  //getting the current wheelspeeds

  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds(){
    return new MecanumDriveWheelSpeeds(
      spark_fl.getEncoder().getVelocity(),
      spark_fr.getEncoder().getVelocity(),
      spark_bl.getEncoder().getVelocity(),
      spark_br.getEncoder().getVelocity());

  }

  //getting wheel voltages and amps
  public double[] getWheelVoltages(){
    return new double[]{spark_fl.getBusVoltage(), spark_fr.getBusVoltage(), spark_bl.getBusVoltage(), spark_br.getBusVoltage()};
  }

  public double[] getWheelAmperage(){
    return new double[]{spark_fl.getOutputCurrent(), spark_fr.getOutputCurrent(), spark_bl.getOutputCurrent(), spark_br.getOutputCurrent()};
  }


  //Returns the total accumulated yaw angle (Z Axis, in degrees) reported by the sensor.
  public Rotation2d getNavxAngle(){
    return Rotation2d.fromDegrees(-navx.getAngle());
  }

   // setter for setting the navxAngleOffset
   public void setNavxAngleOffset(Rotation2d angle){
    navxAngleOffset = angle;
  }


  public double linearAccelX(){
    return navx.getWorldLinearAccelX();
  }

  public double linearAccelY(){
    return navx.getWorldLinearAccelY();
  }

  public double linearAccelZ(){
    return navx.getWorldLinearAccelZ();
  }

  public double getAngularVel(){
    return navx.getRate();
  }

  public boolean isCalibrating(){
    return navx.isCalibrating();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Updating the Odometry
    m_odometry.update(getNavxAngle(), getMecanumDriveWheelPositions());
    // m_odometry.update(vision_odometry.getRobotPose().getRotation(), vision_odometry.getRobotPose());
    // m_field.setRobotPose(m_odometry.getPoseMeters());
  }
  
}
