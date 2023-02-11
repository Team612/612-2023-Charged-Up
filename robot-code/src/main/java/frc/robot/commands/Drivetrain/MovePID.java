// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class MovePID extends CommandBase {

  Drivetrain m_drivetrain;
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
  public static final TrapezoidProfile.Constraints kPositionControllerConstraints = new TrapezoidProfile.Constraints(1,3);

  // PID controllers with velocity and acceleration constraints
  ProfiledPIDController m_controller_x = new ProfiledPIDController(1, 0, 0.6, kPositionControllerConstraints);
  ProfiledPIDController m_controller_y = new ProfiledPIDController(1, 0, 0.6, kPositionControllerConstraints);
  ProfiledPIDController m_controller_angle = new ProfiledPIDController(1, 0, 0.6, kThetaControllerConstraints);
  

  //Desired position
  private double setpointX; //= 1; //should be in meters
  private double setPointY; //= 1; //should be in meters
  private double setPointAngle; //= 0; //should be in degrees

  

  //initial position
  private double initX;
  private double initY;
  private double initAngle;

  //thresholds
  private double posThreshold = 0.05;
  private double angleThreshold = 1;


  // PIDController m_controller_y = new PIDContro\ller(0, 0, 0)
  public MovePID(Drivetrain drivetrain, double _setpointX, double _setpointY, double _setPointAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    setpointX = _setpointX;
    setPointY = _setpointX;
    setPointAngle = _setPointAngle;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller_x.setGoal(setpointX);
    m_controller_y.setGoal(setPointY);
    m_controller_angle.setGoal(setPointAngle);
    
    initX = m_drivetrain.getPose().getX();
    initY = m_drivetrain.getPose().getY();
    initAngle = m_drivetrain.getPose().getRotation().getDegrees();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //calculating the outputs 
    double pidOutput_X = m_controller_x.calculate(m_drivetrain.getPose().getX(),setpointX);    
    double pidOutput_Y = m_controller_y.calculate(m_drivetrain.getPose().getY(),setPointY);
    double pidOutput_angle = m_controller_angle.calculate(m_drivetrain.getPose().getRotation().getDegrees(), setPointAngle);

    //feeding outputs in
    m_drivetrain.driveMecanum(pidOutput_X, pidOutput_Y, pidOutput_angle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stopping the robot
    m_drivetrain.driveMecanum(0, 0, 0, 0);    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean x_thresh = m_drivetrain.getPose().getX() - initX >= setpointX - posThreshold && m_drivetrain.getPose().getX() - initX <= setpointX + posThreshold;
    boolean y_thresh = m_drivetrain.getPose().getY() - initY >= setpointX - posThreshold && m_drivetrain.getPose().getY() - initY <= setpointX + posThreshold;
    boolean angle_thresh = m_drivetrain.getPose().getRotation().getDegrees() - initAngle >= setPointAngle - angleThreshold && m_drivetrain.getPose().getRotation().getDegrees() - initX <= setPointAngle + angleThreshold;    
    return x_thresh && y_thresh && angle_thresh;
  }
}
