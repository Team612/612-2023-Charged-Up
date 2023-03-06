// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drivetrain.DefaultDrive;
import frc.robot.commands.Drivetrain.FieldOrientedDrive;
import frc.robot.commands.Drivetrain.FollowTrajectory;
import frc.robot.commands.Drivetrain.SetForward;
import frc.robot.commands.Drivetrain.TrajectoryCreation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  //subsystem declarations 
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  
  public final Vision m_Vision = Vision.getVisionInstance();
  //public final Vision m_Vision = new Vision(camera);

  public final PoseEstimator estimator = PoseEstimator.getPoseEstimatorInstance();

  private final DefaultDrive m_defaultdrive = new DefaultDrive(m_drivetrain);

  // Trajectories
  private final FollowTrajectory m_follower = new FollowTrajectory();
  private final TrajectoryCreation m_traj = new TrajectoryCreation();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  

  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    configureDefaultCommands();
    configureShuffleBoardBindings();
  }

  private void configureShuffleBoardBindings(){
    m_chooser.addOption("Strafe Right debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.StrafeRightMeter(estimator),estimator)));
    m_chooser.addOption("Strafe Left debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.StrafeLeftMeter(estimator),estimator)));
    m_chooser.addOption("Forward debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.ForwardMeter(estimator),estimator)));
    m_chooser.addOption("Backward debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.BackwardMeter(estimator),estimator)));
    m_chooser.addOption("square debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.square(estimator),estimator)));
    m_chooser.addOption("LeaveAndDock", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.LeaveAndDock(estimator),estimator)));
    SmartDashboard.putData(m_chooser);
  }

  private void configureButtonBindings() {
    m_driverController.y().whileTrue(new SetForward(m_drivetrain));
    m_driverController.back().toggleOnTrue(new FieldOrientedDrive(m_drivetrain));
  }

  private void configureDefaultCommands(){
    m_drivetrain.setDefaultCommand(m_defaultdrive);
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
