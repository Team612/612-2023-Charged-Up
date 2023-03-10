// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drivetrain.AutoBalance;
import frc.robot.commands.Drivetrain.DefaultDrive;
import frc.robot.commands.Drivetrain.FollowTrajectory;
import frc.robot.commands.Drivetrain.SetForward;
import frc.robot.commands.Drivetrain.SlowmoDrive;
import frc.robot.commands.Drivetrain.TrajectoryCreation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.commands.Drivetrain.DockingSequence;
import frc.robot.commands.Drivetrain.RollOff;
import frc.robot.commands.Grab;
import frc.robot.commands.Pivot;
import frc.robot.commands.Release;
import frc.robot.commands.ScoreArm;
import frc.robot.commands.TelescopeDetract;
import frc.robot.commands.TelescopeExtend;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Telescope;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.commands.Drivetrain.FieldOrientedDrive;
import frc.robot.commands.Drivetrain.FollowTrajectoryPathPlanner;
import frc.robot.commands.Drivetrain.followTag;
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


  private final DefaultDrive m_defaultdrive = new DefaultDrive(m_drivetrain);

 // Trajectories
  private final FollowTrajectory m_follower = new FollowTrajectory();
  private final TrajectoryCreation m_traj = new TrajectoryCreation();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final Arm m_arm = Arm.getInstance();
  private final Telescope m_scope = Telescope.getInstance();
  private final Grabber m_grabber = Grabber.getInstance();

  private final Pivot m_pivot = new Pivot(m_arm);
  private final TelescopeDetract m_telescopeDetract = new TelescopeDetract(m_scope);
  private final TelescopeExtend m_telescopeExtend = new TelescopeExtend(m_scope);
  private final Grab m_grab = new Grab(m_grabber);
  private final Release m_release = new Release(m_grabber);
  private final AutoBalance m_autoBalance = new AutoBalance(m_drivetrain);
  private final ScoreArm m_scoreArm = new ScoreArm(m_arm);
  
  public final Vision m_Vision = Vision.getVisionInstance();
  //public final Vision m_Vision = new Vision(camera);

  public final PoseEstimator estimator = PoseEstimator.getPoseEstimatorInstance();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController gunner =
      new CommandXboxController(OperatorConstants.kGunnerControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    configureShuffleBoardBindings();
    configureDefaultCommands();
  }

  private void configureShuffleBoardBindings(){
    m_chooser.addOption("Auto-Balance", new DockingSequence(m_drivetrain));
    m_chooser.addOption("RollOff", new RollOff(m_drivetrain));
    
    m_chooser.addOption("align trajectory", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.return_alignTrajectory(m_Vision.getCamera(), new Translation2d(1.6,0)),estimator)));
    m_chooser.addOption("Align", new ProxyCommand(() -> new followTag(m_drivetrain, m_Vision.getCamera())));
    m_chooser.addOption("Tune Angles", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.tuneAngle,estimator)));
    
    m_chooser.addOption("Strafe Right debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.StrafeRightMeter(estimator),estimator)));
    m_chooser.addOption("Strafe Left debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.StrafeLeftMeter(estimator),estimator)));
    m_chooser.addOption("Forward debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.ForwardMeter(estimator),estimator)));
    m_chooser.addOption("Backward debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.BackwardMeter(estimator),estimator)));
    
    m_chooser.addOption("RedRightLeaveAndDock", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedRightLeaveAndDock", new PathConstraints(2.5, 1), true, false)));
    m_chooser.addOption("RedRightLeave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedRightLeave", new PathConstraints(2.5, 1), true, false)));
    m_chooser.addOption("RedLeftLeaveAndDock", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedLeftLeaveAndDock", new PathConstraints(2.5, 1), true, false)));
    m_chooser.addOption("RedLeftLeave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedLeftLeave", new PathConstraints(2.5, 1), true, false)));
    m_chooser.addOption("RedMiddleLeaveAndDoc", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedMiddleLeaveAndDoc", new PathConstraints(2.5, 1), true, false)));
   
    m_chooser.addOption("BlueRightLeaveAndDock", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueRightLeaveAndDock", new PathConstraints(2.5, 1), true, true)));
    m_chooser.addOption("BlueRightLeave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueRightLeave", new PathConstraints(2.5, 1), true, true)));
    m_chooser.addOption("BlueLeftLeaveAndDock", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueLeftLeaveAndDock", new PathConstraints(2.5, 1), true, true)));
    m_chooser.addOption("BlueLeftLeave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueLeftLeave", new PathConstraints(2.5, 1), true, true)));
    SmartDashboard.putData(m_chooser);
    SmartDashboard.putData("Slowmo (Toggle)", new SlowmoDrive(m_drivetrain));
    
  }

  private void configureButtonBindings() {
    gunner.leftBumper().whileTrue(m_telescopeExtend);
    gunner.rightBumper().whileTrue(m_telescopeDetract);
    gunner.leftTrigger().whileTrue(m_grab);
    gunner.rightTrigger().whileTrue(m_release);
    gunner.a().onTrue(m_scoreArm);
    m_driverController.y().whileTrue(new SetForward(m_drivetrain));
    m_driverController.back().toggleOnTrue(new FieldOrientedDrive(m_drivetrain));
    m_driverController.x().toggleOnTrue(m_autoBalance);
  }

  private void configureDefaultCommands(){
    m_drivetrain.setDefaultCommand(m_defaultdrive);
    m_arm.setDefaultCommand(m_pivot);
  }

  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
