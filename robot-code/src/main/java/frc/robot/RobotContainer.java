// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drivetrain.DefaultDrive;
import frc.robot.commands.Drivetrain.FollowTrajectory;
import frc.robot.commands.Drivetrain.SetForward;
import frc.robot.commands.Drivetrain.TrajectoryCreation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.commands.Drivetrain.DockingSequence;
import frc.robot.commands.Drivetrain.RollOff;
import frc.robot.commands.Grab;
import frc.robot.commands.Pivot;
import frc.robot.commands.Release;
import frc.robot.commands.TelescopeDetract;
import frc.robot.commands.TelescopeExtend;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Telescope;
import frc.robot.commands.Drivetrain.SlowmoDrive;
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
  public final PhotonCamera camera = new PhotonCamera(Constants.cameraName);
  public final Vision m_Vision = new Vision(camera);

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DefaultDrive m_defaultdrive = new DefaultDrive(m_drivetrain);
  private final SlowmoDrive m_slowmodrive = new SlowmoDrive(m_drivetrain);

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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController gunner =
      new CommandXboxController(OperatorConstants.kGunnerControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    configureShuffleBoardBindings();
    configureDefaultCommands();
  }


  public void runCommands(){
    PhotonPipelineResult result = camera.getLatestResult();
    if(result.hasTargets()){
      // System.out.println(result.getBestTarget().getFiducialId());
      System.out.println(m_Vision.return_camera_pose_tag(camera.getLatestResult().getBestTarget().getFiducialId(), camera.getLatestResult()));
    }
    else{
      System.out.println("********************************No targets*****************************************");
    }

  }

  private void configureShuffleBoardBindings(){
    m_chooser.addOption("Auto-Balance", new DockingSequence(m_drivetrain));
    m_chooser.addOption("TestTrajectory", m_follower.generateTrajectory(m_drivetrain, m_traj.testTrajectory));
    m_chooser.addOption("RollOff", new RollOff(m_drivetrain));
    m_chooser.addOption("TestTrajectory", m_follower.generateTrajectory(m_drivetrain, m_traj.testTrajectory));
    m_chooser.addOption("Vision Trajectory", m_follower.generateTrajectory(m_drivetrain, m_traj.return_Trajectory(camera, m_Vision, new Pose3d(14.2, 1.071626, 0.462788, new Rotation3d(new Quaternion(0,0,0,1))))));
    SmartDashboard.putData(m_chooser);
    SmartDashboard.putData("Slowmo (Toggle)", new SlowmoDrive(m_drivetrain));
  }

  private void configureButtonBindings() {

    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));
    //Button-Bindings
    driver.y().whileTrue(new SetForward(m_drivetrain));
    driver.a().whileTrue(new DockingSequence(m_drivetrain));
    gunner.leftBumper().whileTrue(m_telescopeExtend);
    gunner.rightBumper().whileTrue(m_telescopeDetract);
    gunner.leftTrigger().whileTrue(m_grab);
    gunner.rightTrigger().whileTrue(m_release);
  }

  private void configureDefaultCommands(){
    m_drivetrain.setDefaultCommand(m_defaultdrive);
    m_arm.setDefaultCommand(m_pivot);
  }

  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
