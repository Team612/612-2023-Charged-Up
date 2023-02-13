// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drivetrain.DefaultDrive;
import frc.robot.commands.Drivetrain.FollowTrajectory;
import frc.robot.commands.Drivetrain.MovePID;
import frc.robot.commands.Drivetrain.SetForward;
import frc.robot.commands.Drivetrain.TrajectoryCreation;
import frc.robot.commands.Drivetrain.XAlignWithTag;
import frc.robot.commands.Drivetrain.YAlignWithTag;
import frc.robot.commands.Drivetrain.ZAlignWithTag;
import frc.robot.commands.Drivetrain.followTag;
import frc.robot.subsystems.Drivetrain;
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
  public final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.cameraName);
  public final Vision m_Vision = new Vision(camera);

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
    configureShuffleBoardBindings();
    configureDefaultCommands();
  }

  public void runCommands(){
    // PhotonPipelineResult result = camera.getLatestResult();
    // if(result.hasTargets()){
    //   // System.out.println(result.getBestTarget().getFiducialId());
    //   // System.out.println(m_Vision.return_camera_pose_tag(camera.getLatestResult().getBestTarget().getFiducialId(), camera.getLatestResult()).getY());
    //   PhotonTrackedTarget bestTarget = result.getBestTarget();
     
    //   Transform3d transform3d = bestTarget.getBestCameraToTarget();
    //   System.out.println(transform3d.getY());    
    // }
    // else{
    //   System.out.println("********************************No targets*****************************************");
    // }
    // System.out.println(Drivetrain.NavxAngle());
    // if(camera.getLatestResult().hasTargets()){
    //   // System.out.println(-camera.getLatestResult().getBestTarget().getYaw());
    //   // System.out.println(Drivetrain.NavxAngle());
    // }
    // System.out.println(Drivetrain.NavxAngle());

  }

  private void configureShuffleBoardBindings(){
    m_chooser.addOption("Align Trajectory", m_follower.generateTrajectory(m_drivetrain, m_traj.return_alignTrajectory(camera, new Translation2d(1.65,0))));
    m_chooser.addOption("Vision Trajectory", m_follower.generateTrajectory(m_drivetrain, m_traj.return_Trajectory(camera, m_Vision, new Pose3d(14.2, 1.071626, 0.462788, new Rotation3d(new Quaternion(0,0,0,1))))));
    m_chooser.addOption("Align", new followTag(m_drivetrain, camera));
    m_chooser.addOption("Tune Angles", m_follower.generateTrajectory(m_drivetrain, m_traj.tuneAngle));
    m_chooser.addOption("straif left debug", m_follower.generateTrajectory(m_drivetrain, m_traj.StraifLeft));
    m_chooser.addOption("straif right debug", m_follower.generateTrajectory(m_drivetrain, m_traj.StraifRight(m_drivetrain)));

    m_chooser.addOption("forward debug", m_follower.generateTrajectory(m_drivetrain, m_traj.forwardTrajectory(m_drivetrain)));
    m_chooser.addOption("backwards debug", m_follower.generateTrajectory(m_drivetrain, m_traj.backwardTrajectory));

    m_chooser.addOption("PID debug", new MovePID(m_drivetrain, 1, 0, 0));

    m_chooser.addOption("Align X", new XAlignWithTag(m_drivetrain, camera));
    m_chooser.addOption("Align Z", new YAlignWithTag(m_drivetrain, camera));
    m_chooser.addOption("Align Z", new ZAlignWithTag(m_drivetrain, camera));



    SmartDashboard.putData(m_chooser);
  }

  private void configureButtonBindings() {

    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    m_driverController.y().whileTrue(new SetForward(m_drivetrain));
  }

  private void configureDefaultCommands(){
    m_drivetrain.setDefaultCommand(m_defaultdrive);
  }

  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
