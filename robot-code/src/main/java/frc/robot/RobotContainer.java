// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drivetrain.AutoBalance;
import frc.robot.commands.Drivetrain.DefaultDrive;
import frc.robot.commands.Drivetrain.DockingSequence;
import frc.robot.commands.Drivetrain.FollowTrajectory;
import frc.robot.commands.Drivetrain.SetForward;
import frc.robot.commands.Drivetrain.SlowmoDrive;
import frc.robot.commands.Drivetrain.TrajectoryCreation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.commands.Drivetrain.RollOff;
import frc.robot.commands.Drivetrain.RunOnTheFly;
import frc.robot.commands.Grab;
import frc.robot.commands.Pivot;
import frc.robot.commands.Release;
import frc.robot.commands.ExtendRetract;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Telescope;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.commands.Drivetrain.FieldOrientedDrive;
import frc.robot.commands.Drivetrain.FollowTrajectoryPathPlanner;
import frc.robot.commands.Drivetrain.followTag;
import frc.robot.commands.PivotPositions.ExtendToPosition;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import frc.robot.commands.PivotPositions.MoveToPosition;
import frc.robot.controls.ControlMap;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  //subsystem declarations 
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();


  private final DefaultDrive m_defaultdrive = new DefaultDrive(m_drivetrain);
  private final FieldOrientedDrive m_FieldOrientedDrive = new FieldOrientedDrive(m_drivetrain);

 // Trajectories
  private final FollowTrajectory m_follower = new FollowTrajectory();
  private final TrajectoryCreation m_traj = new TrajectoryCreation();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final Arm m_arm = Arm.getInstance();
  private final Telescope m_scope = Telescope.getInstance();
  private final Grabber m_grabber = Grabber.getInstance();

  private final Pivot m_pivot = new Pivot(m_arm);

  private final ExtendRetract m_telescope = new ExtendRetract(m_scope);
  private final Grab m_grab = new Grab(m_grabber);
  private final Release m_release = new Release(m_grabber);
  private final AutoBalance m_autoBalance = new AutoBalance(m_drivetrain);

  private final SequentialCommandGroup m_midCone = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.3, EncoderConstants.MidPositionConePivot).
    andThen(new ExtendToPosition(m_scope, 0.3, EncoderConstants.MidPositionConeTele)));
  
  private final SequentialCommandGroup m_midCube = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.3, EncoderConstants.MidPositionCubePivot).
    andThen(new ExtendToPosition(m_scope, 0.3, EncoderConstants.MidPositionCubeTele)));

  private final SequentialCommandGroup m_highCube = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.3, EncoderConstants.HighPositionCubePivot).
    andThen(new ExtendToPosition(m_scope, 0.3, EncoderConstants.HighPositionCubeTele)));

  private final SequentialCommandGroup m_highCone = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.3, EncoderConstants.HighPositionConePivot).
    andThen(new ExtendToPosition(m_scope, 0.3, EncoderConstants.HighPositionConeTele)));
  
  private final SequentialCommandGroup m_lowGeneral = new SequentialCommandGroup(
    new ExtendToPosition(m_scope, 0.3, 0).
    andThen(new MoveToPosition(m_arm, 0.3, EncoderConstants.LowPositionPivot)).
    andThen(new ExtendToPosition(m_scope, 0.3, EncoderConstants.LowPositionTele)));
  
      

  


  // private final MoveToPosition m_HighPositionCone = new MoveToPosition(m_arm, 0.3, ShuffleBoardButtons.highCone.getDouble(0));
  // private final MoveToPosition m_HighPositionCube = new MoveToPosition(m_arm, 0.3, ShuffleBoardButtons.highCube.getDouble(0));
  // private final MoveToPosition m_MidPositionCone = new MoveToPosition(m_arm, 0.3, ShuffleBoardButtons.midCone.getDouble(0));
  // private final MoveToPosition m_MidPositionCube = new MoveToPosition(m_arm, 0.3, ShuffleBoardButtons.midCube.getDouble(0));
  // private final MoveToPosition m_HumanStation = new MoveToPosition(m_arm, 0.3, ShuffleBoardButtons.humanStation.getDouble(0));
  // private final MoveToPosition m_GroundIntake = new MoveToPosition(m_arm, 0.3, ShuffleBoardButtons.ground.getDouble(0));

  //button binded
  // private final MoveToPosition m_LowGeneral = 


  
  public final Vision m_Vision = Vision.getVisionInstance();
  //public final Vision m_Vision = new Vision(camera);

  public final PoseEstimator estimator = PoseEstimator.getPoseEstimatorInstance();



  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_gunnerController =
      new CommandXboxController(OperatorConstants.kGunnerControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    configureShuffleBoardBindings();
    configureDefaultCommands();
  }

  private void configureShuffleBoardBindings(){
    //m_chooser.addOption("Auto-Balance", new DockingSequence(m_drivetrain));
    //m_chooser.addOption("RollOff", new RollOff(m_drivetrain));
    
    //m_chooser.addOption("Align", new ProxyCommand(() -> new followTag(m_drivetrain, m_Vision.getCamera())));
    
    //m_chooser.addOption("Strafe Right debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.StrafeRightMeter(estimator),estimator)));
    //m_chooser.addOption("Strafe Left debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.StrafeLeftMeter(estimator),estimator)));
    //m_chooser.addOption("Forward debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.ForwardMeter(estimator),estimator)));
    //m_chooser.addOption("Backward debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.BackwardMeter(estimator),estimator)));
    
    m_chooser.addOption("RedRightLeaveAndDock", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedRightLeaveAndDock", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, false)));
    m_chooser.addOption("RedRightLeave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedRightLeave", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, false)));
    m_chooser.addOption("RedLeftLeaveAndDock", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedLeftLeaveAndDock", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, false)));
    m_chooser.addOption("RedLeftLeave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedLeftLeave", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, false)));
    //m_chooser.addOption("RedMiddleLeaveAndDoc", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedMiddleLeaveAndDoc", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, false)));
   
    m_chooser.addOption("BlueRightLeaveAndDock", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueRightLeaveAndDock", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, true)));
    m_chooser.addOption("BlueRightLeave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueRightLeave", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, true)));
    m_chooser.addOption("BlueLeftLeaveAndDock", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueLeftLeaveAndDock", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, true)));
    m_chooser.addOption("BlueLeftLeave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueLeftLeave", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, true)));
  
    SmartDashboard.putData(m_chooser);
    SmartDashboard.putData("Slowmo (Toggle)", new SlowmoDrive(m_drivetrain));
  }

  private void configureButtonBindings() {

    m_gunnerController.leftTrigger().whileTrue(m_grab);
    m_gunnerController.rightTrigger().whileTrue(m_release);
    m_driverController.y().whileTrue(new SetForward(m_drivetrain));
    m_driverController.back().toggleOnTrue(m_defaultdrive);
    m_driverController.x().toggleOnTrue(m_autoBalance);

    ControlMap.blue1.toggleOnTrue(new ProxyCommand(() -> m_midCube));
    ControlMap.blue2.toggleOnTrue(new ProxyCommand(() -> m_highCube));
    ControlMap.red4.toggleOnTrue(new ProxyCommand(() -> m_midCone));
    ControlMap.red5.toggleOnTrue(new ProxyCommand(() -> m_highCone));
    ControlMap.green2.toggleOnTrue(new ProxyCommand(() -> m_lowGeneral));
    
    ControlMap.yellow1.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, true, m_traj, m_Vision, Units.inchesToMeters(32))));
    ControlMap.yellow2.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, true, m_traj, m_Vision, 0)));
    ControlMap.green1.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, true, m_traj, m_Vision, Units.inchesToMeters(-34))));
    


    // ControlMap.red6.toggleOnTrue(new ProxyCommand(() -> new MoveToPosition(m_arm, 0.3, ShuffleBoardButtons.midCube.getDouble(0))));
  
  }


  private void configureDefaultCommands(){
    m_drivetrain.setDefaultCommand(m_FieldOrientedDrive);
    m_arm.setDefaultCommand(m_pivot);
    m_scope.setDefaultCommand(m_telescope);
  }

  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
