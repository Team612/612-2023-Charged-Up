// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Proxy;

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
import frc.robot.commands.PivotPositions.DefenseMode;
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
  
  //Drive subsystems declarations 
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  private final DefaultDrive m_defaultdrive = new DefaultDrive(m_drivetrain);
  private final FieldOrientedDrive m_FieldOrientedDrive = new FieldOrientedDrive(m_drivetrain);

 // Trajectories
  private final FollowTrajectory m_follower = new FollowTrajectory();
  private final TrajectoryCreation m_traj = new TrajectoryCreation();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  //Subsystems
  private final Arm m_arm = Arm.getInstance();
  private final Telescope m_scope = Telescope.getInstance();
  private final Grabber m_grabber = Grabber.getInstance();

  //Commands
  private final Pivot m_pivot = new Pivot(m_arm);
  private final ExtendRetract m_telescope = new ExtendRetract(m_scope);
  private final Grab m_grab = new Grab(m_grabber);
  private final Release m_release = new Release(m_grabber);
  private final AutoBalance m_autoBalance = new AutoBalance(m_drivetrain);
  
  //gunner outtakes/defense mode
  private final SequentialCommandGroup m_midCone = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.7, EncoderConstants.MidPositionConePivot).
    andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.MidPositionConeTele)));
  
  private final SequentialCommandGroup m_midCube = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.7, EncoderConstants.MidPositionCubePivot).
    andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.MidPositionCubeTele)));

  private final SequentialCommandGroup m_highCube = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.7, EncoderConstants.HighPositionCubePivot).
    andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.HighPositionCubeTele)));

  private final SequentialCommandGroup m_humanStation = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.7, EncoderConstants.HumanStationIntakePivot).
    andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.HumanStationIntakeTele)));
  
  private final SequentialCommandGroup m_lowGeneral = new SequentialCommandGroup(
  new ExtendToPosition(m_scope, 0.7, 0).
  andThen(new MoveToPosition(m_arm, 0.7, EncoderConstants.LowPositionPivot)).
  andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.LowPositionTele)));

  
  private final DefenseMode m_DefenseMode = new DefenseMode(m_scope, 0.7);
  
  public final Vision m_Vision = Vision.getVisionInstance();
  //public final Vision m_Vision = new Vision(camera);

  public final PoseEstimator estimator = PoseEstimator.getPoseEstimatorInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_gunnerController =
      new CommandXboxController(OperatorConstants.kGunnerControllerPort);

  
  private final SequentialCommandGroup m_RedMiddleLeaveAndDock = new SequentialCommandGroup(
    new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedMiddleLeaveAndDock", Constants.DrivetrainConstants.constraint, true, false)
    .andThen(new DockingSequence(m_drivetrain))
  );

  private final SequentialCommandGroup m_BlueMiddleLeaveAndDock = new SequentialCommandGroup(
    new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueMiddleLeaveAndDock", Constants.DrivetrainConstants.constraint, true, false)
    .andThen(new DockingSequence(m_drivetrain))
  );

  private final SequentialCommandGroup m_RedTopLeaveAndDock = new SequentialCommandGroup(
    new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedTopLeaveAndDock", Constants.DrivetrainConstants.constraint, true, false)
    .andThen(new DockingSequence(m_drivetrain))
  );

  private final SequentialCommandGroup m_BlueTopLeaveAndDock = new SequentialCommandGroup(
    new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueTopLeaveAndDock", Constants.DrivetrainConstants.constraint, true, false)
    .andThen(new DockingSequence(m_drivetrain))
  );

  private final SequentialCommandGroup m_RedBottomLeaveAndDock = new SequentialCommandGroup(
    new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedBottomLeaveAndDock", Constants.DrivetrainConstants.constraint, true, false)
    .andThen(new DockingSequence(m_drivetrain))
  );

  private final SequentialCommandGroup m_BlueBottomLeaveAndDock = new SequentialCommandGroup(
    new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueBottomLeaveAndDock", Constants.DrivetrainConstants.constraint, true, false)
    .andThen(new DockingSequence(m_drivetrain))
  );

  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    configureShuffleBoardBindings();
    configureDefaultCommands();
  }

  private void configureShuffleBoardBindings(){
    m_chooser.addOption("Auto-Balance", new DockingSequence(m_drivetrain));
    m_chooser.addOption("Red Bottom Leave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedBottomLeave", Constants.DrivetrainConstants.constraint, true, false)));
    m_chooser.addOption("Red Top Leave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedTopLeave", Constants.DrivetrainConstants.constraint, true, false)));
    m_chooser.addOption("Blue Bottom Leave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueBottomLeave", Constants.DrivetrainConstants.constraint, true, true)));
    m_chooser.addOption("Blue Top Leave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueTopLeave", Constants.DrivetrainConstants.constraint, true, true)));
    m_chooser.addOption("Red Middle Leave and Dock", new ProxyCommand(() -> m_RedMiddleLeaveAndDock));
    m_chooser.addOption("Blue Middle Leave and Dock", new ProxyCommand(() -> m_BlueMiddleLeaveAndDock));
    m_chooser.addOption("Red Top Leave And Dock", m_RedTopLeaveAndDock);
    m_chooser.addOption("Blue Top Leave And Dock", m_BlueTopLeaveAndDock);
    m_chooser.addOption("Red Bottom Leave And Dock", m_RedBottomLeaveAndDock);
    m_chooser.addOption("Blue Bottom Leave and Dock", m_BlueBottomLeaveAndDock);

  
    SmartDashboard.putData(m_chooser);
    SmartDashboard.putData("Slowmo (Toggle)", new SlowmoDrive(m_drivetrain));
  }

  private void configureButtonBindings() {

    m_gunnerController.leftTrigger().whileTrue(m_grab);
    m_driverController.y().whileTrue(new SetForward(m_drivetrain));
    m_driverController.back().toggleOnTrue(m_defaultdrive);
    ControlMap.red2.toggleOnTrue(new ProxyCommand(() -> m_autoBalance));

    ControlMap.blue1.toggleOnTrue(new ProxyCommand(() -> m_midCube));
    ControlMap.blue2.toggleOnTrue(new ProxyCommand(() -> m_highCube));
    ControlMap.red4.toggleOnTrue(new ProxyCommand(() -> m_midCone));
    ControlMap.red5.toggleOnTrue(new ProxyCommand(() -> m_humanStation));
    ControlMap.green2.toggleOnTrue(new ProxyCommand(() -> m_lowGeneral));

    ControlMap.green1.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, m_traj, m_Vision, Units.inchesToMeters(22 + 4)))); //robot oriented right cone
    ControlMap.yellow2.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, m_traj, m_Vision, 0))); //april tag alignment
    ControlMap.yellow1.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, m_traj, m_Vision, Units.inchesToMeters(-22 - 4)))); 
    ControlMap.red6.toggleOnTrue(new ProxyCommand(() -> m_DefenseMode));
  }


  private void configureDefaultCommands(){
    m_drivetrain.setDefaultCommand(m_FieldOrientedDrive);
    m_arm.setDefaultCommand(m_pivot);
    m_scope.setDefaultCommand(m_telescope);
    m_grabber.setDefaultCommand(m_release);
  }

  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
