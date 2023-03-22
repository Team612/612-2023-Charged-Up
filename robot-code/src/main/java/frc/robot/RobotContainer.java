// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.commands.PivotPositions.DefenseMode;
import frc.robot.commands.PivotPositions.ExtendToPosition;
import frc.robot.commands.ReleaseAuto;
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
  private final ExtendRetract m_telescope = new ExtendRetract(m_scope, m_arm);
  private final Grab m_grab = new Grab(m_grabber);
  private final Release m_release = new Release(m_grabber);
  private final ReleaseAuto m_releaseauto = new ReleaseAuto(m_grabber);
  private final AutoBalance m_autoBalance = new AutoBalance(m_drivetrain);
  
  //gunner outtakes/defense mode
  private final Command m_midCone = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.7, EncoderConstants.MidPositionConePivot).
    andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.MidPositionConeTele))).
    until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1 || ControlMap.GUNNER_RB.getAsBoolean() || ControlMap.GUNNER_LB.getAsBoolean());
  
  private final Command m_midCube = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.7, EncoderConstants.MidPositionCubePivot).
    andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.MidPositionCubeTele))).
    until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1 || ControlMap.GUNNER_RB.getAsBoolean() || ControlMap.GUNNER_LB.getAsBoolean());

  private final Command m_highCube = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.7, EncoderConstants.HighPositionCubePivot).
    andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.HighPositionCubeTele))).
    until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1 || ControlMap.GUNNER_RB.getAsBoolean() || ControlMap.GUNNER_LB.getAsBoolean());
    
  private final Command m_humanStation = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.7, EncoderConstants.HumanStationIntakePivot).
    andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.HumanStationIntakeTele))).
    until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1 || ControlMap.GUNNER_RB.getAsBoolean() || ControlMap.GUNNER_LB.getAsBoolean());

      
    // private final SequentialCommandGroup m_highCone = new SequentialCommandGroup(
    //   new ExtendToPosition(m_scope, 0.7, 0).
    //   andThen(new MoveToPosition(m_arm, 0.7, EncoderConstants.HighPositionCubePivot)).
    //   andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.HighPositionConeTele)));

  private final Command m_lowGeneral = new SequentialCommandGroup(
    new ExtendToPosition(m_scope, 0.7, 0).
    andThen(new MoveToPosition(m_arm, 0.7, EncoderConstants.LowPositionPivot)).
    andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.LowPositionTele))).
    until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1 || ControlMap.GUNNER_RB.getAsBoolean() || ControlMap.GUNNER_LB.getAsBoolean());
    
  private final DefenseMode m_DefenseMode = new DefenseMode(m_scope, 0.7); //write override here
  private final ParallelCommandGroup m_stow = new ParallelCommandGroup(new DefenseMode(m_scope, 0.5).alongWith(new MoveToPosition(m_arm, 0.3, 0)));
  
  private final Command m_autoScore = new SequentialCommandGroup(
    (new SequentialCommandGroup(new DefenseMode(m_scope, 0.1)
    .andThen(new Grab(m_grabber))))
  .andThen(
    new ParallelCommandGroup(new SequentialCommandGroup(
      new MoveToPosition(m_arm, 0.7, EncoderConstants.MidPositionConePivot).
      andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.MidPositionConeTele)))
    .alongWith(new Grab(m_grabber))))
  .andThen(new MoveToPosition(m_arm, 0.3, 100))
  .andThen(m_releaseauto))
  .andThen(m_stow);

  public final Vision m_Vision = Vision.getVisionInstance();
  //public final Vision m_Vision = new Vision(camera);

  public final PoseEstimator estimator = PoseEstimator.getPoseEstimatorInstance();

  // private final SequentialCommandGroup m_RedRightLeave_doc = new SequentialCommandGroup(
  //   new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedRightLeaveAndDock", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, false)
  //   .andThen(new DockingSequence(m_drivetrain))
  // );

  // private final SequentialCommandGroup m_RedLeftLeave_doc = new SequentialCommandGroup(
  //   new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedLeftLeaveAndDock", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, false)
  //   .andThen(new DockingSequence(m_drivetrain))
  // );

  // private final SequentialCommandGroup m_BlueLeftLeave_doc = new SequentialCommandGroup(
  //   new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueLeftLeaveAndDock", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, false)
  //   .andThen(new DockingSequence(m_drivetrain))
  // );

  // private final SequentialCommandGroup m_BlueRightLeave_doc = new SequentialCommandGroup(
  //   new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueRightLeaveAndDock", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, false)
  //   .andThen(new DockingSequence(m_drivetrain))
  // );

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
    //configureOverrideCommands();
  }

  /*private void configureOverrideCommands() {
    m_midCone.withInterruptBehavior(m_pivot.getInterruptionBehavior());
    m_midCube.withInterruptBehavior(m_pivot.getInterruptionBehavior());
    m_highCube.withInterruptBehavior(m_pivot.getInterruptionBehavior());
    m_humanStation.withInterruptBehavior(m_pivot.getInterruptionBehavior());
    m_lowGeneral.withInterruptBehavior(m_pivot.getInterruptionBehavior());
    m_midCone.until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1);
    m_midCube.until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1);
    m_highCube.until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1);
    m_humanStation.until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1);
    m_lowGeneral.until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1);
  }*/

  private void configureShuffleBoardBindings(){
    m_chooser.addOption("Auto-Balance", new DockingSequence(m_drivetrain));
    //m_chooser.addOption("RollOff", new RollOff(m_drivetrain));
    
    //m_chooser.addOption("Align", new ProxyCommand(() -> new followTag(m_drivetrain, m_Vision.getCamera())));
    
    //m_chooser.addOption("Strafe Right debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.StrafeRightMeter(estimator),estimator)));
    //m_chooser.addOption("Strafe Left debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.StrafeLeftMeter(estimator),estimator)));
    //m_chooser.addOption("Forward debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.ForwardMeter(estimator),estimator)));
    //m_chooser.addOption("Backward debug", new ProxyCommand(() -> m_follower.generateTrajectory(m_drivetrain, m_traj.BackwardMeter(estimator),estimator)));
  
    // m_chooser.addOption("RedRightLeaveAndDock", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedRightLeaveAndDock", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, false)));
    m_chooser.addOption("Red Bottom Leave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedBottomLeave", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, false)));
    // m_chooser.addOption("RedLeftLeaveAndDock", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedLeftLeaveAndDock", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, false)));
    m_chooser.addOption("Red Top Leave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedTopLeave", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, false)));


    // m_chooser.addOption("RedRightLeaveAndDock", new ProxyCommand(() -> m_RedRightLeave_doc));
    // m_chooser.addOption("RedLeftLeaveAndDock", new ProxyCommand(() -> m_RedLeftLeave_doc));
    // m_chooser.addOption("BlueRightLeaveAndDock", new ProxyCommand(() -> m_BlueRightLeave_doc));
    // m_chooser.addOption("BlueLeftLeaveAndDock", new ProxyCommand(() -> m_BlueLeftLeave_doc));

    //m_chooser.addOption("RedMiddleLeaveAndDoc", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedMiddleLeaveAndDoc", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, false)));
   
    // m_chooser.addOption("BlueRightLeaveAndDock", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueRightLeaveAndDock", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, true)));
    m_chooser.addOption("Blue Bottom Leave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueBottomLeave", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, true)));
    // m_chooser.addOption("BlueLeftLeaveAndDock", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueLeftLeaveAndDock", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, true)));
    m_chooser.addOption("Blue Top Leave", new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueTopLeave", new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq), true, true)));
    m_chooser.addOption("Auto Score Cone", m_autoScore);
  
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
    //ControlMap.red5.toggleOnTrue(new ProxyCommand(() -> m_highCone));
    ControlMap.green2.toggleOnTrue(new ProxyCommand(() -> m_lowGeneral));

    ControlMap.green1.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, m_traj, m_Vision, Units.inchesToMeters(28.5)))); //robot oriented right cone
    ControlMap.yellow2.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, m_traj, m_Vision, 0))); //april tag alignment
    ControlMap.yellow1.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, m_traj, m_Vision, Units.inchesToMeters(-22.5 -6)))); 

    // ControlMap.green1.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, m_traj, m_Vision, Units.inchesToMeters(-34))));
    


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
