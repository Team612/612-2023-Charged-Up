
package frc.robot.commands.Drivetrain;

import java.util.List;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants  ;

/** Add your docs here. */
public class TrajectoryCreation {

    public TrajectoryConfig config = new TrajectoryConfig(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq)
        .setKinematics(Constants.DrivetrainConstants.kDriveKinematics);

    public Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(new Translation2d(0.5,0)),
        new Pose2d(1,0, new Rotation2d(0)), 
        config);   
}