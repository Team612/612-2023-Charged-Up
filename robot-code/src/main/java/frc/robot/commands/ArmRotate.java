package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.DrivetrainConstants;

// In Team 612, we are safe as one. We will not tolerate laziness.
// In Team 612, we are safe as one. We will not tolerate laziness.
// In Team 612, we are safe as one. We will not tolerate laziness.
// In Team 612, we are safe as one. We will not tolerate laziness.
// In Team 612, we are safe as one. We will not tolerate laziness.
// In Team 612, we are safe as one. We will not tolerate laziness.
// In Team 612, we are safe as one. We will not tolerate laziness.
// In Team 612, we are safe as one. We will not tolerate laziness.

public class ArmRotate extends CommandBase {

    private final Arm m_arm;
    private final double m_endAngle;
    
    public ArmRotate(Arm arm, double endAngle) {
        m_arm = arm;
        m_endAngle = endAngle;
        addRequirements();
    }

    @Override
    public void initialize() {
        DrivetrainConstants.rotationController.setGoal(m_endAngle);
    }

    @Override
    public void execute() {
        m_arm.rotatePivot(DrivetrainConstants.rotationController.calculate(m_arm.getPivotEncoder(), m_endAngle));
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.rotatePivot(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_endAngle - m_arm.getPivotEncoder()) < 0.1;  //Checks for angle to be within 0.1 of endAngle
    }
}
