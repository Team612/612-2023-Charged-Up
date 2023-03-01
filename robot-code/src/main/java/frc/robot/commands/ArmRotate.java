package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmRotate extends CommandBase {

    //that looks like a major blunder ngl
    private final Arm m_arm;
    private final double m_endAngle;

    public ArmRotate(Arm arm, double endAngle) {
        m_arm = arm;
        m_endAngle = endAngle;
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return m_arm.getPivotEncoder() > m_endAngle-.1 && m_arm.getPivotEncoder() < m_endAngle+.1;
    }
}
