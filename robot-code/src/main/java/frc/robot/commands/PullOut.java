package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;


public class PullOut extends CommandBase {
    private final Telescope m_telescope;    

    public PullOut(Telescope t) {
        m_telescope = t;
        addRequirements();
    }

    @Override
    public void initialize() {
        ArmConstants.translationController.setGoal(0);
    }

    @Override
    public void execute() {
        m_telescope.rotateTelescope(ArmConstants.translationController.calculate(m_telescope.getTelescopeEncoder(), 0));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_telescope.rotateTelescope(0);
    }

  // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_telescope.getTelescopeEncoder()<.1;
    }
}
