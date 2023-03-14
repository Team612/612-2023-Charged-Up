// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LedCommands;
import frc.robot.subsystems.led;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class led4 extends CommandBase {
  /** Creates a new led4. */
  private final led m_led;
  private int t = 0; //per led
  private int r= 255;// led color
  private int b= 0;// led color
  private int g= 0;// led color
  public led4(led l) {
    m_led = l;
    addRequirements(m_led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (t == m_led.getLength()) {
      t = 0;
    }

    m_led.led4(t,r,g,b);
    //trust the logic
    if (r == 255 && b == 0 && g != 255){
    g += 15;
    t++;
    }
    else if (g == 255 && b == 0 && r != 0){
      r -= 15;
      t++;
    }
    else if(r == 0 && g == 255 && b != 255) {
      b += 15;
      t++;
    }
    else if (r == 0 && b == 255 && g != 0){
      g -= 15;
      t++;
    }
    else if (g == 0 && b == 255 && r != 255){
      r += 15;
      t++;
    }
    else if (r == 255 && g == 0 && b != 0){
      b -= 15;
      t++;
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
