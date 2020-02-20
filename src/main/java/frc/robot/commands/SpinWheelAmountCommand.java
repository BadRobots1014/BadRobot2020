/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class SpinWheelAmountCommand extends CommandBase {
  private ControlPanelSubsystem m_controlPanel;
  private int colorsPassed;
  private char lastColor;
  /**
   * Creates a new ShootCommand.
   */
  public SpinWheelAmountCommand(ControlPanelSubsystem subsystem) {
    m_controlPanel = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorsPassed = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    char currentColor = m_controlPanel.getColor();
    if (lastColor != currentColor && currentColor != '?')
    {
        colorsPassed++;
        lastColor = currentColor;
    }
    if (colorsPassed < 24)
    {
        m_controlPanel.spinWheel(.4);
    }
    else
    {
        m_controlPanel.spinWheel(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controlPanel.spinWheel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (colorsPassed >= 24)
    {
        return true;
    }
    else
    {
        return false;
    }
  }
}