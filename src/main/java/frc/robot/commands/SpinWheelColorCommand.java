/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class SpinWheelColorCommand extends CommandBase {
  private ControlPanelSubsystem m_controlPanel;
  private boolean hitColor;
  private String gameData;
  /**
   * Creates a new ShootCommand.
   */
  public SpinWheelColorCommand(ControlPanelSubsystem subsystem) {
    m_controlPanel = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hitColor = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (gameData == null)
    {
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'B' :
                    //Blue case code
                    break;
                case 'G' :
                    //Green case code
                    break;
                case 'R' :
                    //Red case code
                    break;
                case 'Y' :
                    //Yellow case code
                    break;
                default :
                    //This is corrupt data
                    gameData = null;
                    System.out.println("Game Data Is Corrupt");
                    break;
            }
        }
        else
        {
            //not got color code yet
            System.out.println("No Game Data Found");
            gameData = null;
        }
    }
    else
    {

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
    return hitColor;
  }
}