/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.GyroProvider;
import frc.robot.Robot;

public class AutoAimCommand extends TurnCommand {
  private final DriveTrainSubsystem m_driveTrain;
    private final GyroProvider m_gyro;
    private double m_startHeading, m_endHeading;
    private boolean m_deltaHeadingDirection; // true is clockwise
    private final double m_angle, m_speed;
    
    private double centerX = Robot.getCenterX();
  /**
   * Creates a new AutoAimCommand.
   */
  public AutoAimCommand(DriveTrainSubsystem driveTrain, GyroProvider gyro, double speed, double threshold) {
    super(driveTrain, gyro, speed, speed, threshold);
    m_driveTrain = driveTrain;
    m_gyro = gyro;
    m_angle = (5 / 13) * (centerX - 65);
    m_speed = speed;
    // m_threshold = threshold;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
