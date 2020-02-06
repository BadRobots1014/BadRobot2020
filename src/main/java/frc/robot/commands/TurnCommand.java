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

public class TurnCommand extends CommandBase {
    private final DriveTrainSubsystem m_driveTrain;
    private final GyroProvider m_gyro;
    private double m_startHeading, m_endHeading;
    private boolean m_deltaHeadingDirection; // true is clockwise
    private final double m_angle, m_speed;
    // private final double m_threshold;
  /**
   * Creates a new TurnCommand.
   */
  public TurnCommand(DriveTrainSubsystem driveTrain, GyroProvider gyro, double angle, double speed, double threshold) {
    m_driveTrain = driveTrain;
    m_gyro = gyro;
    m_angle = angle;
    m_speed = speed;
    // m_threshold = threshold;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startHeading = m_gyro.getHeading();
    m_endHeading = m_gyro.getHeading() + m_angle;
    if ((m_endHeading - m_startHeading) >= 0) {
        m_deltaHeadingDirection = true;
    } else {
        m_deltaHeadingDirection = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_deltaHeadingDirection) {
        if (m_gyro.getHeading() < (m_endHeading)) {
            m_driveTrain.arcadeDrive(0.0, -m_speed);
        }
    } else {
        if (m_gyro.getHeading() > (m_endHeading)) {
            m_driveTrain.arcadeDrive(0.0, m_speed);
        }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_deltaHeadingDirection) {
        if (m_gyro.getHeading() >= (m_endHeading)) {
            return true;
        } else {
            return false;
        }
    } else {
        if (m_gyro.getHeading() <= (m_endHeading)) {
            return true;
        } else {
            return false;
        }
    }
  }
}