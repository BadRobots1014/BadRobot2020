/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SingleFireCommandGroup extends ParallelRaceGroup {
  /**
   * Creates a new SingleFireCommandGroup.
   */
  public SingleFireCommandGroup(ShooterSubsystem m_shooterSubsystem, FeedSubsystem m_feedSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new RunShooterCommand(m_shooterSubsystem),
      new SequentialCommandGroup(
        new WaitUntilCommand(() -> {
          if (m_shooterSubsystem.getDeltaDesiredVelocity() >= -ShooterConstants.kFeedThresholdAngularSpeedDelta
          && m_shooterSubsystem.getDeltaDesiredVelocity() <= ShooterConstants.kFeedThresholdAngularSpeedDelta) {
            return true;
          } else {
            return false;
          }
        }),
        new ParallelRaceGroup(
          new FeedCommand(m_feedSubsystem),
          new WaitUntilCommand(() -> {
            if (m_shooterSubsystem.getDeltaDesiredVelocity() <= ShooterConstants.kShootThresholdAngularSpeedDelta // should be negative
            //&& m_shooterSubsystem.getDeltaDesiredActiveCurrent() >= ShooterConstants.kShootThresholdActiveCurrentDelta
            ) {
              return true;
            } else {
              return false;
            }
          })
        ),
        new WaitCommand(ShooterConstants.kDelay)
      )
    );
  }
}