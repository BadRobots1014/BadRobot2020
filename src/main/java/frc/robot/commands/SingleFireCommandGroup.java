/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.GathererSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SingleFireCommandGroup extends ParallelRaceGroup {
  /**
   * Creates a new SingleFireCommandGroup.
   */
  public SingleFireCommandGroup(ShooterSubsystem shooterSubsystem, MagazineSubsystem magSubsystem, GathererSubsystem gathererSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ParallelCommandGroup(
        new GathererOutCommand(gathererSubsystem),
        new RunCommand(() -> gathererSubsystem.stopGather(), gathererSubsystem)
      ),
      new RunShooterCommand(shooterSubsystem),
      new SequentialCommandGroup(
        new WaitUntilCommand(() -> {
          if (shooterSubsystem.getDeltaDesiredVelocity() >= -ShooterConstants.kFeedThresholdAngularSpeedDelta
          && shooterSubsystem.getDeltaDesiredVelocity() <= ShooterConstants.kFeedThresholdAngularSpeedDelta) {
            return true;
          } else {
            return false;
          }
        }),
        new ParallelRaceGroup(
          new ParallelCommandGroup(
            new RunMagazineMotorCommand(magSubsystem).withTimeout(1.0),
            new PerpetualCommand(new InstantCommand())
          // new SequentialCommandGroup(
          //   new WaitCommand(2.0),)
          //   new ParallelRaceGroup(
          //     new RunGathererReversedCommand(gathererSubsystem),
          //     new WaitCommand(1.0)
          //   ),
          //   new GatherCommand(gathererSubsystem)
          // ),
          ),
          new WaitUntilCommand(() -> {
            if (shooterSubsystem.getDeltaDesiredVelocity() <= ShooterConstants.kShootThresholdAngularSpeedDelta // should be negative
            //&& m_shooterSubsystem.getDeltaDesiredActiveCurrent() >= ShooterConstants.kShootThresholdActiveCurrentDelta
            ) {
              return true;
            } else {
              return false;
            }
          })//.withTimeout(6.0)
        ),
        new WaitCommand(ShooterConstants.kDelay)
      )
    );
  }
}