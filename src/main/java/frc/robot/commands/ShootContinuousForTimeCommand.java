/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.GathererSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootContinuousForTimeCommand extends SequentialCommandGroup {
  /**
   * Creates a new ShootContinuousForTimeCommand.
   */
  public ShootContinuousForTimeCommand(GathererSubsystem gatherer, MagazineSubsystem mag, ShooterSubsystem shooter, double time) {
    super(
      new GathererOutCommand(gatherer),
      new ParallelDeadlineGroup(
        new WaitCommand(time),
        new SequentialCommandGroup(
          new WaitCommand(ShooterConstants.kContinuousShootSpinUpTime),
          new RunCommand(mag::runMotor, mag)
        ),
        new RunCommand(gatherer::stopGather, gatherer),
        new RunCommand(shooter::runShooter, shooter)
      ),
      new InstantCommand(shooter::stopShooter, shooter),
      new InstantCommand(mag::stopMotor, mag)
    );
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
  }
}
