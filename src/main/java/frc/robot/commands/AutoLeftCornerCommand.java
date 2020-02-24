/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.GathererSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.util.GyroProvider;
import frc.robot.util.RamseteUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoLeftCornerCommand extends SequentialCommandGroup {
    /**
     * Creates a new AutoDriveCommandGroup.
     */
    LEDSubsystem m_lights;

    public AutoLeftCornerCommand(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, 
    GathererSubsystem gatherer, MagazineSubsystem magazine, GyroProvider gyro) {
    //m_lights = lights;
    // Before starting, set the pose to 0, -3, because that's where the path starts in the Example that was created.
    addCommands(new TurnCommand(driveTrain, gyro, -135)
                .andThen(() -> driveTrain.stop()),
                new SingleFireCommandGroup(shooter, magazine)
                .andThen(() -> shooter.setZeroSpeed()),
                new TurnCommand(driveTrain, gyro, 135)
                .andThen(() -> driveTrain.stop()), 
                RamseteUtil.getRamseteCommandForPath("paths/RedLeftCollect.wpilib.json", driveTrain)
                .raceWith(new GatherCommand(gatherer))
                .beforeStarting(() -> driveTrain.setPose(new Pose2d(2.912, -0.692, new Rotation2d(0))))
                .andThen(() -> driveTrain.stop())
                .andThen(() -> gatherer.stopGather()),
                RamseteUtil.getRamseteCommandForPath("paths/RedLeftReturn.wpilib.json", driveTrain)
                .andThen(() -> driveTrain.stop()),
                new SingleFireCommandGroup(shooter, magazine)
                .andThen(() -> driveTrain.stop())
    );
  }
}

 