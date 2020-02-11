/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.RobotController;

/**
 * An example command that uses an example subsystem.
 */
public class TeleopDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrainSubsystem m_driveTrain; 

  // Initialize these so that it is not empty.
  private DoubleSupplier m_leftDoubleSupplier = () -> 0.0;
  private DoubleSupplier m_rightDoubleSupplier = () -> 0.0;

  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TeleopDriveCommand(DriveTrainSubsystem subsystem) {
    m_driveTrain = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public void setControllerSupplier(DoubleSupplier leftDoubleSupplier, DoubleSupplier rightDoubleSupplier) {
    m_leftDoubleSupplier = leftDoubleSupplier;
    m_rightDoubleSupplier = rightDoubleSupplier;
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = -m_speedLimiter.calculate(m_leftDoubleSupplier.getAsDouble()) * DriveConstants.kMaxSpeed;
    double rot = -m_rotLimiter.calculate(m_rightDoubleSupplier.getAsDouble()) * DriveConstants.kMaxAngularSpeed;

    m_driveTrain.arcadeDrive(xSpeed, rot);//m_controller.getY(Hand.kLeft), m_controller.getY(Hand.kRight));
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