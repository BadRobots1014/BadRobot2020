/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * An example command that uses an example subsystem.
 */
public class TeleopDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrainSubsystem m_driveTrain;  

  // Initialize these so that it is not empty.
  private DoubleSupplier m_leftDoubleSupplier = () -> 0.0;
  private DoubleSupplier m_rightDoubleSupplier = () -> 0.0;

  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);

  public static final double kDefaultDeadband = 0.02;
  public static final double kDefaultMaxOutput = 1.0;
  private static final boolean kSquareInputs = true; 

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
    double xSpeed = m_leftDoubleSupplier.getAsDouble();
    double zRotation = m_rightDoubleSupplier.getAsDouble();
    
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    xSpeed = applyDeadband(xSpeed, kDefaultDeadband);

    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    zRotation = applyDeadband(zRotation, kDefaultDeadband);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (kSquareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double slewedXSpeed = -m_speedLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeed;
    double slewedRotation = -m_rotLimiter.calculate(zRotation) * DriveConstants.kMaxAngularSpeed;

    m_driveTrain.arcadeDrive(slewedXSpeed, slewedRotation);//m_controller.getY(Hand.kLeft), m_controller.getY(Hand.kRight));
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

    /**
   * Returns 0.0 if the given value is within the specified range around zero. The remaining range
   * between the deadband and 1.0 is scaled from 0.0 to 1.0.
   *
   * @param value    value to clip
   * @param deadband range around zero
   */
  protected double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}