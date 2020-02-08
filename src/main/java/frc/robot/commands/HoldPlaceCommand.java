package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.GyroProvider;

/**
 * A command that will turn the robot to the specified angle.
 */
public class HoldPlaceCommand extends PIDCommand {
  private final DriveTrainSubsystem m_driveTrain;
  private final GyroProvider m_gyro;

  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */

  public HoldPlaceCommand(DriveTrainSubsystem driveTrain, GyroProvider gyro, double setPoint) {
    super(new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
        // Close loop on heading
        driveTrain::getHeading,
        // Set reference to target
        setPoint,
        // Pipe output to turn robot
        output -> {
          double clamped = MathUtil.clamp(-output, -1, 1);
          System.out.println(clamped + "    " + gyro.getHeading());
          driveTrain.arcadeDrive(0, clamped * DriveConstants.kMaxAngularSpeed); 
        },
        // Require the drive
        driveTrain);
    m_driveTrain = driveTrain;
    m_gyro = gyro;
    System.out.println("holdcommand");

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);

  }

  @Override
  public void initialize() {
    reset();
    System.out.println("Initialized Command");
    System.out.println(m_gyro.getHeading());
  }

  public void reset() {
    getController().setSetpoint(m_gyro.getHeading());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}