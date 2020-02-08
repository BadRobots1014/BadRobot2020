package frc.robot.commands;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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
  public HoldPlaceCommand(DriveTrainSubsystem driveTrain, GyroProvider gyro) {
    // super(
    //   new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
    //    new DoubleSupplier(){
    //      @Override
    //      public double getAsDouble() {
    //        return 0;}},
    //    targetAngleDegrees,
    //    output -> driveTrain.arcadeDrive(0, output),
    //    driveTrain);
         
  
       super(
        new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
        // Close loop on heading
        driveTrain::getHeading,
        // Set reference to target
        gyro.getHeading(),
        // Pipe output to turn robot
        output -> driveTrain.arcadeDrive(0, output),
        // Require the drive
        driveTrain
        );

        m_driveTrain = driveTrain;
        m_gyro = gyro;
    
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}