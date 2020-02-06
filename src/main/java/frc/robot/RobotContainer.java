/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.GatherCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.GathererSubsystem;
import frc.robot.util.GyroProvider;
import frc.robot.util.SparkMaxProvider;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.constraint.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem m_driveTrain;  
  private final GathererSubsystem m_gathererSubsystem;
  private final FeedSubsystem m_feedSubsystem;

  private final TeleopDriveCommand m_teleopDriveCommand;
  private final GatherCommand m_gatherCommand;
  private final FeedCommand m_feedCommand;

  private final XboxController m_driverController = new XboxController(OIConstants.kDriverController);
  private final XboxController m_attachmentsController = new XboxController(OIConstants.kAttachmentsController);


  private final GyroProvider m_gyroProvider;
  private final SparkMaxProvider m_speedControllerProvider;



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    boolean isReal = Robot.isReal();
    m_gyroProvider = new GyroProvider(isReal);
    m_speedControllerProvider = new SparkMaxProvider(isReal);

    m_driveTrain = new DriveTrainSubsystem(m_speedControllerProvider, m_gyroProvider);
    m_gathererSubsystem = new GathererSubsystem(new TalonSRX(34));
    m_feedSubsystem = new FeedSubsystem(new TalonSRX(21));
    m_teleopDriveCommand = new TeleopDriveCommand(m_driveTrain);
    m_gatherCommand = new GatherCommand(m_gathererSubsystem);
    m_feedCommand = new FeedCommand(m_feedSubsystem);
    // Configure the button bindings
    configureButtonBindings();
    configureDriveTrain();
    configureGatherer();
    configureFeeder();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings()
  {
    DoubleSupplier leftYJoystick = () -> m_driverController.getY(Hand.kLeft);
    DoubleSupplier leftAttatchmentJoystick = () -> m_attachmentsController.getY(Hand.kLeft);

    DoubleSupplier rightJoystick = () -> {
      if (Math.abs(m_driverController.getX(Hand.kRight)) > 0.1) {
        return m_driverController.getX(Hand.kRight);
      } else {
        return 0;
      }
    };

    m_teleopDriveCommand.setControllerSupplier(leftYJoystick, rightJoystick);

    m_gatherCommand.setControllerSupplier(leftAttatchmentJoystick);
    

    new JoystickButton(m_driverController, Button.kBumperRight.value)
    .whenPressed(() -> m_driveTrain.setMaxOutput(0.25))
    .whenReleased(() -> m_driveTrain.setMaxOutput(1));

    //new JoystickButton(m_driverController, Button.kBumperLeft.value).whenHeld(new DriveStraight(leftYJoystick, m_gyroProvider, m_driveTrain));
    new JoystickButton(m_driverController, Button.kA.value)
    .whenPressed(new TurnCommand(m_driveTrain, m_gyroProvider, 90, Math.PI/2, 10));

  }

  private void configureDriveTrain()
  {
    m_driveTrain.setDefaultCommand(m_teleopDriveCommand);
  }

  private void configureGatherer() {
    m_gathererSubsystem.setDefaultCommand(m_gatherCommand);
  }

  private void configureFeeder() {
    m_feedSubsystem.setDefaultCommand(m_feedCommand);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            m_driveTrain.getDriveKinematics(),
            10);

     // Create config for trajectory
     TrajectoryConfig config =
     new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                          AutoConstants.kMaxAccelerationMetersPerSecondSquared)
         // Add kinematics to ensure max speed is actually obeyed
         .setKinematics(m_driveTrain.getDriveKinematics())
         // Apply the voltage constraint
         .addConstraint(autoVoltageConstraint);

      
    
    // An example trajectory to follow.  All units in meters.
    Trajectory firstTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        firstTrajectory,
        m_driveTrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        m_driveTrain.getDriveKinematics(),
        m_driveTrain::getWheelSpeeds,
        new PIDController(DriveConstants.kLeftP, 0, 0),
        new PIDController(DriveConstants.kRightP, 0, 0),
        // RamseteCommand passes volts to the callback
        m_driveTrain::tankDriveVolts,
        m_driveTrain
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_driveTrain.tankDriveVolts(0, 0));
  }
}
