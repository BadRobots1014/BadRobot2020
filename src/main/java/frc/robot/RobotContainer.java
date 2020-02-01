/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.GatherCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GathererSubsystem;
import frc.robot.util.GyroProvider;
import frc.robot.util.SparkMaxProvider;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  private final TeleopDriveCommand m_teleopDriveCommand;
  private final GatherCommand m_gatherCommand;
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
    m_teleopDriveCommand = new TeleopDriveCommand(m_driveTrain);
    m_gatherCommand = new GatherCommand(m_gathererSubsystem);
    // Configure the button bindings
    configureButtonBindings();
    configureDriveTrain();
    configureGatherer();
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


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_teleopDriveCommand;
    // An ExampleCommand will run in autonomous
  }
}
