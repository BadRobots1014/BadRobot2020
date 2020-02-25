/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OIConstants.ControllerSetup;
import frc.robot.Constants.AccessoryConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.AimCommand;
import frc.robot.commands.AutoDriveExamplePathCommandGroup;
import frc.robot.commands.AutoLeftCommand;
import frc.robot.commands.AutoLeftCornerCommand;
import frc.robot.commands.AutoMiddleCommand;
import frc.robot.commands.AutoRightCommand;
import frc.robot.commands.GatherCommand;
import frc.robot.commands.HoldPlaceCommand;
import frc.robot.commands.RainbowLedCommand;
import frc.robot.commands.RunShooterCommand;
import frc.robot.commands.SingleFireCommandGroup;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.commands.HoodCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GathererSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.GyroProvider;
import frc.robot.util.SparkMaxProvider;

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
  private final MagazineSubsystem m_magSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  private final TeleopDriveCommand m_teleopDriveCommand;
  private final GatherCommand m_gatherCommand;
  private final RunShooterCommand m_shootCommand;

  private final XboxController m_driverController = new XboxController(OIConstants.kDriverController);
  private final XboxController m_attachmentsController = new XboxController(OIConstants.kAttachmentsController);

  public final GyroProvider m_gyroProvider;
  private final SparkMaxProvider m_speedControllerProvider;
  
  private final LEDSubsystem m_LEDSubsystem;
  private final AddressableLEDBuffer m_LEDBuffer;
  private final AddressableLED m_LED;

  private final AutoDriveExamplePathCommandGroup m_exampleDrive;
  private final AutoLeftCornerCommand m_autoLeftCorner;
  private final AutoLeftCommand m_autoLeft;
  private final AutoMiddleCommand m_autoMiddle;
  private final AutoRightCommand m_autoRight;
  private final RainbowLedCommand m_defaultLedCommand;
  private HoldPlaceCommand m_holdPlaceCommand;
  private AimCommand m_AutoAimCommand;

  private final ShuffleboardTab m_autonomousShuffleboardTab = Shuffleboard.getTab("Autonomous");
  private SendableChooser<Command> m_autonomousChooser;

  private ControllerSetup controllerSetupMode = ControllerSetup.COMPETITION;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    boolean isReal = Robot.isReal();

    m_gyroProvider = new GyroProvider(isReal);
    m_speedControllerProvider = new SparkMaxProvider(isReal);
    m_LED = new AddressableLED(LEDConstants.kLEDPwmPort);
    m_LEDBuffer = new AddressableLEDBuffer(LEDConstants.kLEDStrandLength);
    m_driveTrain = new DriveTrainSubsystem(m_speedControllerProvider, m_gyroProvider);
    m_LEDSubsystem = new LEDSubsystem(m_LED, m_LEDBuffer);
    m_gathererSubsystem = new GathererSubsystem(new TalonSRX(AccessoryConstants.kGathererPort));
    m_shooterSubsystem = new ShooterSubsystem();
    m_magSubsystem = new MagazineSubsystem();

    m_teleopDriveCommand = new TeleopDriveCommand(m_driveTrain);
    m_gatherCommand = new GatherCommand(m_gathererSubsystem);
    m_holdPlaceCommand = new HoldPlaceCommand(m_driveTrain, m_gyroProvider);
    m_AutoAimCommand = new AimCommand(m_driveTrain, m_gyroProvider, Robot::getCenterX);
    m_shootCommand = new RunShooterCommand(m_shooterSubsystem);

    // Configure the button bindings
    m_defaultLedCommand = new RainbowLedCommand(m_LEDSubsystem, m_driverController, m_attachmentsController);
    m_LEDSubsystem.setDefaultCommand(m_defaultLedCommand);
    m_driveTrain.setDefaultCommand(m_teleopDriveCommand);
    configureButtonBindings();
    

    m_exampleDrive = new AutoDriveExamplePathCommandGroup(m_driveTrain);
    m_autoLeftCorner = new AutoLeftCornerCommand(m_driveTrain, m_shooterSubsystem, m_gathererSubsystem, m_magSubsystem, m_gyroProvider);
    m_autoLeft = new AutoLeftCommand(m_driveTrain, m_shooterSubsystem, m_gathererSubsystem, m_magSubsystem);
    m_autoMiddle = new AutoMiddleCommand(m_driveTrain, m_shooterSubsystem, m_gathererSubsystem, m_magSubsystem);
    m_autoRight = new AutoRightCommand(m_driveTrain, m_shooterSubsystem, m_gathererSubsystem, m_magSubsystem);
    // Configure SmartDashboard Tabs
    configureAutonomousTab();


  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings()
  {
    if (controllerSetupMode == ControllerSetup.COMPETITION) {
      // Competition controls
      configureDriverControls();
      configureAttachmentControls();
    } else if (controllerSetupMode == ControllerSetup.SINGLE_CONTROLLER) {
      // Useful for when only a single controller is available
      configureDriverControls();
      configureSingleControllerControls();
    } else if (controllerSetupMode == ControllerSetup.DIAGNOSTIC) {
      // Use this mode to setup manual running of various systems
      configureDiagnosticControls();
    }

  }

  private void configureDriverControls() {
    DoubleSupplier leftYJoystick = () -> m_driverController.getY(Hand.kLeft);
    DoubleSupplier rightJoystick = () -> m_driverController.getX(Hand.kRight);
    BooleanSupplier driverQuickTurn = () -> m_driverController.getTriggerAxis(Hand.kLeft) > 0;
    m_teleopDriveCommand.setControllerSupplier(leftYJoystick, rightJoystick, driverQuickTurn);

    new JoystickButton(m_driverController, Button.kBumperLeft.value)
    .whenPressed(new TurnCommand(m_driveTrain, m_gyroProvider, -5));

    new JoystickButton(m_driverController, Button.kBumperRight.value)
    .whenPressed(new TurnCommand(m_driveTrain, m_gyroProvider, 5));

    //AutoAim Command
    new JoystickButton(m_driverController, Button.kX.value)
    .whileHeld(m_AutoAimCommand);

    new JoystickButton(m_driverController, Button.kB.value)
    .whileHeld(m_holdPlaceCommand);


  }

  private void configureSingleControllerControls() {

    new JoystickButton(m_driverController, Button.kBack.value)
    .toggleWhenPressed(m_gatherCommand);

    new JoystickButton(m_driverController, Button.kY.value)
    .whenHeld(m_shootCommand);
    
    new JoystickButton(m_driverController, Button.kA.value)
    .whenHeld(new SingleFireCommandGroup(m_shooterSubsystem, m_magSubsystem));
  }

  private void configureAttachmentControls()
  {

    new JoystickButton(m_attachmentsController, Button.kBack.value)
    .toggleWhenPressed(m_gatherCommand);

    new JoystickButton(m_attachmentsController, Button.kY.value)
    .whenHeld(m_shootCommand);
    
    new JoystickButton(m_attachmentsController, Button.kA.value)
    .whenHeld(new SingleFireCommandGroup(m_shooterSubsystem, m_magSubsystem));

    new JoystickButton(m_attachmentsController, Button.kX.value)
    .whenHeld(new HoodCommand(m_shooterSubsystem));
    
  }

  private void configureDiagnosticControls() {
    // TODO
  }

  private void configureAutonomousTab()
  {
    m_autonomousChooser = new SendableChooser<Command>();
    m_autonomousChooser.setDefaultOption("Example Path Drive", m_exampleDrive);
    m_autonomousChooser.addOption("Far Left", m_autoLeftCorner);
    m_autonomousChooser.addOption("Left", m_autoLeft);
    m_autonomousChooser.addOption("Middle", m_autoMiddle);
    m_autonomousChooser.addOption("Right", m_autoRight);
    m_autonomousChooser.addOption("Hold Place", new PrintCommand("Not doing anything for auton"));


    m_autonomousShuffleboardTab.add("Autonomous Chooser", m_autonomousChooser);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();    
  }
}