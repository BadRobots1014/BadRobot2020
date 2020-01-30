/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.GyroProvider;
import frc.robot.util.SparkMaxProvider;

public class DriveTrainSubsystem extends SubsystemBase {


  private final CANSparkMax m_leftMaster = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
  private final CANSparkMax m_rightMaster = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);;

  private final CANEncoder m_leftEncoder = m_leftMaster.getEncoder();
  private final CANEncoder m_rightEncoder = m_rightMaster.getEncoder();

  private final SpeedControllerGroup m_leftGroup;
  private final SpeedControllerGroup m_rightGroup;

  private final GyroProvider m_gyro;

  private final PIDController m_leftPIDController = new PIDController(5e-5, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(5e-5, 0, 0);

  private final DifferentialDriveKinematics m_kinematics
      = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.151, 2.78, 0.409);


//  // The motors on the left side of the drive.
//   private final SpeedControllerGroup m_leftMotors;

//  // The motors on the right side of the drive.
//   private final SpeedControllerGroup m_rightMotors;

 // The robot's drive
  //private final DifferentialDrive m_drive;
  

  public DriveTrainSubsystem(SparkMaxProvider speedControllerProvider, GyroProvider gyroProvider) {
    m_gyro = gyroProvider;
    gyroProvider.reset();

    m_leftMaster.setInverted(false);
    m_leftMaster.setIdleMode(IdleMode.kBrake);
    m_rightMaster.setInverted(true);
    m_rightMaster.setIdleMode(IdleMode.kBrake);

    m_leftFollower.setIdleMode(IdleMode.kBrake);
    m_leftFollower.follow(m_leftMaster);
    m_rightFollower.setIdleMode(IdleMode.kBrake);
    m_rightFollower.follow(m_rightMaster);

    m_leftGroup = new SpeedControllerGroup(m_leftMaster);
    m_rightGroup = new SpeedControllerGroup(m_rightMaster);

    // TODO: Convert RPM to meters per second conversion
    
    
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    m_odometry = new DifferentialDriveOdometry(getAngle());
    // m_leftMotors = new SpeedControllerGroup(
    //   speedControllerProvider.getSpeedController(DriveConstants.kLeftMotor1Port), 
    //   speedControllerProvider.getSpeedController(DriveConstants.kLeftMotor2Port));
    // m_rightMotors = new SpeedControllerGroup(
    //   speedControllerProvider.getSpeedController(DriveConstants.kRightMotor1Port), 
    //   speedControllerProvider.getSpeedController(DriveConstants.kRightMotor2Port));
  
    //m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  }

  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-m_gyro.getHeading());
  }

   /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    // TODO: These PIDControllers aren't effectively doing anything right now until the velocityConversionFactor is set.
    final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getVelocity(),
        speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getVelocity(),
        speeds.rightMetersPerSecond);
    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void arcadeDrive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  public void directDrive(double xSpeed, double rot) {
    arcadeDrive(xSpeed, rot);
  }

  public void updateOdometry() {
    // TODO: This isn't being used yet.
    m_odometry.update(getAngle(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }


    // public void tankDrive(double leftSpeed, double rightSpeed)
    // {
    //     m_drive.tankDrive(leftSpeed, rightSpeed);    
    // }

    // public void directDrive(double speed, double angle) {
    //     m_drive.arcadeDrive(speed, angle, false);
    // }

    // public void arcadeDrive(double speed, double angle) {
    //     m_drive.arcadeDrive(speed, angle);
    // }

    // public void stop() {
    //     m_drive.stopMotor();
    // }

    /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
    public void setMaxOutput(double maxOutput) {
        //m_drive.setMaxOutput(maxOutput);
    }
}