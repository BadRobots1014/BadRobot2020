/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.SparkMaxProvider;

public class DriveTrainSubsystem extends SubsystemBase {

 // The motors on the left side of the drive.
 private final SpeedControllerGroup m_leftMotors;

 // The motors on the right side of the drive.
 private final SpeedControllerGroup m_rightMotors;

 // The robot's drive
 private final DifferentialDrive m_drive;
  

 public DriveTrainSubsystem(SparkMaxProvider speedControllerProvider) {
    m_leftMotors = new SpeedControllerGroup(
      speedControllerProvider.getSpeedController(DriveConstants.kLeftMotor1Port), 
      speedControllerProvider.getSpeedController(DriveConstants.kLeftMotor2Port));
    m_rightMotors = new SpeedControllerGroup(
      speedControllerProvider.getSpeedController(DriveConstants.kRightMotor1Port), 
      speedControllerProvider.getSpeedController(DriveConstants.kRightMotor2Port));
  
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  }

    public void tankDrive(double leftSpeed, double rightSpeed)
    {
        m_drive.tankDrive(leftSpeed, rightSpeed);    
    }

    public void directDrive(double speed, double angle) {
        m_drive.arcadeDrive(speed, angle, false);
    }

    public void arcadeDrive(double speed, double angle) {
        m_drive.arcadeDrive(speed, angle);
    }

    public void stop() {
        m_drive.stopMotor();
    }

    /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }
}