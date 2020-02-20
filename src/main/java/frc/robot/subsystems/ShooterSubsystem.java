/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_shooterMotor = new TalonFX(ShooterConstants.kShooterId);
  private final ShuffleboardTab m_shooterTab = Shuffleboard.getTab("Shooter");
  private final NetworkTableEntry m_velocityEntry = m_shooterTab.add("Shooter Velocity", 0).getEntry();
  private final NetworkTableEntry m_currentEntry = m_shooterTab.add("Shooter Current", 0).getEntry();
  /**
   * Creates a new ExampleSubsystem.
   */
  public ShooterSubsystem() {
    m_shooterMotor.setNeutralMode(NeutralMode.Coast);
    m_shooterMotor.setInverted(ShooterConstants.kShooterReversed);
  }

  public void setMaxSpeed() {
    m_shooterMotor.set(TalonFXControlMode.PercentOutput, ShooterConstants.kMaxPercentOutput);
  }

  public void setZeroSpeed() {
    m_shooterMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void updateEntries() {
    m_velocityEntry.setDouble(m_shooterMotor.getSelectedSensorVelocity() / ShooterConstants.kEncoderConstant);
    m_currentEntry.setDouble(m_shooterMotor.getSupplyCurrent());
  }

  // Return the difference between the desired velocity and the current velocity
  // Guages what's going on with the motor.
  public double getDeltaDesiredVelocity() {
    return  m_velocityEntry.getDouble(ShooterConstants.kDesiredAngularSpeed) - ShooterConstants.kDesiredAngularSpeed ;
  }

  public double getDeltaDesiredActiveCurrent() {
    return  m_currentEntry.getDouble(ShooterConstants.kDesiredActiveCurrent) - ShooterConstants.kDesiredActiveCurrent;
  }
  
  @Override
  public void periodic() {
    updateEntries();
  }
  
  /* Once specifics are available
  public void setDesiredVelocity() {
    m_shooterMotor.set(TalonFXControlMode.Velocity, ShooterConstants.kDesiredAngularSpeed * ShooterConstants.kEncoderConstant);
  }
  */

  /**
   * Pertinent information:
   * Encoder: 2048 CPR
   * 1 rpm -> 1.462847143 encoder velocity units
   * Gearing ratio of wheel to motor: (7/3):1
   */

  /**
   * Shooter flow:
   * 
   * Mode 1: Single fire
   * 1. Accelerate the shooter wheel to the desired speed
   * 2. Activate the feeder
   * 3. Wait for the ball to go through--decrease in angular velocity and current spike
   * 4. Chill
   * 
   * Mode 2: Continuous fire
   * 
   */

}
