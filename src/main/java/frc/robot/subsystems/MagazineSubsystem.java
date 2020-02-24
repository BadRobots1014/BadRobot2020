/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MagazineConstants;

public class MagazineSubsystem extends SubsystemBase {
  private final TalonSRX m_magazineMotor = new TalonSRX(MagazineConstants.kMotorPort);
  private final AnalogInput m_sensor = new AnalogInput(MagazineConstants.kSensorPort);
  /**
   * Creates a new MagazineSubsystem.
   */
  public MagazineSubsystem() {

  }

  public void runMotor() {
    m_magazineMotor.set(ControlMode.PercentOutput, MagazineConstants.kMaxSpeed);
  }

  public void stopMotor() {
    m_magazineMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean getSensorStatus() { // true is unobstructed
    if (m_sensor.getValue() > 300) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Magazine Sensor State", m_sensor.getValue());
    // This method will be called once per scheduler run
  }
}
