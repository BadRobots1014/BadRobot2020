/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeedSubsystem extends SubsystemBase {

  private final TalonSRX m_feeder;


  /**
   * Creates a new ExampleSubsystem.
   */
  public FeedSubsystem(TalonSRX talon) {
    m_feeder = talon;
    m_feeder.setInverted(false);
  }

  public void joystickControl(DoubleSupplier joystickValue) {
    m_feeder.set(ControlMode.PercentOutput, joystickValue.getAsDouble());
  }
}
