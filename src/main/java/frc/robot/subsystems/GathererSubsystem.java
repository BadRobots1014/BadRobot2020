/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GathererSubsystem extends SubsystemBase {
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(0, 1);
  public static final double kGathererSpeed = .5;

  private final TalonSRX m_gatherer;

  private final ShuffleboardTab m_gathererTab = Shuffleboard.getTab("Gatherer");
  private final NetworkTableEntry m_gathererState = m_gathererTab.add("Gatherer State", false).getEntry(); // False is gatherer in, true is gatherer out

  /**
   * Creates a new ExampleSubsystem.
   */
  public GathererSubsystem(TalonSRX talon) {
    m_gatherer = talon;
    m_gatherer.setInverted(true);
  }

  public void gathererOut(boolean climb) {
    if (climb) {
      m_doubleSolenoid.set(Value.kForward);
    } else {
      m_doubleSolenoid.set(Value.kOff);
    }
    m_gathererState.setBoolean(true);
  }

  public void gathererIn(boolean climb) {
    if (climb) {
      m_doubleSolenoid.set(Value.kReverse);
    } else {
      m_doubleSolenoid.set(Value.kOff);
    }
    m_gathererState.setBoolean(false);
  }

  public void gathererToggle(boolean climb) {
    if (m_gathererState.getBoolean(false)) {
      gathererIn(climb);
    } else {
      gathererOut(climb);
    }
  }

  public void runGatherer() {
    m_gatherer.set(ControlMode.PercentOutput, kGathererSpeed);
  }

  public void runGathererReversed() {
    m_gatherer.set(ControlMode.PercentOutput, -kGathererSpeed);
  }

  public void stopGather() {
    m_gatherer.set(ControlMode.PercentOutput, 0);
  }
}
