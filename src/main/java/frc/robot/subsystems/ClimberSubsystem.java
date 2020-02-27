/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  //private final Solenoid m_solenoid = new Solenoid(0);
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(2, 3);
  private final CANSparkMax m_winch = new CANSparkMax(ClimberConstants.kWinchMotor, MotorType.kBrushless);
  //private final ShuffleboardTab m_gathererTab = Shuffleboard.getTab("Climber");
  //private final NetworkTableEntry m_climberState = m_gathererTab.add("Gatherer State", false).getEntry(); // False is gatherer in, true is gatherer out
  private boolean m_climberState = false;
  /**
   * Creates a new Climber.
   */
  public ClimberSubsystem() {
    m_winch.restoreFactoryDefaults();
    m_winch.setInverted(false);
    m_winch.setIdleMode(IdleMode.kBrake);
    //m_winch.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);
  }

  public void climberUp() {
    m_doubleSolenoid.set(Value.kForward);
    m_climberState = true;
  }

  public void climberDown() {
    m_doubleSolenoid.set(Value.kReverse);
    m_climberState = false;
  }

  public void climberToggle() {
    System.out.println("Get State:" + m_doubleSolenoid.get());
    if (!m_climberState) {
      climberUp();
    } else {
      climberDown();
    }
  }
  public void stop() {
    m_winch.set(0);
  }
  
  public void runWinch(){
    m_winch.set(ClimberConstants.kWinchSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}