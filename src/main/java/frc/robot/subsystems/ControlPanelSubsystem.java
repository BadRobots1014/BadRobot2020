/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;

public class ControlPanelSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher;
  private final TalonSRX m_spinningMotor;

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  public ControlPanelSubsystem(ColorSensorV3 colorSensor, ColorMatch colorMatcher, TalonSRX spinningMotor) {
    m_colorSensor = colorSensor;
    m_colorMatcher = colorMatcher;
    m_spinningMotor = spinningMotor;
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  public char getColor() {
    Color detectedColor = m_colorSensor.getColor();
    char colorChar;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget) {
      colorChar = 'b';
    } else if (match.color == kRedTarget) {
      colorChar = 'r';
    } else if (match.color == kGreenTarget) {
      colorChar = 'g';
    } else if (match.color == kYellowTarget) {
      colorChar = 'y';
    } else {
      colorChar = '?';
    }
    return colorChar;
  }

  public Color getColorRGB() {
    Color detectedColor = m_colorSensor.getColor();
    return detectedColor;
  }
}
