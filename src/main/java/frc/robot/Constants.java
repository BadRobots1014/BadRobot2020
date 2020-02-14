/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class OIConstants {
        public static int kDriverController = 0;
        public static int kAttachmentsController = 1;
    }

    public static final class DriveConstants {
        public static final int kRightMotor1Port = 1;
        public static final int kRightMotor2Port = 3;
        public static final int kLeftMotor1Port = 2;
        public static final int kLeftMotor2Port = 4;
        public static final int kGathererMotor = 5;
        public static final int kFeederMotor = 6;

        public static final double kStabilizationP = 1;
        public static final double kStabilizationI = 0.5;
        public static final double kStabilizationD = 0;

        public static final double kMaxSpeed = 4.0; // meters per second
        public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
        public static final double kGathererSpeed = .5;
      
        public static final double kTrackWidth = 0.689796288; // meters
        public static final double kWheelDiameter = 0.1524; // meters
        //public static final double kPDriveVel = 1.0;
        public static final double kDriveGearing = 10.75;
        
        public static final double ksVolts = 0.151;
        public static final double kvVoltSecondsPerMeter = 2.78;
        public static final double kaVoltSecondsSquaredPerMeter = 0.409;

        public static final double kLeftP = 14.5;
        public static final double kRightP = 14.5;

        public static final double kTurnP = .05;
        public static final double kTurnI = 0;
        public static final double kTurnD = .000001;
        public static final boolean kGyroReversed = true;

        public static final double kTurnToleranceDeg = 10;
        public static final double kTurnRateToleranceDegPerS = 10;//degree per sencond

    }

    public static final class AutoConstants{
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class LEDConstants{
        public static final int kLEDPwmPort = 9;
        public static final int kLEDStrandLength = 150;
    }
}
