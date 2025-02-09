// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.5); 
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 7.0 / 150.0;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = .5; // tune later
    public static final double kITurning = 0;

}

public static final class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(28); 
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(28); 
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));// If wheels have an x shape, then switch the negatives
            
    //make sure motors have matching IDs

    /*
    public static final int kFrontLeftDriveMotorPort = 5;
    public static final int kBackLeftDriveMotorPort = 7;
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kBackRightDriveMotorPort = 1;

    //make sure motors have matching IDs
    public static final int kFrontLeftTurningMotorPort = 6;
    public static final int kBackLeftTurningMotorPort = 8;
    public static final int kFrontRightTurningMotorPort = 4;
    public static final int kBackRightTurningMotorPort = 2;
     */

    public static final int kFrontLeftDriveMotorPort = 5;
    public static final int kBackLeftDriveMotorPort = 7;
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kBackRightDriveMotorPort = 1;

    //make sure motors have matching IDs
    public static final int kFrontLeftTurningMotorPort = 6;
    public static final int kBackLeftTurningMotorPort = 8;
    public static final int kFrontRightTurningMotorPort = 4;
    public static final int kBackRightTurningMotorPort = 2;

    public static final boolean kFrontLeftTurningEncoderReversed = true; 
    public static final boolean kBackLeftTurningEncoderReversed = true; 
    public static final boolean kFrontRightTurningEncoderReversed = true; 
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true; //cheeck these for tuning drive
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    //might need to update
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 3;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
    public static final int kBackRightDriveAbsoluteEncoderPort = 0;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

   // need to update 
   //below one is only a placeholder!
   /**
    *   public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.2151334967421201;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 2.024245872251926;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.7045035029354978664;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.3559470393253346;

    */
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 3.760 - Math.PI;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.38 - Math.PI;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 2.5462100016265356 ;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad =  4.30 - Math.PI;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 4;//might need to update
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2  * Math.PI;//might need to ypdate

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
            kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 7;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 9;
}


public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverController2Port = 1;

    //update below
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 2;
    public static final int kDriverFieldOrientedButtonIdx = 10; // is options on controller

    public static final double kDeadband = .25;//increase deadband to make drive more smooth
}

public static final class OIAutoConstants {
    public static final double DriveFwdD = 2.8; //rndm numbers change it after testing
    public static final double StrafeD = 1;
    public static final double TurningD = 5;
}


}