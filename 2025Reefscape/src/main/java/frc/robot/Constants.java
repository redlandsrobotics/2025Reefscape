// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
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
    public static final double kPTurning = .75; // tune later
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

    public static final int kFrontLeftDriveMotorPort = 4;
    public static final int kBackLeftDriveMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 6;
    public static final int kBackRightDriveMotorPort = 8;

    //make sure motors have matching IDs
    public static final int kFrontLeftTurningMotorPort = 3;
    public static final int kBackLeftTurningMotorPort = 1;
    public static final int kFrontRightTurningMotorPort = 5;
    public static final int kBackRightTurningMotorPort = 7;

    public static final boolean kFrontLeftTurningEncoderReversed = true; 
    public static final boolean kBackLeftTurningEncoderReversed = true ; 
    public static final boolean kFrontRightTurningEncoderReversed = true; 
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true; //cheeck these for tuning drive
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    //might need to update
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 0;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
    public static final int kBackRightDriveAbsoluteEncoderPort = 3;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false; //f
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false; //f

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 3.7459-6.2831;//-2.1224553755712385 + 5.869953525310789 - 6.26 - 0.07;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 1.557+4.14159;//-5.64 + 11.3 + 0.07;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.352-6.2831;//-4.18514150919256 + 4.18514150919256 + 0.336;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad =  1.1085+3.14159;//-0.025380899196148388 - 4.301724399487101 + 8.64;

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

public static final class ArmConstants {
    public static final double UpperLimit = 0.0; // tweak later
    public static final double LowerLimit = 0.0;
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;

}

public static final class WristConstants{
    public static final double UpperLimit = 0.0;
    public static final double LowerLimit = 0.0;
    public static final double kP = 5;
    public static final double kI = 0.0;
    public static final double kD = 0.5;
}

public static final class ElevaConstants{
    public static final double kP = 0.1;  
    public static final double kI = 0.0;
    public static final double kD = 0.0;
}

public static final class setPoints{
    // OFFSETS
    // Wrist: 0.08
    // Arm: 0.02


    //stow
    public static final double armStow = 0.85;
    public static final double wriStow = 0.77;
    public static final double elaStow = 1;

    //L1
    public static final double armL1 = 0.78;
    public static final double wriL1 = 0.62;
    public static final double elaL1 = 10.3911;

    //L2
    public static final double armL2 = 0.77;
    public static final double wriL2 = 0.5;
    public static final double elaL2 = 12.75;

    //L3
    public static final double armL3 = 0.77;
    public static final double wriL3 = 0.5;
    public static final double elaL3 = 21;
    //L1
    public static final double armL4 = 0.58;
    public static final double wriL4 = 0.3;
    public static final double elaL4 = 30;

    //ALGPro
    public static final double armALGPro = 0.0;
    public static final double wriALGPro = 0.0;
    public static final double elaALGPro = 0.0;

    //ALG
    public static final double armALG = 0.0;
    public static final double wriALG = 0.0;
    public static final double elaALG = 0.0;

    //ALGIntUpp
    public static final double armALGIntUpp = 0.0;
    public static final double wriALGIntUpp = 0.0;
    public static final double elaALGIntUpp = 0.0;

    //ALGIntLow
    public static final double armALGIntLow = 0.0;
    public static final double wriALGIntLow = 0.0;
    public static final double elaALGIntLow = 0.0;

    //CORInt
    public static final double armCORInt = 0.0;
    public static final double wriCORInt = 0.0;
    public static final double elaCORInt = 0.0;

}

public static final class ElevaConstants2{




public class ElevatorConstants {

  public static final boolean IS_INVERTED = true;

  public static final double TOLERANCE_INCHES = 0.25;

  public static final Distance MAX_HEIGHT = Inches.of(74);

  public static final Distance MIN_HEIGHT = Inches.of(0.0);

  public static final Distance JUST_ABOVE_HARDSTOP = Inches.of(1.0); // set hardstop a bit above 0

  public static final Distance HEIGHT_SWITCH_SLOT0 = Inches.of(20); // FIXME: Update these values
  public static final Distance HEIGHT_SWITCH_SLOT1 = Inches.of(40); // FIXME: Update these values

  public static final double PULLY_CIRCUMFERANCE_INCHES = 5.9055;
  public static final int GEAR_RATIO = 5;

  public static final double ELEVATOR_MASS_KG = 4.5; // FIXEME: Update this value

  public static final int LEAD_MOTOR_ID = 10;
  public static final int FOLLOWER_MOTOR_ID = 11;

  public static final boolean DEBUGGING = true;
  public static final boolean TESTING = true;

  public static final String SUBSYSTEM_NAME = "Elevator";

  public static final double ELEVATOR_LOWERING_VOLTAGE = -2.0; // FIXME: Update this value

  public static final double ELEVATOR_RAISE_SLOW_VOLTAGE = 2.0; // FIXME: Update this value

  public static final double ELEVATOR_LOWERING_SLOW_VOLTAGE = -2.0; // FIXME: Update this value

  // FIXME: Update all K values

  public static final double KP_SLOT0 = 0.0;
  public static final double KI_SLOT0 = 0;
  public static final double KD_SLOT0 = 0;
  public static final double KS_SLOT0 = 0.01;
  public static final double KV_SLOT0 = 0.67505;
  public static final double KA_SLOT0 = 0.027564;
  public static final double KG_SLOT0 = 0.33833;

  public static final double KP_SLOT1 = 40.0;
  public static final double KI_SLOT1 = 0;
  public static final double KD_SLOT1 = 0;
  public static final double KS_SLOT1 = 0.01;
  public static final double KV_SLOT1 = 0.67505;
  public static final double KA_SLOT1 = 0.027564;
  public static final double KG_SLOT1 = 0.33833;

  public static final double KP_SLOT2 = 40.0;
  public static final double KI_SLOT2 = 0;
  public static final double KD_SLOT2 = 0;
  public static final double KS_SLOT2 = 0.01;
  public static final double KV_SLOT2 = 0.67505;
  public static final double KA_SLOT2 = 0.027564;
  public static final double KG_SLOT2 = 0.33833;

  public static final double KV_EXPO = 0.6;

  // was 0.05 with no funnel or climber on robot, caused wheels to leave ground
  // arbitrary increase for now
  public static final double KA_EXPO = 0.2;

  public static final double CRUISE_VELOCITY = 0;

  public static final double STALL_CURRENT = 40.0;

  public enum ReefBranch {
    HARDSTOP,

    L1,
    L2,
    L3,
    L4,

    ALGAE_1,
    ALGAE_2,

    BELOW_ALGAE_1,
    ABOVE_ALGAE_1,

    BELOW_ALGAE_2,
    ABOVE_ALGAE_2
  }

  /*
   * Highest point of each reef branch in inches
   */

  public static final Distance L2_HEIGHT = Inches.of(30); // 1 coral away 35
  public static final Distance L3_HEIGHT = Inches.of(45); // 1 coral away 51
  public static final Distance L4_HEIGHT = Inches.of(71);

  public static final Distance ALGAE1_HEIGHT =
      Inches.of(13.0); // height under is 9 // height of impact is 13
  public static final Distance ALGAE2_HEIGHT =
      Inches.of(28.0); // height under is 24 // height of impact is 28

  public static final Distance ABOVE_ALGAE_2_HEIGHT = Inches.of(34.0);
  public static final Distance BELOW_ALGAE_2_HEIGHT = Inches.of(20.0);

  public static final Distance BELOW_ALGAE_1_HEIGHT = Inches.of(7.0);
  public static final Distance ABOVE_ALGAE_1_HEIGHT = Inches.of(16.0);

  public static final double ELEVATOR_PEAK_CURRENT_LIMIT = 60.0;
}
}

}