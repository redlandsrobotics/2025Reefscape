// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
   final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
             DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
             DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
             DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
   

   final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
    );
   final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
    );
    
   final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed
    );

     public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    // public final AHRS gyro = new AHRS(SPI.Port.kOnboardCS0);
    // public final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    

    //private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
               // new Rotation2d(0));
   // private Pose2d pose = new Pose2d();
    public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
                DriveConstants.kDriveKinematics, gyro.getRotation2d(), getModulePositions(), 
                     new Pose2d()); // might need to change according to docs, check https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html

    // private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
    // DriveConstants.kDriveKinematics, gyro.getAngle(), getModulePositions(), 
    //         new Pose2d());


    public SwerveSubsystem() {
        RobotConfig config = null; // Declare outside
/* 
        try {
            config = RobotConfig.fromGUISettings(); // Assign to the existing variable
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Ensure config is initialized before using it
        if (config == null) {
            throw new IllegalStateException("Failed to initialize RobotConfig");
        }
*/
    
        // Configure AutoBuilder last
        /*AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (now correctly passing a method that accepts Pose2d)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // PPHolonomicController for holonomic drive trains
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID constants
            ),
            config, // Robot configuration
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            }, 
    this // Reference to this subsystem to set requirements
);*/
  }

    // Assuming this is a method in your drive subsystem
    //path planner
    

    public void driveRobotRelative(ChassisSpeeds speeds)
    {
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        RobotContainer.swerveSubsystem.setModuleStates(moduleStates);
    }


    // public ChassisSpeeds getRobotRelativSpeeds()
    // {
    //       return 
    // }
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                                                               frontRight.getState(),
                                                               backLeft.getState(),
                                                               backRight.getState());}
    
    
    

    
    

   public void turning(){
    frontRight.turningtest();
   }

    public double getDrive() {
        //double[] drivePositions = {frontLeft.getDrivePosition(), frontRight.getDrivePosition(), backLeft.getDrivePosition(), backRight.getDrivePosition()};
        double drivePosition1 = frontLeft.getDrivePosition(); 
        double drivePosition2 = frontRight.getDrivePosition(); 
        double drivePosition3 = backLeft.getDrivePosition(); 
        double drivePosition4 = backRight.getDrivePosition(); 
        

        double averagePosition = (drivePosition1 + drivePosition2 + drivePosition3 + drivePosition4) / 4;
        
        return averagePosition;

    }

    public void getVoltages(){
        SmartDashboard.putNumber("FrontLeft Turn: ", frontLeft.turnV());
        SmartDashboard.putNumber("FrontRight Turn: ", frontRight.turnV());
        SmartDashboard.putNumber("BackLeft Turn: ", backLeft.turnV());
        SmartDashboard.putNumber("BackRight Turn: ", backRight.turnV());

        SmartDashboard.putNumber("FrontLeft Drive: ", frontLeft.driveV());
        SmartDashboard.putNumber("FrontRight Drive: ", frontRight.driveV());
        SmartDashboard.putNumber("BackLeft Drive: ", backLeft.driveV());
        SmartDashboard.putNumber("BackRight Drive: ", backRight.driveV());
    }

    

    public void getSpeeds() {
        SmartDashboard.putNumber("FrontLeft Drive Speed", frontLeft.getTurningVelocity());
        SmartDashboard.putNumber("frontRight Drive Speed", frontRight.getTurningVelocity());
        SmartDashboard.putNumber("backLeft Drive Speed", backLeft.getTurningVelocity());
        SmartDashboard.putNumber("backRight Drive Speed", backRight.getTurningVelocity());
    }

    public double[] getEncoders() {
        double[] drr = {frontLeft.getDrivePosition(), frontRight.getDrivePosition(), backLeft.getDrivePosition(), backRight.getDrivePosition()};

        SmartDashboard.putNumber("FrontLeft Drive Encoder", frontLeft.getDrivePosition());
        SmartDashboard.putNumber("FrontRight Drive Encoder", frontRight.getDrivePosition());
        SmartDashboard.putNumber("BackLeft Drive Encoder", backLeft.getDrivePosition());
        SmartDashboard.putNumber("BackRight Drive Encoder", backRight.getDrivePosition());
        
        return drr;
    }

    public double getTurn(){
        double turnPosition1 = frontLeft.getTurningPosition(); 
        double turnPosition2 = frontRight.getTurningPosition(); 
        double turnPosition3 = backLeft.getTurningPosition(); 
        double turnPosition4 = backRight.getTurningPosition(); 

        double averagePosition = (turnPosition1 + turnPosition2 + turnPosition3 + turnPosition4) / 4;
      
        return averagePosition;
    }

    public void dReset(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void setD(){
        frontLeft.setEncoders();
        frontRight.setEncoders();
        backLeft.setEncoders();
        backRight.setEncoders();
    }

    // public void resetConstantss() {
    //     frontLeft.resetConstants();
    //     frontRight.resetConstants();
    //     backLeft.resetConstants();
    //     backRight.resetConstants();
    // }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    

    public void zeroHeading() {
           gyro.reset();
    }

    public double getHeading() {
        return -Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

     public Pose2d getPose() {
         return odometer.getPoseMeters();
     }

    //  public Pose2d resetPose() {
    //     return odometer.resetPosition(getHeading(), getModulePositions(), pose );
    //  }hello

    public void resetOdometry(Pose2d pose) {
        // //in case of odomoter problems check this first for debug
             odometer.resetPosition(
                getRotation2d(),
                new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
                },
                pose);
         }

     
         

    public Command exampleMethodCommand() {
        return null;
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        //PPLibTelemetry newTelemetry = new PPLibTelemetry();
        odometer.update(getRotation2d() , getModulePositions()); //in case of odomoter problems check this first for debug
            //  SmartDashboard.putNumber("Robot Heading", getHeading());
            SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
              //getVoltages();
              //getSpeeds();
             // getEncoders();
        SmartDashboard.putNumber("Gyro Heading", getHeading());
        // System.out.printsqtln("test");
        // System.out.println("FL | " + frontLeft.returnVoltage());
        // SmartDashboard.putNumber("FL | ",  frontLeft.getAbsoluteEncoderRad());
  
        // System.out.println("FR | " + frontRight.returnVoltage());
    //    SmartDashboard.putNumber("FR | ", frontRight.getAbsoluteEncoderRad());

        // System.out.println("BL | " + backLeft.returnVoltage());
        // SmartDashboard.putNumber("FL | ", frontLeft.getAbsoluteEncoderRad());
        // SmartDashboard.putNumber("FR | ", frontRight.getAbsoluteEncoderRad());
        // SmartDashboard.putNumber("BL | ", backLeft.getAbsoluteEncoderRad());
        // SmartDashboard.putNumber("BR | ", backRight.getAbsoluteEncoderRad());
    //    System.out.println("FR | " + frontRight.getAbsoluteEncoderRad());
    //    System.out.println("BR | " + backRight.getAbsoluteEncoderRad());
    //      System.out.println("FL | " + frontLeft.getAbsoluteEncoderRad());
    //      System.out.println("BL | " + backLeft.getAbsoluteEncoderRad());
        // System.out.println("BR | " + backRight.returnVoltage());
        // SmartDashboard.putNumber("BR | ", backRight.getAbsoluteEncoderRad());
        // System.out.println("FL |" + frontLeft.getTurningPosition());
        // System.out.println("FL |" + frontRight.getTurningPosition());
        // System.out.println("BL |" + backLeft.getTurningPosition());
        // System.out.println("BR |" + backRight.getTurningPosition());
        //System.out.println("Gyro Value" + getHeading());

        //System.out.println("this" + getDrive());

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); //check this for debug first
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void SetModuleStatesAutoPID(SwerveModuleState[] desiredStates, double[] startPosition) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); //check this for debug first
        frontLeft.setDesiredStateAutoPID(desiredStates[0], startPosition[0]);
        frontRight.setDesiredStateAutoPID(desiredStates[1], startPosition[1]);
        backLeft.setDesiredStateAutoPID(desiredStates[2], startPosition[2]);
        backRight.setDesiredStateAutoPID(desiredStates[3], startPosition[3]);
    }

        @Override
        public void simulationPeriodic() {
            // This method will be called once per scheduler run during simulation
        }

        
    }
