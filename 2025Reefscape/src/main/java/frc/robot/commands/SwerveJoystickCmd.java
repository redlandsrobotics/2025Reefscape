package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.RobotContainer;

public class SwerveJoystickCmd extends Command{
    private SwerveSubsystem swerveSubsystem;
    private Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private Supplier<Boolean> fieldOrientedFunction;
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, 
    Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);

    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        //inputs
        // if (RobotContainer.elevator.getDistance() > value){divide speed by two}
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get()*0.825;

        
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // System.out.println(xSpeed);
        // System.out.println(ySpeed);
        // System.out.println(turningSpeed);

        //make drive smooth 
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        //turningSpeed = turningLimiter.calculate(turningSpeed);


        // //comment
        // xSpeed = -0.2;
        // ySpeed = 0;
        // turningSpeed = 0;

        // System.out.println("|" + xSpeed);
        // System.out.println("|" + ySpeed);
        // System.out.println("|" + turningSpeed);

        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()){
        //     // speed, not displacement. Speeds are recorded by a the joystick in RobotContainer
        //     /**
        //      * Passed into the constructor: 
        //      * 1. x, y speed relative to the field 
        //      * 2. turningSpeed: angular momentum
        //      * 3. swerveSubsystem.getRotation2d(): gyro angle
        //      * */ 
            SmartDashboard.putBoolean("field oreinted", true);
           chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
           //chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-0.2, 0, 0, swerveSubsystem.getRotation2d());
        } 
        else {
            SmartDashboard.putBoolean("field oreinted", false);
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // Kinematics turns ChassisSpeeds into swerve states 
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }

    //}

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

