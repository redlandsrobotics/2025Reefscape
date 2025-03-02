package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;
    private final PIDController drivePidController;
    private final AnalogInput absoluteEncoder;
  	private final boolean absoluteEncoderReversed;
  	private final double absoluteEncoderOffsetRad;
    private double startPosition = 0.0;
   
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
	int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
		this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
		this.absoluteEncoderReversed = absoluteEncoderReversed;
		absoluteEncoder = new AnalogInput(absoluteEncoderId);
    
	
		driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
		turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
	  driveMotor.restoreFactoryDefaults();
    turningMotor.restoreFactoryDefaults();

		driveMotor.setInverted(driveMotorReversed);
		turningMotor.setInverted(turningMotorReversed);
	
		driveEncoder = driveMotor.getEncoder();
		turningEncoder = turningMotor.getEncoder();
	
		driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
		driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
		turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
		turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
	
		turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI );
    
    drivePidController = new PIDController(0.75,1.5, 0);

    //resetEncoders();
  driveMotor.burnFlash();
  turningMotor.burnFlash();


	}

  public void turningtest(){
    turningMotor.set(0.2);
  }
  
  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }

  public double getTurningPosition(){
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity(){
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad(){
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }
  public double driveV(){
    double driveV = driveMotor.getBusVoltage();
    return driveV;
  }

  public double turnV(){
    double turnV = turningMotor.getBusVoltage();
    return turnV;
  }

   public void resetEncoders(){
     driveEncoder.setPosition(0);
     turningEncoder.setPosition(getAbsoluteEncoderRad());
   }

   public void setEncoders(){
    driveEncoder.setPosition(0.002);
   }

  public double returnVoltage(){
    return absoluteEncoder.getVoltage();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(),   new Rotation2d(getAbsoluteEncoderRad())); /*new Rotation2d(getTurningPosition()));//switch between neo encoders and abs encoders
*/
  }

  public SwerveModulePosition getPosition() {
	return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoderRad())/*new Rotation2d(getTurningPosition())*/);//switch between neo encoders and abs encoders
  }

  public void setDesiredState(SwerveModuleState state) {

    if(Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);




    turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad()/*getTurningPosition()*/, state.angle.getRadians()));//switch between neo encoders and abs encoders
  //   SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
  }

  public void setDesiredStateAutoPID(SwerveModuleState state, double startPosition) {
    
   

    if(Math.abs(getDrivePosition() - startPosition) < 0.1){
      stop();
      return;
    }


    state = SwerveModuleState.optimize(state, getState().angle);
    System.out.println("driveMotorVelo" + getDriveVelocity());
    System.out.println("driveMotorPos0" + getDrivePosition());
    System.out.println("StartPosition" + startPosition);
    double output = drivePidController.calculate(getDrivePosition(), startPosition);
    output = output / 10.0;
    driveMotor.set(output / Math.abs(output) * Math.max(output, 0.1)); //TODO: fixme!

    // turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
  //   SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());

  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

    /**
     * Example command factory method.
     *
     * @return a command
     */
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
      // This method will be called once per scheduler run

    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
  }

