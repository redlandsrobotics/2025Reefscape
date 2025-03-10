// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import com.ctre.phoenix6.*;

import frc.robot.RobotContainer;
import frc.robot.Constants.ElevaConstants2;

import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.ElevaConstants2.ElevatorConstants;







public class ElevatorSubsystem extends SubsystemBase 
{


  private MotionMagicExpoVoltage leadPositionRequest;
  private DynamicMotionMagicVoltage leadPositionRequestDown;
  private VoltageOut leadVoltageRequest;
  private StatusSignal<Current> leadStatorCurrent;
  private StatusSignal<Current> followerStatorCurrent;
  private StatusSignal<Voltage> leadVoltageSupplied;
  private StatusSignal<Voltage> followerVoltageSupplied;

  private StatusSignal<Current> leadSupplyCurrent;
  private StatusSignal<Current> followerSupplyCurrent;

  private StatusSignal<Angle> elevatorPositionStatusSignal;

  private StatusSignal<Temperature> elevatorLeadTempStatusSignal;
  private StatusSignal<Temperature> elevatorFollowerTempStatusSignal;

  private StatusSignal<AngularVelocity> elevatorVelocityStatusSignal;

  private double localPosition = 0.0;


  // private ElevatorSystemSim elevatorSystemSim;

  public TalonFX elevatorMotorLead = new TalonFX(1);
  public TalonFX elevatorMotorFollower = new TalonFX(0);
  TalonFXConfiguration elevatorLeftCfg = new TalonFXConfiguration();
  
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() 
  {
    // leadStatorCurrent = elevatorMotorLead.getStatorCurrent();
    // followerStatorCurrent = elevatorMotorFollower.getStatorCurrent();

    // leadVoltageSupplied = elevatorMotorLead.getMotorVoltage();
    // followerVoltageSupplied = elevatorMotorFollower.getMotorVoltage();

    // leadSupplyCurrent = elevatorMotorLead.getSupplyCurrent();
    // followerSupplyCurrent = elevatorMotorFollower.getSupplyCurrent();

    // elevatorPositionStatusSignal = elevatorMotorLead.getPosition();

    // elevatorLeadTempStatusSignal = elevatorMotorLead.getDeviceTemp();
    // elevatorFollowerTempStatusSignal = elevatorMotorFollower.getDeviceTemp();

    // elevatorVelocityStatusSignal = elevatorMotorLead.getVelocity();

    // leadPositionRequest = new MotionMagicExpoVoltage(0);
    // leadPositionRequestDown = new DynamicMotionMagicVoltage(0, 10, 100, 500);
    // leadVoltageRequest = new VoltageOut(0);

    configElevatorMotorLead(elevatorMotorLead);
    configElevatorMotorFollower(elevatorMotorFollower);

    elevatorMotorFollower.setControl(new Follower(elevatorMotorLead.getDeviceID(), true));
    
    /* 
        elevatorSystemSim =
        new ElevatorSystemSim(
            elevatorMotorLead,
            ElevatorConstants.IS_INVERTED,
            ElevatorConstants.GEAR_RATIO,
            ElevatorConstants.ELEVATOR_MASS_KG,
            Units.inchesToMeters(ElevatorConstants.PULLY_CIRCUMFERANCE_INCHES / (Math.PI * 2)),
            ElevatorConstants.MIN_HEIGHT.in(Meters),
            ElevatorConstants.MAX_HEIGHT.in(Meters),
            0.0,
            ElevatorConstants.SUBSYSTEM_NAME);
    */
  }

  public void up()
  {
    elevatorMotorLead.set(0.1); // these numbers might need to be changed... -AOP
    elevatorMotorFollower.set(0.1);
  }

  public void down()
  {
    elevatorMotorLead.set(-0.1);
    elevatorMotorFollower.set(-0.1);
  }

  public void stop()
  {
    elevatorMotorLead.set(0);
    elevatorMotorFollower.set(0);
  }

  public void set(double speed)
  {
    elevatorMotorLead.set(speed);
    elevatorMotorFollower.set(speed);
  }

  public double getDistance() {
    return elevatorMotorLead.getPosition().getValueAsDouble();
}
  
public boolean isAtPositionSetpoint(double position) {
  return Math.abs(elevatorMotorLead.getPosition().getValueAsDouble() - position) < 0.2; // 0.2 pivot error
}

  private void configElevatorMotorLead(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    MotionMagicConfigs leadMotorConfig = config.MotionMagic;

    config.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO;

    // config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.ELEVATOR_PEAK_CURRENT_LIMIT;
    // config.CurrentLimits.SupplyCurrentLowerLimit = ElevatorConstants.ELEVATOR_PEAK_CURRENT_LIMIT;
    // config.CurrentLimits.SupplyCurrentLowerTime = 0;
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.ELEVATOR_PEAK_CURRENT_LIMIT;
    // config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.Inverted =
    ElevatorConstants.IS_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // config.Slot0.kP = ElevatorConstants.KP_SLOT0;
    // config.Slot0.kI = ElevatorConstants.KI_SLOT0;
    // config.Slot0.kD = ElevatorConstants.KD_SLOT0;
    // config.Slot0.kS = ElevatorConstants.KS_SLOT0;
    // config.Slot0.kV = ElevatorConstants.KV_SLOT0;
    // config.Slot0.kA = ElevatorConstants.KA_SLOT0;
    // config.Slot0.kG = ElevatorConstants.KG_SLOT0;

    // config.Slot0.withGravityType(GravityTypeValue.Elevator_Static);

    // config.Slot1.kP = ElevatorConstants.KP_SLOT1;
    // config.Slot1.kI = ElevatorConstants.KI_SLOT1;
    // config.Slot1.kD = ElevatorConstants.KD_SLOT1;
    // config.Slot1.kS = ElevatorConstants.KS_SLOT1;
    // config.Slot1.kV = ElevatorConstants.KV_SLOT1;
    // config.Slot1.kA = ElevatorConstants.KA_SLOT1;
    // config.Slot1.kG = ElevatorConstants.KG_SLOT1;

    // config.Slot1.withGravityType(GravityTypeValue.Elevator_Static);

    // config.Slot2.kP = ElevatorConstants.KP_SLOT2;
    // config.Slot2.kI = ElevatorConstants.KI_SLOT2;
    // config.Slot2.kD = ElevatorConstants.KD_SLOT2;
    // config.Slot2.kS = ElevatorConstants.KS_SLOT2;
    // config.Slot2.kV = ElevatorConstants.KV_SLOT2;
    // config.Slot2.kA = ElevatorConstants.KA_SLOT2;
    // config.Slot2.kG = ElevatorConstants.KG_SLOT2;

    // config.Slot2.withGravityType(GravityTypeValue.Elevator_Static);
    /* 
    leadMotorConfig.MotionMagicExpo_kA = kAExpo.get();
    leadMotorConfig.MotionMagicExpo_kV = kVExpo.get();

    leadMotorConfig.MotionMagicCruiseVelocity = cruiseVelocity.get();

    */

    // configure a hardware limit switch that zeros the elevator when lowered; there is no hardware
    // limit switch, but we will set it using a control request
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
    config.HardwareLimitSwitch.ReverseLimitEnable = true;
/* 
    Phoenix6Util.applyAndCheckConfiguration(elevatorMotorLead, config, configAlert);

    FaultReporter.getInstance()
        .registerHardware(ElevatorConstants.SUBSYSTEM_NAME, "Elevator Motor Lead", motor);
  */
  }

  public void configElevatorMotorFollower(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;


  }


  /* 
  public double getRotation()
  {
    if (left.isAlive() && right.isAlive())
    {
      // ??????
    }

    else 
    {
      return 0.0;
    }
  }
    public double getRotation() // returns absolute position relative to reset
  {
    if (encoder.isConnected())
    {
            return encoder.get();
    }
    else {  
      return 0.0;
    }
  }

  public double getDistance()
  {
    return encoder.getDistance();
  }

  public void resetRotation() // sets current rotation to 0.0
  {
    encoder.reset();
  }

  public double GetPositionOffset() // get the position offset from when the encoder was reset
  {
    return encoder.getPositionOffset();
  }

  public void SetPositionOffset(double dub)
  {
      encoder.setPositionOffset(dub); // set the position offset to double

  }
    */

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

     SmartDashboard.putNumber("ELEVA | ", RobotContainer.elevator.getDistance());

  }
}
