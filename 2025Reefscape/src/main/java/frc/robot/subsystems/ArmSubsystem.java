// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANSparkJNI;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DriverStation;
import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicBoolean;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

public class ArmSubsystem extends SubsystemBase 
{
  public SparkMax motor = new SparkMax(0, MotorType.kBrushless);
  DutyCycleEncoder encoder = new DutyCycleEncoder(0);

  public ArmSubsystem() 
  {
    motor.setIdleMode(SparkBase.IdleMode.kBrake); 
  }

  public void up()  
  {
    motor.set(0.1); // these numbers might need to be changed... -AOP
  }
  

  public void down()
  {
    motor.set(-0.1);
  }

  public void stop()
  {
    motor.set(0);
  }

  public void set(double speed)
  {
    motor.set(speed);
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
    return encoder.getPosition();
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 
}
