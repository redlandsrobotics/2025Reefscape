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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANSparkJNI;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicBoolean;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkAbsoluteEncoder;

public class ArmSubsystem extends SubsystemBase 
{
  public SparkMax motor = new SparkMax(9, MotorType.kBrushless);
  DutyCycleEncoder encoder = new DutyCycleEncoder(0);

  public ArmSubsystem() 
  {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);

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

//   public double getRotation() // returns absolute position relative to reset
//   {
//     if (encoder.isConnected())
//     {
//             return encoder.get();
//     }
//     else {
//       return 0.0;
//     }
//   }

  public double getDistance()
  {
    return encoder.get();
  }

//   private double positionOffset = 0.0;

// public void resetRotation() {
//     positionOffset = encoder.get();
// }

// public double getRelativePosition() {
//     return encoder.get() - positionOffset;
// }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
          SmartDashboard.putNumber("ARM | ", encoder.get());

  } 
}
