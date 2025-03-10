// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */

  public SparkMax motor = new SparkMax(10, MotorType.kBrushless);
  DutyCycleEncoder encoder2 = new DutyCycleEncoder(1);

  public WristSubsystem() 
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

  public double getDistance()
  {
    return encoder2.get();
  }

  // private double positionOffset = 0.0;

  // public void resetRotation() {
  //     positionOffset = encoder.get();
  // }
  
  // public double getRelativePosition() {
  //     return encoder.get() - positionOffset;
  // }
  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      SmartDashboard.putNumber("WRIST | ", encoder2.get());

  }
}
