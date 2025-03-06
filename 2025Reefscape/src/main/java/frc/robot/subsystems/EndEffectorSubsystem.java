// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkBase.ControlType;
import com.revrobotics.SparkBase.IdleMode;
import com.revrobotics.SparkFlex;
import com.revrobotics.SparkLowLevel.MotorType;
import com.revrobotics.SparkLowLevel.PeriodicFrame;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new EndEffectorSubsystem. */
  
  public SparkFlex top = new SparkFlex(9, MotorType.kBrushless);
  public SparkFlex bottom = new SparkFlex(10, MotorType.kBrushless);

  public EndEffectorSubsystem() {}

  public void shoot()
  {
    top.set(0.75); // to be tuned later
    bottom.set(-0.75); // to be tuned later
  }



  public void intake()
  {
    top.set(-0.75); // to be tuned later
    bottom.set(0.75); // to be tuned later

  }

  public void stop()
  {
    top.set(0.0);
    bottom.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
