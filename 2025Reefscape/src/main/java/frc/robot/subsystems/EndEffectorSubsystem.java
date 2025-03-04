// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new EndEffectorSubsystem. */
  
  public CANSparkFlex top = new CANSparkFlex(9, MotorType.kBrushless);
  public CANSparkFlex bottom = new CANSparkFlex(10, MotorType.kBrushless);

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
