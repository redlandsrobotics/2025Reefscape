// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmSubsystem extends SubsystemBase 
{
  public CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
  DutyCycleEncoder encoder = new DutyCycleEncoder(0);

  public ArmSubsystem() 
  {
    motor.setNeutralMode(NeutralModeValue.Brake);
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
            return encoder.getAbsolutePosition();
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
