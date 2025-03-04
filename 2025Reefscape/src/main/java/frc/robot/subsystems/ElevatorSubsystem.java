// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorSubsystem extends SubsystemBase 
{
  public TalonFX left = new TalonFX(1);
  public TalonFX right = new TalonFX(0);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() 
  {

  }

  public void up()
  {
    left.set(0.1); // these numbers might need to be changed... -AOP
    right.set(0.1);
  }

  public void down()
  {
    left.set(-0.1);
    right.set(-0.1);
  }

  public void stop()
  {
    left.set(0);
    right.set(0);
  }

  public void set(double speed)
  {
    left.set(speed);
    right.set(speed);
  }

  public double getDistance()
  {
    return encoder.getDistance();
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
  }
}
