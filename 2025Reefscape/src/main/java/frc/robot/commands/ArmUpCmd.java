// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmUpCmd extends Command {
  /** Creates a new ElevatorCmd. */
  private final ArmSubsystem m_subsystem;
  private Supplier<Double> speedFunction;


  public ArmUpCmd(ArmSubsystem subsystem, Supplier<Double> speedFunction) 
  {
    m_subsystem = subsystem;
    this.speedFunction = speedFunction;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double realTimeSpeed = speedFunction.get();

    if(realTimeSpeed<0.05 && realTimeSpeed>-0.05)
    {
      RobotContainer.arm.stop();
    }
    if(realTimeSpeed < -0.05)
    {
      RobotContainer.arm.down();
    }
    if(realTimeSpeed> 0.05)
    {
      RobotContainer.arm.up();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    RobotContainer.arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
