// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristUpCmd extends Command {
  /** Creates a new WristCmd. */
  private final WristSubsystem m_subsystem;
  private Supplier<Double> speedFunction;
 

  public WristUpCmd(WristSubsystem subsystem, Supplier<Double> speedFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
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
    RobotContainer.wrist.up();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    RobotContainer.wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
