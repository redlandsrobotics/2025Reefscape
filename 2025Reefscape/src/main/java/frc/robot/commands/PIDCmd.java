// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ElevaConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDCmd extends Command {
  /** Creates a new WristCmd. */
  private final WristSubsystem wristSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final PIDController wristPIDController;
  private final PIDController armPIDController;
  private final PIDController elevaPIDController;
  private final double armSetpoint;
 

  public PIDCmd(WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, double armSetpoint, double wristSetpoint, double elevaSetpoint) {    
    this.wristSubsystem = wristSubsystem;
    this.armSubsystem = armSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSetpoint = armSetpoint;
    this.wristPIDController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    this.armPIDController = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
    this.elevaPIDController = new PIDController(ElevaConstants.kP, ElevaConstants.kI, ElevaConstants.kD);
    wristPIDController.setSetpoint(wristSetpoint);
    armPIDController.setSetpoint(armSetpoint);
    elevaPIDController.setSetpoint(elevaSetpoint);
    addRequirements(armSubsystem);
    addRequirements(wristSubsystem);
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristPIDController.reset();
    armPIDController.reset();
    elevaPIDController.reset();
    System.out.println("PID running");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
    double speedW = wristPIDController.calculate(wristSubsystem.getDistance());
    double speedA = armPIDController.calculate(armSubsystem.getDistance());
    double speedE = elevaPIDController.calculate(elevatorSubsystem.getDistance()) / 3;
    //System.out.println("wrist speed: " + speedW);
    System.out.println("arm speed: " + speedA);
    //System.out.println("eleva speed: " + speedE);
    //System.out.println("arm setpoint: " + armSetpoint);
    //System.out.println("arm speed: " + speedA);
    //System.out.println("eleva speed: " + speedE);
    //wristSubsystem.set(-speedW);
    armSubsystem.set(speedA);
    //elevatorSubsystem.set(speedE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    wristSubsystem.set(0);
    armSubsystem.set(0);
    elevatorSubsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
