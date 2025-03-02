// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroHeadingCmd;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  

  

  //commands
  ZeroHeadingCmd zeroheading = new ZeroHeadingCmd(swerveSubsystem); 
  
  

  // Replace with CommandPS4Controller or CommandJoystick if needed

  XboxController controller2 = new XboxController(1);
  PS5Controller controller1 = new PS5Controller(Constants.OIConstants.kDriverControllerPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
		// joystick 1
		swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
       () -> Math.abs(controller1.getRawAxis(OIConstants.kDriverYAxis)) * controller1.getRawAxis(OIConstants.kDriverYAxis),// x and y speed switched up
       () -> Math.abs(controller1.getRawAxis(OIConstants.kDriverXAxis)) * controller1.getRawAxis(OIConstants.kDriverXAxis),
       () ->Math.abs(controller1.getRawAxis(OIConstants.kDriverRotAxis)) * controller1.getRawAxis(OIConstants.kDriverRotAxis),
       () ->!controller1.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));


    NamedCommands.registerCommand("align", new ZeroHeadingCmd(swerveSubsystem));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // joystick 1
    new JoystickButton(controller1, 5).toggleOnTrue(zeroheading);//this line replaces the above
   
    


    // joystick 2
    // new JoystickButton(joystick2, 10).whenPressed(() -> swerveSubsystem.dReset()); // remove this after ONLY FOR AUTO TESTING!!!!
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    

    PathPlannerAuto path = Robot.paths.getSelected();

    return path;   

  }
}
