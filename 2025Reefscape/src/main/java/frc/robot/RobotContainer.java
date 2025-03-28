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

import edu.wpi.first.wpilibj.Joystick;
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

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.ElevatorUpCmd;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.commands.WristUpCmd;
import frc.robot.commands.WristDownCmd;
import frc.robot.commands.ArmUpCmd;
import frc.robot.commands.PIDCmd;
import frc.robot.commands.EndEffectorIntakeCmd;
import frc.robot.commands.EndEffectorOutputCmd;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.Constants.setPoints;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public static ElevatorSubsystem elevator = new ElevatorSubsystem();
  public static WristSubsystem wrist = new WristSubsystem();
  public static EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
  public static ArmSubsystem arm = new ArmSubsystem();
  /* 
  public static VisionSubsystem vision = new VisionSubsystem();
  public static ShooterSubsystem shooter = new ShooterSubsystem();
  public static GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem();
  public static ArmSubsystem arm = new ArmSubsystem();
  public static InnerShooterSubsystem innerShooter = new InnerShooterSubsystem();
  */
  

  

  //commands
  ZeroHeadingCmd zeroheading = new ZeroHeadingCmd(swerveSubsystem); 
  private static PIDCmd stowPID = new PIDCmd(wrist, arm, elevator, setPoints.armStow, setPoints.wriStow, setPoints.elaStow);
  private static PIDCmd L1PID = new PIDCmd(wrist, arm, elevator, setPoints.armL1, setPoints.wriL1, setPoints.elaL1);
  private static PIDCmd L2PID = new PIDCmd(wrist, arm, elevator, setPoints.armL2, setPoints.wriL2, setPoints.elaL2);
  private static PIDCmd L3PID = new PIDCmd(wrist, arm, elevator, setPoints.armL3, setPoints.wriL3, setPoints.elaL3);
  private static PIDCmd L4PID = new PIDCmd(wrist, arm, elevator, setPoints.armL4, setPoints.wriL4, setPoints.elaL4);
  private static PIDCmd ALGProPID  = new PIDCmd(wrist, arm, elevator, setPoints.armALGPro, setPoints.wriALGPro, setPoints.elaALGPro);
  private static PIDCmd ALGPID = new PIDCmd(wrist, arm, elevator, setPoints.armALG, setPoints.wriALG, setPoints.elaALG);
  private static PIDCmd ALGIntUppPID = new PIDCmd(wrist, arm, elevator, setPoints.armALGIntUpp, setPoints.wriALGIntUpp, setPoints.elaALGIntUpp);
  private static PIDCmd ALGIntLowPID = new PIDCmd(wrist, arm, elevator, setPoints.armALGIntLow, setPoints.wriALGIntLow, setPoints.elaALGIntLow);
  private static PIDCmd ALGCorPID = new PIDCmd(wrist, arm, elevator, setPoints.armCORInt, setPoints.wriCORInt, setPoints.elaCORInt);
  
  private static WristUpCmd WristUp = new WristUpCmd(wrist);
  private static WristDownCmd WristDown = new WristDownCmd(wrist);
  private static EndEffectorIntakeCmd EndEffectorIntake = new EndEffectorIntakeCmd(endEffector);
  private static EndEffectorOutputCmd EndEffectorOutake = new EndEffectorOutputCmd(endEffector);





  //Naming commands
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //XboxController controller1 = new XboxController(Constants.OIConstants.kDriverControllerPort);
  XboxController controller2 = new XboxController(1);
  PS5Controller controller1 = new PS5Controller(Constants.OIConstants.kDriverControllerPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
		// joystick 1
		swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
       () -> -Math.abs(controller1.getRawAxis(OIConstants.kDriverYAxis)) * controller1.getRawAxis(OIConstants.kDriverYAxis),// x and y speed switched up
       () -> -Math.abs(controller1.getRawAxis(OIConstants.kDriverXAxis)) * controller1.getRawAxis(OIConstants.kDriverXAxis),
       () ->Math.abs(controller1.getRawAxis(OIConstants.kDriverRotAxis)) * controller1.getRawAxis(OIConstants.kDriverRotAxis),
       () ->!controller1.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    arm.setDefaultCommand(new ArmUpCmd(arm, () -> -controller2.getRawAxis(1)));
    elevator.setDefaultCommand(new ElevatorUpCmd(elevator, () -> -controller2.getRawAxis(5)));
    // shooter.setDefaultCommand(new ShootCmd(shooter,  () -> controller1.getRawAxis(4), () -> controller1.getRawAxis(3)));
    // NamedCommands.registerCommand("pos1", new ArmPIDCmd(arm, ArmConstants.pos1));
    // NamedCommands.registerCommand("stow", new ArmPIDCmd(arm, ArmConstants.Stow));
    // NamedCommands.registerCommand("shoot", new AutoShootCmd(shooter));
    // NamedCommands.registerCommand("LRShoot", new LRShootCmd(innerShooter));
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
    //new JoystickButton(controller1, 5).onTrue(zeroheading);
    new JoystickButton(controller1, 5).toggleOnTrue(zeroheading);//this line replaces the above
    new JoystickButton(controller2, 1).whileTrue(L1PID);
    // new JoystickButton(controller2, 2).whileTrue(L2PID);
    new JoystickButton(controller2, 3).whileTrue(L3PID);
    new JoystickButton(controller2, 4).whileTrue(L4PID);
    // new JoystickButton(controller2, 8).whileTrue(ALGIntUppPID);
    // //new JoystickButton(controller2, 6).whileTrue(stowPID);
    // new JoystickButton(controller2, 7).whileTrue(ALGIntLowPID);
    // new JoystickButton(controller2, 9).whileTrue(ALGPID);
    // new JoystickButton(controller2, 10).whileTrue(ALGProPID);
    // new JoystickButton(controller2,11).whileTrue(ALGCorPID);
    new JoystickButton(controller2, 5).whileTrue(WristUp);
    new JoystickButton(controller2, 6).whileTrue(WristDown);
    new JoystickButton(controller1, 5).whileTrue(EndEffectorIntake);
    new JoystickButton(controller1, 6).whileTrue(EndEffectorOutake);
    new JoystickButton(controller2, 6).whileTrue(stowPID);
    new JoystickButton(controller2, 2).whileTrue(L2PID);



    


    //new JoystickButton(controller1, 1).toggleOnTrue(align);
    // new JoystickButton(controller1, 6).whileTrue(LRshoot);
    // new JoystickButton(controller2, 1).whileTrue(Speaker1);
    // new JoystickButton(controller2,2 ).whileTrue(Speaker2);
    // new JoystickButton(controller2, 3).whileTrue(Speaker3);
    // new JoystickButton(controller2, 4).whileTrue(Speaker45
    // new JoystickButton(controller2, 5).whileTrue(Source);
    // new JoystickButton(controller2, 6).whileTrue(Stow);
    // new JoystickButton(controller2, 8).whileTrue(Amp);

    


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
