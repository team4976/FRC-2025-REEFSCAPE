// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

//import pathplanner library things
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

//import wpilib library things
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

//import other robot files
import frc.robot.commands.ActuateDown;
import frc.robot.commands.ActuateUp;
import frc.robot.commands.ArmDown;
import frc.robot.commands.ArmUp;
import frc.robot.commands.AutoAlignLeft;
import frc.robot.commands.AutoAlignRight;
import frc.robot.commands.DumbAlign;
import frc.robot.commands.DumbAlignReverse;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.Intake;
import frc.robot.commands.L1;
import frc.robot.commands.L1Shot;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.commands.L3;
import frc.robot.commands.L4;
import frc.robot.commands.Outake;
import frc.robot.commands.Reset;
import frc.robot.commands.RunIntake;
import frc.robot.generated.TunerConstants_other;
import frc.robot.subsystems.Actuation;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Driving;
import frc.robot.subsystems.Elevator1;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.PhotonVisionRear;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.SideCam;

public class RobotContainer {

    //setup for telemetry. 
    private double MaxSpeed = TunerConstants_other.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final Telemetry logger = new Telemetry(MaxSpeed);

    //define controller objects
    private final CommandXboxController drive_controller = new CommandXboxController(0);
    private final CommandXboxController op_controller = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants_other.createDrivetrain();

    //define subsystemss
    private final Elevator1 elevator = new Elevator1();
    private final PivotArm pivot = new PivotArm();
    private final EndEffector effector = new EndEffector();
    private final Actuation actuation = new Actuation();
    private final PhotonVision photon = new PhotonVision();
    private final SideCam m_SideCam = new SideCam();
    private final Pigeon pig = new Pigeon();
    private final PhotonVisionRear photonRear = new PhotonVisionRear(); 

    //this is what the drivetrain is actually run and controlled out of, mostly in the periodic function
    private final Driving driving = new Driving(drivetrain);

    //sets up multiple paths within the auto
    /* Path follower */
   private final SendableChooser<Command> autoChooser;
   private final SendableChooser<Command> autoChooser2;
   private final SendableChooser<Command> autoChooser3;
   private final SendableChooser<Command> autoChooser4;
   private final SendableChooser<Command> autoChooser5;

   private final SendableChooser<Command> autoChooserb;
   private final SendableChooser<Command> autoChooser2b;
   private final SendableChooser<Command> autoChooser3b;
   private final SendableChooser<Command> autoChooser4b;
   private final SendableChooser<Command> autoChooser5b;


   //this is the autochooser that appears to actually be used
   private final SendableChooser<Command> theAutochooser;

    public RobotContainer() {
        //see above. this just builds them. these don't actually have to go to smart dashboard at all
        autoChooser = AutoBuilder.buildAutoChooser("1a");
        autoChooser2 = AutoBuilder.buildAutoChooser("2a");
        autoChooser3 = AutoBuilder.buildAutoChooser("3a");
        autoChooser4 = AutoBuilder.buildAutoChooser("4a");
        autoChooser5 = AutoBuilder.buildAutoChooser("5a");

        autoChooserb = AutoBuilder.buildAutoChooser("1b");
        autoChooser2b = AutoBuilder.buildAutoChooser("2b");
        autoChooser3b = AutoBuilder.buildAutoChooser("3b");
        autoChooser4b = AutoBuilder.buildAutoChooser("4b");
        autoChooser5b = AutoBuilder.buildAutoChooser("5b");



        //autochooser actually in use
        theAutochooser = new SendableChooser<>();

        //this is all one auto. there has to be a better way to do this. an autonomous file of some sort...
        //composite command to end all composite commands. if it ain't broke don't fix it
        theAutochooser.addOption("Left", (autoChooser.getSelected().alongWith(new ElevatorToPosition(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm)))
        .andThen(new ElevatorToPosition(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm).alongWith(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset, 1200)))//was 1350
        //.andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset))
        .andThen(new Outake(effector, pivot, driving, elevator))
        .andThen(new L1(elevator, pivot).alongWith(autoChooser2.getSelected()))
        .andThen(Commands.deadline(new Intake(effector, pivot), new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig)))
        .andThen(new ElevatorToPosition(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm).alongWith(autoChooser3.getSelected()))
        .andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset, 1200).alongWith(new ElevatorToPosition(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm)))
        //.andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset))
        .andThen(new Outake(effector, pivot, driving, elevator))
        .andThen(new L1(elevator, pivot).alongWith(autoChooser4.getSelected()))
        .andThen(Commands.deadline(new Intake(effector, pivot), new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig)))
        .andThen(autoChooser5.getSelected().alongWith(new ElevatorToPosition(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm)))
        .andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset,1200).alongWith(new ElevatorToPosition(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm)))
        //.andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset))
        .andThen(new Outake(effector, pivot, driving, elevator))
        .andThen(new L1(elevator, pivot).alongWith(new Intake(effector, pivot))));

        //the other auto
        theAutochooser.addOption("Right", (autoChooserb.getSelected().alongWith(new ElevatorToPosition(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm)))
        .andThen(new ElevatorToPosition(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm).alongWith(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset, 1200)))
        //.andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset))
        .andThen(new Outake(effector, pivot, driving, elevator))
        .andThen(new L1(elevator, pivot).alongWith(autoChooser2b.getSelected()))
        .andThen(Commands.deadline(new Intake(effector, pivot), new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig)))
        .andThen(new ElevatorToPosition(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm).alongWith(autoChooser3b.getSelected()))
        .andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset,1200).alongWith(new ElevatorToPosition(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm)))
        //.andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset))
        .andThen(new Outake(effector, pivot, driving, elevator))
        .andThen(new L1(elevator, pivot).alongWith(autoChooser4b.getSelected()))
        .andThen(Commands.deadline(new Intake(effector, pivot), new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig)))
        .andThen(autoChooser5b.getSelected().alongWith(new ElevatorToPosition(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm)))
        .andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset, 1200).alongWith(new ElevatorToPosition(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm)))
        //.andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset))
        .andThen(new Outake(effector, pivot, driving, elevator))
        .andThen(new L1(elevator, pivot).alongWith(new Intake(effector, pivot))));

        //puts the auto chooser to smartdashboard
        SmartDashboard.putData("Auto:", theAutochooser);


        //commands can be trigged from pathplanner trajectories with these
        NamedCommands.registerCommand("L1", new L1(elevator, pivot).alongWith(new Intake(effector, pivot)));
        NamedCommands.registerCommand("L4", new ElevatorToPosition(elevator, pivot, effector, 31.26, -2.7));
        NamedCommands.registerCommand("Intake", new Intake(effector, pivot));
        NamedCommands.registerCommand("LeftAlign", new DumbAlign(m_SideCam, driving, -0.35, photon, pig, 0.34, 2000));

        configureBindings();
    }

    private void configureBindings() {

        //operator. manual elevator controls
        op_controller.rightBumper().onTrue(new ElevatorUp(elevator));
        op_controller.leftBumper().onTrue(new ElevatorDown(elevator));
       
        //operator. manual control for "wrist"
        op_controller.axisMagnitudeGreaterThan(4, 0.2).whileTrue((new ArmDown(pivot, op_controller.getRightX())));

        //driver. dumb align and then... shoot low? TODO: ask ben
        joystick.x().toggleOnTrue(new Reset(pig));

        joystick.rightTrigger().toggleOnTrue(new L2(elevator, pivot, effector, 0.5, 0.32));
        
        //andrew.povUp().onTrue(new L4(elevator));
        //joystick.povRight().onTrue(new L3(elevator));
        //joystick.povDown().toggleOnTrue(new L2(elevator, pivot, effector));
        //joystick.povLeft().onTrue(new L1(elevator));

        //joystick.povUp().toggleOnTrue(new L4(elevator, pivot, effector));
        //joystick.povRight().toggleOnTrue(new L3(elevator, pivot, effector));
        //joystick.povLeft().toggleOnTrue(new L2(elevator, pivot, effector));
        //joystick.povDown().toggleOnTrue(new L1(elevator));

        //joystick.povDown().onTrue(new Intake(effector, pivot));
        //joystick.leftBumper().toggleOnTrue(new AutoAlignLeft(photon, driving));
        //joystick.rightBumper().toggleOnTrue(new AutoAlignRight(photon, driving));



        //joystick.b().toggleOnTrue(new Intake(effector, pivot));
        //driver. run outake
        drive_controller.a().toggleOnTrue(new Outake(effector, pivot, driving, elevator));
        //driver. actuator. TODO: isn't this obsolete when we don't have a climber?
        drive_controller.start().toggleOnTrue(new ActuateUp(actuation));
        drive_controller.back().toggleOnTrue(new ActuateDown(actuation));

        //operator. intake manual controls
        op_controller.b().whileTrue(new RunIntake(effector, 5));
        op_controller.a().whileTrue(new RunIntake(effector, -5));
        
        //probably being kept around just in case
        //drive_controller.povRight().onTrue(new ArmUp(pivot));
        //drive_controller.povLeft().onTrue(new ArmDown(pivot));

        //driver. elevator controls. 
        drive_controller.povLeft().onTrue(new ElevatorToPosition(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm));//L2 7.4
        drive_controller.povRight().onTrue(new ElevatorToPosition(elevator, pivot, effector, Constants.L3Elevator, Constants.L3Arm));//L3 15.7
        drive_controller.povUp().onTrue(new ElevatorToPosition(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm));//L4 29.9
        drive_controller.povDown().toggleOnTrue(new L1(elevator, pivot).alongWith(new Intake(effector, pivot)));//L1

        //driver. camera-assisted alignment
        drive_controller.rightBumper().toggleOnTrue(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset, 2000));
        //joystick.rightBumper().onTrue(new L2(elevator, pivot, effector, Constants.L3Elevator, Constants.L3Arm));
        drive_controller.leftBumper().toggleOnTrue(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset, 2000));
        //joystick.leftBumper().onTrue(new L2(elevator, pivot, effector, Constants.L3Elevator, Constants.L3Arm));
        drive_controller.b().toggleOnTrue(new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig));
        
        //we still want sysid routines in our back pocket but we don't need them right now


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        drive_controller.back().and(drive_controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        drive_controller.back().and(drive_controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        drive_controller.start().and(drive_controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        drive_controller.start().and(drive_controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        //keeping for future reference
        // reset the field-centric heading on left bumper press
        //drive_controller.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        //telemetry. sends some data back to the robot on position and such
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    

    public Command getAutonomousCommand() {
     
        //there was a whole ton of old autos code that seemed defunct so I've just binned it.

        //returns the selected (composite) command from smartdashboard
        return theAutochooser.getSelected();


    }
}
