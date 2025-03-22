// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.commands.L4;
//import frc.robot.commands.L3;
//import frc.robot.commands.L4;
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
    private double MaxSpeed = TunerConstants_other.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    /* Setting up bindings for necessary control of the swerve drive platform 
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);*/

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController andrew = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants_other.createDrivetrain();

    //private final Arm arm = new Arm();
    //private final Arm1 armed = new Arm1();
    private final Elevator1 elevator = new Elevator1();
    private final PivotArm pivot = new PivotArm();
    private final EndEffector effector = new EndEffector();
    private final Climber climber = new Climber();
    private final Actuation actuation = new Actuation();
    private final Driving driving = new Driving(drivetrain);
    private final PhotonVision photon = new PhotonVision();
    private final SideCam m_SideCam = new SideCam();
    private final Pigeon pig = new Pigeon();
    private final PhotonVisionRear photonRear = new PhotonVisionRear(); 

    //private final PhotonVisionRear photonRear = new PhotonVisionRear();

    
    

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


   private final SendableChooser<Command> theAutochooser;

    public RobotContainer() {
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



        
        theAutochooser = new SendableChooser<>();

        theAutochooser.addOption("Left", (autoChooser.getSelected().alongWith(new L2(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm)))
        .andThen(new L2(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm).alongWith(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset, 1350)))
        //.andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset))
        .andThen(new Outake(effector, pivot, driving, elevator))
        .andThen(new L1(elevator, pivot).alongWith(autoChooser2.getSelected()))
        .andThen(Commands.deadline(new Intake(effector, pivot), new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig)))
        .andThen(new L2(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm).alongWith(autoChooser3.getSelected()))
        .andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset, 1350).alongWith(new L2(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm)))
        //.andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset))
        .andThen(new Outake(effector, pivot, driving, elevator))
        .andThen(new L1(elevator, pivot).alongWith(autoChooser4.getSelected()))
        .andThen(Commands.deadline(new Intake(effector, pivot), new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig)))
        .andThen(autoChooser5.getSelected().alongWith(new L2(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm)))
        .andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset,1350).alongWith(new L2(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm)))
        //.andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset))
        .andThen(new Outake(effector, pivot, driving, elevator))
        .andThen(new L1(elevator, pivot).alongWith(new Intake(effector, pivot))));

        theAutochooser.addOption("Right", (autoChooserb.getSelected().alongWith(new L2(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm)))
        .andThen(new L2(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm).alongWith(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset, 1400)))
        //.andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset))
        .andThen(new Outake(effector, pivot, driving, elevator))
        .andThen(new L1(elevator, pivot).alongWith(autoChooser2b.getSelected()))
        .andThen(Commands.deadline(new Intake(effector, pivot), new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig)))
        .andThen(new L2(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm).alongWith(autoChooser3b.getSelected()))
        .andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset,1400).alongWith(new L2(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm)))
        //.andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset))
        .andThen(new Outake(effector, pivot, driving, elevator))
        .andThen(new L1(elevator, pivot).alongWith(autoChooser4b.getSelected()))
        .andThen(Commands.deadline(new Intake(effector, pivot), new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig)))
        .andThen(autoChooser5b.getSelected().alongWith(new L2(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm)))
        .andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset, 1400).alongWith(new L2(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm)))
        //.andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset))
        .andThen(new Outake(effector, pivot, driving, elevator))
        .andThen(new L1(elevator, pivot).alongWith(new Intake(effector, pivot))));

        SmartDashboard.putData("Auto:", theAutochooser);


        

        
        //SmartDashboard.putData("Auto Mode", autoChooser);
        //NamedCommands.registerCommand("Leftaim", new AutoAlignLeft(photon, driving));
        //NamedCommands.registerCommand("Rightaim", new AutoAlignRight(photon, driving));
        NamedCommands.registerCommand("L1", new L1(elevator, pivot).alongWith(new Intake(effector, pivot)));
        NamedCommands.registerCommand("L4", new L2(elevator, pivot, effector, 31.26, -2.7));
        NamedCommands.registerCommand("Intake", new Intake(effector, pivot));
        NamedCommands.registerCommand("LeftAlign", new DumbAlign(m_SideCam, driving, -0.35, photon, pig, 0.34, 2000));

        configureBindings();
    }

    private void configureBindings() {

        andrew.rightBumper().onTrue(new ElevatorUp(elevator));
        andrew.leftBumper().onTrue(new ElevatorDown(elevator));
        //andrew.povRight().onTrue(new ArmUp(pivot));
        //andrew.povLeft().onTrue(new ArmDown(pivot));

        //andrew.rightStick().

        andrew.axisMagnitudeGreaterThan(4, 0.2).whileTrue((new ArmDown(pivot, andrew.getRightX())));

        //joystick.leftTrigger().toggleOnTrue(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, 0.4).andThen(new L1Shot(pivot, elevator, pig, driving, 20, 1.5, 0.78)));
        //joystick.rightTrigger().toggleOnTrue(new DumbAlign(m_SideCam, driving, 0, photon, pig, 0.36, 1500).andThen(new L1Shot(pivot, elevator, pig, driving, 0, 0.5, 0.32)));
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
        joystick.a().toggleOnTrue(new Outake(effector, pivot, driving, elevator));
        joystick.start().toggleOnTrue(new ActuateUp(actuation));
        joystick.back().toggleOnTrue(new ActuateDown(actuation));

        andrew.b().whileTrue(new RunIntake(effector, 5));
        andrew.a().whileTrue(new RunIntake(effector, -5));
        //joystick.povRight().onTrue(new ArmUp(pivot));
        //joystick.povLeft().onTrue(new ArmDown(pivot));
        joystick.povLeft().onTrue(new L2(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm));//L2 7.4

        joystick.povRight().onTrue(new L2(elevator, pivot, effector, Constants.L3Elevator, Constants.L3Arm));//L3 15.7
        joystick.povUp().onTrue(new L2(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm));//L4 29.9
        //joystick.x().onTrue(new L2(elevator, pivot, effector, 31.5, 0.346));

        joystick.povDown().toggleOnTrue(new L1(elevator, pivot).alongWith(new Intake(effector, pivot)));//L1

        joystick.rightBumper().toggleOnTrue(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset, 2000));
        //joystick.rightBumper().onTrue(new L2(elevator, pivot, effector, Constants.L3Elevator, Constants.L3Arm));

        joystick.leftBumper().toggleOnTrue(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset, 2000));
        //joystick.leftBumper().onTrue(new L2(elevator, pivot, effector, Constants.L3Elevator, Constants.L3Arm));

        joystick.b().toggleOnTrue(new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig));
        
        //andrew.leftBumper().toggleOnTrue(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset, 2000));
        //andrew.rightBumper().toggleOnTrue(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset, 2000));
        //joystick.povLeft().toggleOnTrue(new RunIntake(effector));
        

        //andrew.povUp().and(joystick.leftBumper()).toggleOnTrue(new L2(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm));
        //andrew.povLeft().and(joystick.leftBumper()).toggleOnTrue(new L2(elevator, pivot, effector, Constants.L3Elevator, Constants.L3Arm));
        //andrew.povDown().and(joystick.leftBumper()).toggleOnTrue(new L2(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm));


        joystick.leftBumper().and(andrew.povUp()).onTrue(new L2(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm));
        joystick.rightBumper().and(andrew.povUp()).onTrue(new L2(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm));

        joystick.leftBumper().and(andrew.povLeft()).onTrue(new L2(elevator, pivot, effector, Constants.L3Elevator, Constants.L3Arm));
        joystick.rightBumper().and(andrew.povLeft()).onTrue(new L2(elevator, pivot, effector, Constants.L3Elevator, Constants.L3Arm));

        joystick.leftBumper().and(andrew.povDown()).onTrue(new L2(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm));
        joystick.rightBumper().and(andrew.povDown()).onTrue(new L2(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm));
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        /*drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );*/

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        /*joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));*/

        /*joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );*/

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        //return autoChooser.getSelected();
        //return null;
        // return autoChooser.getSelected()
        //     .andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig)
        //     .alongWith(new L2(elevator, pivot, effector, 31.62, -2.7)))
        //     .andThen(new L2(elevator, pivot, effector, 31.62, -2.7))
        //     .andThen(new Outake(effector, pivot, driving))
        //     .andThen(new L1(elevator, pivot))
        //     .andThen(autoChooser2.getSelected())
        //     .andThen(new Intake(effector, pivot).alongWith(new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig)))
        //     .andThen(autoChooser3.getSelected())
        //     .andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig)
        //     .alongWith(new L2(elevator, pivot, effector, 31.62, -2.7))).andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig))
        //     .andThen(new Outake(effector, pivot, driving))
        //     .andThen(new L1(elevator, pivot))
        //     .andThen(autoChooser4.getSelected())
        //     .andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig).alongWith(new Intake(effector, pivot)))
        //     .andThen();


        //return new L1(elevator, pivot).andThen(autoChooser2.getSelected());
        //return autoChooser.getSelected();

        /*return (autoChooser.getSelected().alongWith(new L2(elevator, pivot, effector, 31.26, 0.346)))
            .andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, 0.34))//.alongWith(new L2(elevator, pivot, effector, 31.26, -2.7)))
            .andThen(new Outake(effector, pivot, driving))
            .andThen(new L1(elevator, pivot).alongWith(autoChooser2.getSelected()))
            //.andThen(autoChooser2.getSelected())
            .andThen(new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig).withDeadline(new Intake(effector, pivot)))
            .andThen(autoChooser3.getSelected().alongWith(new L2(elevator, pivot, effector,  31.26, -2.7)))
            .andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, 0.34))//.alongWith(new L2(elevator, pivot, effector, 31.26, -2.7)))
            .andThen(new Outake(effector, pivot, driving))
            .andThen(new L1(elevator, pivot).alongWith(autoChooser4.getSelected()))
            //.andThen(autoChooser4.getSelected())
            .andThen(new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig).withDeadline(new Intake(effector, pivot)))
            .andThen(autoChooser5.getSelected().alongWith(new L2(elevator, pivot, effector,  31.26, -2.7)))
            .andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, 0.34))//.alongWith(new L2(elevator, pivot, effector, 31.26, -2.7)))
            .andThen(new Outake(effector, pivot, driving))
            .andThen(new L1(elevator, pivot));*/


        
        
        /*return 
            (autoChooser.getSelected().alongWith(new L2(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm)))
            .andThen(new L2(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm).alongWith(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset)))
            .andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset))
            .andThen(new Outake(effector, pivot, driving))
            .andThen(new L1(elevator, pivot).alongWith(autoChooser2.getSelected()))
            .andThen(Commands.deadline(new Intake(effector, pivot), new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig)))
            .andThen(new L2(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm).alongWith(autoChooser3.getSelected()))
            .andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset).alongWith(new L2(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm)))
            .andThen(new DumbAlign(m_SideCam, driving, Constants.LeftOffset, photon, pig, Constants.ForwardOffset))
            .andThen(new Outake(effector, pivot, driving))
            .andThen(new L1(elevator, pivot).alongWith(autoChooser4.getSelected()))
            .andThen(Commands.deadline(new Intake(effector, pivot), new DumbAlignReverse(m_SideCam, driving, 0, photonRear, pig)))
            .andThen(autoChooser5.getSelected().alongWith(new L2(elevator, pivot, effector, Constants.L2Elevator, Constants.L2Arm)))
            .andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset).alongWith(new L2(elevator, pivot, effector, Constants.L4Elevator, Constants.L4Arm)))
            .andThen(new DumbAlign(m_SideCam, driving, Constants.RightOffset, photon, pig, Constants.ForwardOffset))
            .andThen(new Outake(effector, pivot, driving))
            .andThen(new L1(elevator, pivot).alongWith(new Intake(effector, pivot)));*/

        return theAutochooser.getSelected();


    }
}
