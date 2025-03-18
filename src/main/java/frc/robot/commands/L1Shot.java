// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Driving;
import frc.robot.subsystems.Elevator1;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.PivotArm;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class L1Shot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final PivotArm m_PivotArm;
  private final Elevator1 m_Elevator1;
  private final Pigeon m_Pigeon;
  private final Driving m_Driving;

  private double m_rotational;
  private double m_elevatorpos;
  private double m_pivotpos;


  private double originalGyro;
  private double desiredGyro;

  private boolean done;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public L1Shot(PivotArm pivotarm, Elevator1 elevator1, Pigeon pigeon, Driving driving, double rotational, double elevatorpos, double pivotpos) {
    //m_subsystem = subsystem;
    m_PivotArm = pivotarm;
    m_Elevator1 = elevator1;
    m_Pigeon = pigeon;
    m_Driving = driving;

    m_rotational = rotational;
    m_elevatorpos = elevatorpos;
    m_pivotpos = pivotpos;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
    addRequirements(driving);
    addRequirements(elevator1);
    addRequirements(pivotarm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    m_Driving.setMode(true);
    m_Driving.setX(0);
    m_Driving.setY(0);
    m_Driving.setRotation(0);
    m_PivotArm.goTo(m_pivotpos);
    m_Elevator1.gotolevel(m_elevatorpos);
    originalGyro = m_Pigeon.getAccumGyro();
    desiredGyro = originalGyro+m_rotational;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = -(m_Pigeon.getAccumGyro()-desiredGyro)/100;

    if(output>0.2){
      output = 0.2;
    }
    else if(output<-0.2){
      output = -0.2;
    }


    m_Driving.setRotation(output);

    if (Math.abs(output) < 0.007) {
      done = true;
      
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Driving.setRotation(0);
    m_Driving.setMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
