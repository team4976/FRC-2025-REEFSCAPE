// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator1;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PivotArm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/** An example command that uses an example subsystem. */
public class ElevatorToPosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final Elevator1 m_elevator;
  private final PivotArm m_PivotArm;
  private final EndEffector m_Effector;
  private boolean done;
  private double position;
  private double ArmPosition;
  //private final Command outake;
  private double time;
  

  public ElevatorToPosition(Elevator1 elevator, PivotArm pivot, EndEffector effector, double Position, double arm) {
    //m_subsystem = subsystem;
    ArmPosition = arm;
    m_elevator = elevator;
    m_PivotArm = pivot;
    m_Effector = effector;
    position = Position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    addRequirements(pivot);
    time = System.currentTimeMillis();
    //addRequirements(effector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //starts moving elevator
    m_elevator.gotolevel(position);
    m_PivotArm.goTo(ArmPosition);
    done = false;
    time = System.currentTimeMillis();

    //outake = new Outake(m_Effector, m_PivotArm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //either times out, or gets to the elevator position we want it to and stops moving
     if(System.currentTimeMillis()-time > 1650||(Math.abs(m_elevator.getVelocity())<5 && Math.abs(m_elevator.getRealPosition()-m_elevator.getSetPosition()) <0.3)){
      done = true;
      System.out.println(System.currentTimeMillis()-time);

     }
    //   m_PivotArm.goTo(1.35);
    

    // }
    // else{
    //   m_PivotArm.goTo(2.75);
    // }

    
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_Effector.runed(0);
    //m_PivotArm.goTo(2.75);
    //System.out.println("sigma");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
