// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Driving;
import frc.robot.subsystems.Elevator1;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.PivotArm;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Outake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final EndEffector m_EndEffector;
  private final PivotArm m_PivotArm;
  private final Driving m_Driving;
  //private final Elevator1 m_Elevator1;
  private boolean hasCoral = false;
  private boolean done = false;
  private double time;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Outake(EndEffector endEffector, PivotArm pivotarm, Driving mDriving) {
    //m_subsystem = subsystem;
    m_EndEffector = endEffector;
    m_PivotArm = pivotarm;
    m_Driving = mDriving;
    //m_Elevator1 = elevator1;
    //addRequirements(elevator1);
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(endEffector);
    //addRequirements(pivotarm);
    addRequirements(mDriving);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //if(m_PivotArm.getPosition() < 3){
    if(m_PivotArm.getPosition() > 0.6){
      m_EndEffector.runed(-5.5);
    }
    else if(m_PivotArm.getPosition() == 0.32){
      m_EndEffector.oneSide(0.75);
    }
    else{
      m_EndEffector.runed(5.5);
    }
    
      //m_EndEffector.runed(0.4);
      //time = System.currentTimeMillis();

    //}
    //else{
     // m_EndEffector.runed(-0.3);
    //}
    //m_EndEffector.runed(-0.15);
    hasCoral = false;
    done = false;
    //m_PivotArm.goTo(1.75);




  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_EndEffector.getSwitch() && !hasCoral){
      time = System.currentTimeMillis();
      hasCoral = true;
      
    }
    if(hasCoral && (System.currentTimeMillis()-time > 150)){
      done = true;

    }
    if(Math.abs(m_PivotArm.getPosition() - (m_PivotArm.getRealPostion())) <0.25){
      //m_EndEffector.runed(-0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_EndEffector.runed(0);
    //m_PivotArm.goTo(2.75);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
