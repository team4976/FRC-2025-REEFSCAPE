// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
  private boolean hasCoral = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Outake(EndEffector endEffector, PivotArm pivotarm) {
    //m_subsystem = subsystem;
    m_EndEffector = endEffector;
    m_PivotArm = pivotarm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(endEffector);
    addRequirements(pivotarm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_EndEffector.runed(0.15);
    hasCoral = false;
    //m_PivotArm.goTo(1.75);




  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_EndEffector.getSwitch()){
      hasCoral = true;
    }
    if(Math.abs(m_PivotArm.getPosition() - (m_PivotArm.getRealPostion())) <0.25){
      m_EndEffector.runed(0.2);
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
    return hasCoral;
  }
}
