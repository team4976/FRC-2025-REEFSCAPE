// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Driving;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.PhotonVisionRear;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SideCam;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DumbAlignReverse extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final SideCam m_SideCam;
  private final PhotonVisionRear m_PhotonVision;
  private final Driving m_Driving;
  private final double m_Offset;
  private boolean end = false;
  private Pigeon m_pig;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DumbAlignReverse(SideCam sideCam, Driving driving, double Offset, PhotonVisionRear photonVision, Pigeon pig){

    m_pig = pig;

    m_Offset = Offset;
    m_SideCam = sideCam;
    m_Driving = driving;
    m_PhotonVision = photonVision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driving);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Driving.setMode(true);
    //m_Driving.setY(0.01);
    m_Driving.setY(0);
    //m_Driving.setX(0.05);
    m_Driving.setRotation(0);
    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    System.out.println("oh");
    if(!m_PhotonVision.getHas()){
      m_Driving.setRotation(0);
      m_Driving.setX(0);
      m_Driving.setY(0);
      return;
    }

    double Offset = 0;
    int id = m_PhotonVision.getId();


    if(Constants.tagIdtoRotation.containsKey(id)){
      Offset = Constants.tagIdtoRotation.get(id);
    }
    
    

    double rotation = -(((Offset-m_pig.getYaw()+180)%360+360)%360-180)/100;

    //make sure rotation isn't too agressive, caps it
    if(rotation > 0.2){
      rotation = 0.2;
    }
    else if(rotation < -0.2){
      rotation = -0.2;
    }

    m_Driving.setRotation(-rotation);

    

    double output;
    double forward;

    if(m_SideCam.getYaw() == 1000){
      output = 0;
    }
    else{

      //output = (m_SideCam.getYaw() - (m_Offset))/-20;
      output = m_PhotonVision.getY()+0.7;
      //m_Driving.setY((m_SideCam.getYaw() - (-27))/-40);
    }

    //caps output again
    if(output>0.1){
      output = 0.1;

    }
    else if(output < -0.1){
      output = -0.1;
    }

    //bootleg pdi and even more capping. are we sure none of that is redundant? surely we don't need to do it this many times...
    if(m_PhotonVision.getHas()){
      output = -m_PhotonVision.getY()-m_Offset;

      if(output>0.1){
        output = 0.2;

      }
      else if(output < -0.1){
        output = -0.2;
      }

      forward = -(m_PhotonVision.getX()-0.64);//was 0.72
      if(forward>0.2){
        forward = 0.2;
      }      
      else if(forward < -0.2){
        forward = -0.2;
      }
      m_Driving.setX(forward);

    }
    else{
      output = 0;
      m_Driving.setX(0);
      forward = 0;
    }



    //output = m_PhotonVision.getY()-m_Offset;

    //prevent speed from being too high and getting out of control
    if(output>0.2){
      output = 0.2;

    }
    else if(output < -0.2){
      output = -0.2;
    }
    m_Driving.setY(output);

    //m_Driving.setY((m_SideCam.getYaw() - (-27)));

    //if in acceptable range, or enough time passed, end command
    if(Math.abs(forward) < 0.01 && Math.abs(output) < 0.01 && Math.abs(rotation) < 0.01 ){
      end = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Driving.setX(0);
    m_Driving.setY(0);
    m_Driving.setMode(false);
    System.out.println("oh");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return end;

  }
}
