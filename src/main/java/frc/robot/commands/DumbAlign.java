// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Driving;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SideCam;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DumbAlign extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final SideCam m_SideCam;
  private final PhotonVision m_PhotonVision;
  private final Driving m_Driving;
  private final double m_Offset;
  private boolean end = false;
  private Pigeon m_pig;
  private double time;
  private double m_forwardOffset;

  private ProfiledPIDController forwardPid = new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(0.01, 10000));
  private ProfiledPIDController parallelPid = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(0.2, 0.2));

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DumbAlign(SideCam sideCam, Driving driving, double Offset, PhotonVision photonVision, Pigeon pig, double forwardOffset){

    m_pig = pig;
    m_forwardOffset = forwardOffset;
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

    parallelPid.reset(0);
    forwardPid.reset(0);


    time = System.currentTimeMillis();

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    //System.out.println("oh");
    if(!m_PhotonVision.getHas()){
      m_Driving.setRotation(0);
      m_Driving.setX(0);
      m_Driving.setY(0);
      return;
    }

    double Offset = 0;
    //int id = m_PhotonVision.getId();


    // if(Constants.tagIdtoRotation.containsKey(id)){
    //   Offset = Constants.tagIdtoRotation.get(id);
    // }
    
    



    

    double output;
    double forward;
    double rotation;


    if(m_PhotonVision.getHas()){

      int id = m_PhotonVision.getId();

      if(Constants.tagIdtoRotation.containsKey(id)){
        Offset = Constants.tagIdtoRotation.get(id);
      }

      rotation = (((Offset-m_pig.getYaw()+180)%360+360)%360-180)/100;

      
  
      //m_Driving.setRotation(-rotation/2);


      output = (m_PhotonVision.getY()-m_Offset);
      //output = -parallelPid.calculate(m_PhotonVision.getY(), m_Offset);

      

      forward = m_PhotonVision.getX()-m_forwardOffset;//0.34 works good

      //forward = -forwardPid.calculate(m_PhotonVision.getX(), m_forwardOffset);
      //System.out.println(forward);


      
      //m_Driving.setX(forward/1);

    }
    else{
      output = 0;
      rotation = 0;
      forward = 0;

      
    }



    //output = m_PhotonVision.getY()-m_Offset;
    if(forward>0.2){
      forward = 0.2;
    }
    else if(forward<-0.2){
      forward = -0.2;
    }

    if(output>0.2){
      output = 0.2;

    }
    else if(output < -0.2){
      output = -0.2;
    }

    if(rotation > 0.2){
      rotation = 0.2;
    }
    else if(rotation < -0.2){
      rotation = -0.2;
    }

    m_Driving.setRotation(rotation);
    m_Driving.setY(output/1);
    m_Driving.setX(forward/1);
    

    //m_Driving.setY((m_SideCam.getYaw() - (-27)));

    if((Math.abs(forward) < 0.007 && Math.abs(output) < 0.007 && Math.abs(rotation) < 0.007)|| System.currentTimeMillis() - time >2500 ){
      end = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Driving.setX(0);
    m_Driving.setY(0);
    m_Driving.setMode(false);
    //System.out.println("oh");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return end;

  }
}
