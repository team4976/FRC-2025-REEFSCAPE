// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {

  private PhotonCamera p_photon = new PhotonCamera("Front Cam");
  /** Creates a new ExampleSubsystem. */
  public PhotonVision() {
    

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  double skewy(PhotonTrackedTarget target) { return Units.radiansToDegrees(target.getBestCameraToTarget().getRotation().getX());}

  //double skewy(PhotonTrackedTarget target) { return Units.radiansToDegrees(target.getBestCameraToTarget().getRotation().getZ());}

  public double getYaw(){

    p_photon.setPipelineIndex(0);

    //System.out.println( "isConnected:" +  p_photon.isConnected());
    
    var result = p_photon.getLatestResult();
    double yaw;
    //result = p_photon.getLatestResult();

    boolean hasTargets = result.hasTargets();
    //System.out.println("hasTargets:"+hasTargets);
    if(hasTargets){
      PhotonTrackedTarget target = result.getBestTarget();
      //SmartDashboard.putNumber("skew", skewy(target));
      
      //yaw = skewy(target);
      //SmartDashboard.putNumber("yaw", yaw);
     
      //SmartDashboard.putNumber("pitch", target.getPitch());
      //System.out.println("getYaw:"+target.getYaw());
      return target.getYaw();
    }
    else{
      //System.out.println("skew:-1");
      //SmartDashboard.putNumber("skew", -1);
      //SmartDashboard.putNumber("yaw", -1);
      //SmartDashboard.putNumber("pitch", -1);
      yaw = -1;
      
    }
    SmartDashboard.putBoolean("has", hasTargets);
    return 0;
  
    
       

  }

  public boolean getHas(){
    p_photon.setPipelineIndex(0);

    //System.out.println( "isConnected:" +  p_photon.isConnected());
    
    var result = p_photon.getLatestResult();

    boolean oh = result.hasTargets();
    
    
    return oh;
  }
  public double getAmbiguity(){
    var result = p_photon.getLatestResult();

    PhotonTrackedTarget target = result.getBestTarget();

    return target.getPoseAmbiguity();

  }
  public double getX(){
    var result = p_photon.getLatestResult();
    //PhotonTrackedTarget target = result.getBestTarget();

    return result.getBestTarget().getBestCameraToTarget().getX();
  }
  public double getY(){
    var result = p_photon.getLatestResult();
    //PhotonTrackedTarget target = result.getBestTarget();

    return result.getBestTarget().getBestCameraToTarget().getY();
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    //double x = getYaw();
    //SmartDashboard.putNumber("oh", x);


    // This method will be called once per scheduler run


    var result = p_photon.getLatestResult();
    if(result.hasTargets()){
      SmartDashboard.putNumber("PhotonX", result.getBestTarget().getBestCameraToTarget().getX());
      SmartDashboard.putNumber("PhotonY", result.getBestTarget().getBestCameraToTarget().getY());
      SmartDashboard.putNumber("Yaw", result.getBestTarget().getYaw());

    }
    else{
      
      SmartDashboard.putNumber("PhotonY", -1);
    }
    
    //SmartDashboard.putNumber("PhotonX", result.getBestTarget().getBestCameraToTarget().getX());
    //SmartDashboard.putNumber("PhotonY", result.getBestTarget().getBestCameraToTarget().getY());


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
