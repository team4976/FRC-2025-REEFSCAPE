// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PivotArm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private SparkMax pivot;
  private RelativeEncoder pivotEncoder;

  private AbsoluteEncoder absEncoder;

  private SparkClosedLoopController pid;

  private SparkAnalogSensor pivotAnalogSensor;

  private SparkMaxConfig pivotConfig;

  //private int mode = 1;

  private double position = 0.48;

  ProfiledPIDController m_Controller = new ProfiledPIDController(1.7, 2, 0.1, new TrapezoidProfile.Constraints(26, 35));

  

  
  
  
  public PivotArm() {

    //m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.5, 0.5));


    pivot = new SparkMax(50, MotorType.kBrushless);
    
    

    absEncoder = pivot.getAbsoluteEncoder();

    
    pivotAnalogSensor = pivot.getAnalog();


    pivot.set(0);

    pivotConfig = new SparkMaxConfig();
    //pivotConfig.inverted(true);
    

    pivotConfig.closedLoop.outputRange(-0.25, 0.25);
    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    pivotConfig.idleMode(IdleMode.kBrake);

    pivotConfig.closedLoop.pidf(2.8, 0.01, 2, 0);//p was 0.2
    pivotConfig.closedLoop.iZone(0.05);
    pivotConfig.inverted(false);
    //pivotConfig.closedLoopRampRate(0.1);
    pivotConfig.encoder.positionConversionFactor(0.125);
    

    
    
    
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid = pivot.getClosedLoopController();

    pivotEncoder = pivot.getEncoder();

    pivotEncoder.setPosition((1-absEncoder.getPosition())*-1);

    // if(absEncoder.getPosition()>0.8){
    //   pivotEncoder.setPosition((1-absEncoder.getPosition())*-1);
    // }
    // else{
    //   pivotEncoder.setPosition(absEncoder.getPosition());

    // }
    //pivotEncoder.setPosition(absEncoder.getPosition());

    //pivotEncoder.setPosition(absEncoder.getPosition());

    //pivotEncoder.setPosition(0);




  }
  private boolean Zero(){
    return pivotAnalogSensor.getPosition() > 3;
  }

  

  public void goTo(double pos){
    pid.setReference(pos, SparkBase.ControlType.kPosition);
    position = pos;
  }
    
  public double getPosition(){
    return position;
  }
  public double getVelocity(){
    return pivotEncoder.getVelocity();
  }
  public double getRealPostion(){
    return absEncoder.getPosition();
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
    // This method will be called once per scheduler run
    //if(Zero()){
     // pivotEncoder.setPosition(0);

    //}

    SmartDashboard.putNumber("Pivot Position", absEncoder.getPosition());
    SmartDashboard.putNumber("MotorEncode", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Set", position);
    SmartDashboard.putBoolean("Pivot Limit", Zero());
    //pivot.set(m_Controller.calculate(absEncoder.getPosition(), position));

    // if(DriverStation.isEnabled()){

    //   if(Math.abs(absEncoder.getPosition() - pivotEncoder.getPosition())>0.5){
    //     pivot.set(Math.max(-0.2, Math.min(0.2, m_Controller.calculate(pivotEncoder.getPosition(), position) )));
    //   }
    //   else{
    //     pivot.set(Math.max(-0.2, Math.min(0.2, m_Controller.calculate(absEncoder.getPosition(), position) )));
    //   }
    //   //pivot.set(Math.max(-0.4, Math.min(0.4, m_Controller.calculate(absEncoder.getPosition(), position) )));

    // }
    // else{

    //   //  if(Math.round(m_Controller.calculate(absEncoder.getPosition(), absEncoder.getPosition()+0.45) * 1.0) / 1.0 != 0){
    //   //    System.out.println("DO NOT ENABLE");
         
    //   //  }
    //   double abspos = absEncoder.getPosition();
    //    m_Controller.reset(pivotEncoder.getPosition());

    //    //System.out.println(m_Controller.calculate(abspos, abspos));
    //    System.out.println(Math.abs(absEncoder.getPosition() - pivotEncoder.getPosition()));
    //    //m_Controller.reset(abspos);
      
    //   //System.out.println(m_Controller.calculate(absEncoder.getPosition(), absEncoder.getPosition()));
    //   //System.out.println(Math.round(m_Controller.calculate(absEncoder.getPosition(), absEncoder.getPosition()) * 1000.0) / 1000.0);
    //   pivot.set(0);
    
    // }
    

    //var the = m_profile.calculate(position, null, null) ;
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
