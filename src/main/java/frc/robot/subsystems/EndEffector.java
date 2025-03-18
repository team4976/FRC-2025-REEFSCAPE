// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AnalogInput;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax leftMax;
  private SparkMax rightMax;
  public double output;
  //limit switch in the end effector that says wether a coral is there or not
  private AnalogInput coralLimitSwitch;

  public EndEffector() {
    //configures the motors
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);
    
    leftMax = new SparkMax(51, MotorType.kBrushed);
    config.inverted(true);
    config.openLoopRampRate(0.01);
    
    leftMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(false);

    
    rightMax = new SparkMax(52, MotorType.kBrushed);

    rightMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //SparkMaxConfig sparkconfig = new SparkMaxConfig();
    //sparkconfig.inverted(true);
    
    coralLimitSwitch = leftMax.getAnalog();
 
  }


  //returns true if there's a coral in the end effector 
  public boolean getSwitch(){
    return coralLimitSwitch.getPosition() > 3;
  }

  //runs the end effector
  public void runed(double o){
    output = o;
    //leftMax.set(output);
    leftMax.setVoltage(output);
    //rightMax.set(output);
    rightMax.setVoltage(output);
  }

  //runs half the end effector
  public void oneSide(double output){
    leftMax.set(0.01);
    rightMax.set(output);
  }



  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Intake Switch", coralLimitSwitch.getPosition());
    SmartDashboard.putBoolean("Coral LimitSwitch", getSwitch());
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Effector Left Volt", leftMax.getAppliedOutput());
    SmartDashboard.putNumber("Effector Right Volt", rightMax.getAppliedOutput());
  }
}
