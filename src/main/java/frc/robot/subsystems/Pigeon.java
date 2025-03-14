// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pigeon extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final Pigeon2 pig = new Pigeon2(60);
  public Pigeon() {
    
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

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }


  public double getAccumGyro(){
    return pig.getAccumGyroZ().getValueAsDouble();
  }

  public double getYaw(){
    double angle = pig.getAccumGyroZ().getValueAsDouble();
    angle = angle % 360;  // Get remainder within -360 to 360
    return (angle >= 0) ? angle : angle + 360;  // Convert negative to positive

    //return pig.getRotation3d().getAxis().get;
  }

  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("rotation", getYaw());
    SmartDashboard.putNumber("Yaws", pig.getAccumGyroZ().getValueAsDouble());
    
    SmartDashboard.putNumber("oh3", (((0-getYaw()+180)%360+360)%360-180));

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
