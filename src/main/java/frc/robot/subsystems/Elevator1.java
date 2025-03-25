package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Elevator1 extends SubsystemBase {
  // Digital input for detecting the home position limit switch
  private final DigitalInput homeLimitSwitch = new DigitalInput(Constants.elevatorLimitSwitchPort);
  
  // TalonFX motor controllers for the elevator mechanism
  private final TalonFX elevatorLeader = new TalonFX(Constants.Elevator_Leader_Id);
  private final TalonFX elevatorFollower = new TalonFX(Constants.Elevator_Follower_Id);
  
  // Motion Magic control mode for smooth movement
  private final MotionMagicVoltage motionControl = new MotionMagicVoltage(0).withSlot(0);
  
  // Power distribution hub (used for controlling LEDs to signal human players)
  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  
  // BooleanSupplier to check if the home limit switch is unpressed
  private final BooleanSupplier homeLimitSwitchUnpressed = homeLimitSwitch::get;
  private final Trigger homeLimitSwitchUnpressedTrigger = new Trigger(homeLimitSwitchUnpressed);

  // Target elevator position
  private double setPosition = 0.5;

  public Elevator1() {
    // Set the follower motor to mirror the leader motor
    elevatorFollower.setControl(new Follower(elevatorLeader.getDeviceID(), false));
  }

  /**
   * Moves the elevator to the specified position while keeping it within allowed limits.
   *
   * @param targetPos Desired target position for the elevator.
   */
  public void gotolevel(double targetPos) {
    targetPos = Math.max(Constants.MinMotorPosition, Math.min(targetPos, Constants.MaxMotorPosition));
    setPosition = targetPos;
    //this sets sets the pos
    elevatorLeader.setControl(motionControl.withPosition(targetPos));
  }
  public double getSetPosition() {
    return setPosition;
  }

  /**
   * @return The current acceleration of the elevator motor.
   */
  public double getAcceleration() {
    return elevatorLeader.getAcceleration().getValueAsDouble();
  }

  /**
   * @return The actual position of the elevator.
   */
  public double getRealPosition() {
    return elevatorLeader.getPosition().getValueAsDouble();
  }

  /**
   * @return The current velocity of the elevator.
   */
  public double getVelocity() {
    return elevatorLeader.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // Update SmartDashboard with elevator position data
    SmartDashboard.putNumber("Elevator Position", getRealPosition());
    SmartDashboard.putNumber("Elevator Set", setPosition);
    
    // Control LEDs to indicate when the elevator is ready to intake a coral
    boolean shouldEnableLEDs = Math.abs(setPosition - getRealPosition()) < 0.3 && setPosition == 1.05
        && DriverStation.isEnabled() && !SmartDashboard.getBoolean("Switch", false);
    powerDistribution.setSwitchableChannel(shouldEnableLEDs);
  }
}
