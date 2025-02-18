package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.ElevatorHome;
import frc.robot.commands.ElevatorMovement;

public class Elevator extends SubsystemBase {

  public static DigitalInput HomeLimitSwitch = new DigitalInput(0);

  public static final TalonFX ElevatorLeader = new TalonFX(Constants.Elevator_Leader_Id);
  public static final TalonFX ElevatorFollower = new TalonFX(Constants.Elevator_Follower_Id);

  public static BooleanSupplier HomeLimitSwitchUnpressed = () -> HomeLimitSwitch.get();

  public Trigger HomeLimitSwitchUnpressedTrigger = new Trigger(HomeLimitSwitchUnpressed);

  public final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  public ElevatorMovement L4;
  public ElevatorMovement L3;
  public ElevatorMovement L2;
  public ElevatorMovement L1;
  public ElevatorHome Home;

    
  public Elevator(){
    L4 = new ElevatorMovement(this,100);
    L3 = new ElevatorMovement(this,75);
    L2 = new ElevatorMovement(this,50);
    L1 = new ElevatorMovement(this,25); 
    Home = new ElevatorHome(this);
    HomeLimitSwitchUnpressedTrigger.onFalse(StopElevator());
    HomeLimitSwitchUnpressed.getAsBoolean();
    ElevatorLeader.set(0.2);
    ElevatorFollower.setControl(new Follower(ElevatorLeader.getDeviceID(), false));
  }

  public Command StopElevator() {
    return runOnce(
      () -> {
        ElevatorLeader.set(0);
        ElevatorLeader.setPosition(0);
        ElevatorFollower.setPosition(0);
      }
    );
  }

  public void gotolevel(double targetPos){
    targetPos = SmartDashboard.getNumber("TestNumber", 0);
    System.out.println(targetPos);
    if (targetPos > Constants.MaxMotorPosition) {
      targetPos = Constants.MaxMotorPosition;
    }
    if (targetPos < Constants.MinMotorPosition) {
      targetPos = Constants.MinMotorPosition;
    }
    ElevatorLeader.setControl(m_request.withPosition(targetPos));
  }

  public void gohome(){
    System.out.println("this works");
    if (HomeLimitSwitch.get()) {
      ElevatorLeader.set(0.2);
      System.out.println("LimitSwitchUnPressed");
    }
    else {
      Elevator.ElevatorLeader.set(0.0);
      System.out.println("LimitSwitchPressed");
    }
  }
}