package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorHome extends Command {

    Elevator elevator;

  public ElevatorHome(Elevator subsystem) {
    elevator = subsystem;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.gohome();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean isInterrupted){
    Elevator.ElevatorLeader.set(0.0);
    Elevator.ElevatorLeader.setPosition(0);
    Elevator.ElevatorFollower.setPosition(0);
  }

  @Override
  public boolean isFinished() {
    return Elevator.HomeLimitSwitch.get() == false;
  }
}
