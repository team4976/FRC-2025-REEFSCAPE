package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
//import frc.robot.subsystems.temptest;
import frc.robot.subsystems.Elevator;

public class ElevatorMovement extends Command {

    CommandXboxController c1 = new CommandXboxController(0);
    //temptest temptest = new temptest();

    Elevator elevator;

    int count =0;
    public double targetPos;

    public ElevatorMovement(Elevator subsystem, double _targetPos){
        elevator = subsystem;
        targetPos = _targetPos;
        addRequirements(elevator);
    }



 @Override
  public void initialize() {
    elevator.gotolevel(targetPos);
  }

  @Override
  public void execute() {
    System.out.println(Elevator.ElevatorLeader.getPosition());    
    
  }

  @Override
  public boolean isFinished() {
    return Elevator.ElevatorLeader.getPosition().getValueAsDouble() == targetPos 
    || Elevator.ElevatorLeader.getPosition().getValueAsDouble() > Constants.MaxMotorPosition 
    || Elevator.ElevatorLeader.getPosition().getValueAsDouble() < Constants.MinMotorPosition
    || Elevator.HomeLimitSwitchUnpressed.getAsBoolean() == false;
  }}


