package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.function.BooleanSupplier;

import com.revrobotics.AnalogInput;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.util.Constants;

public class Climber extends SubsystemBase{

    //public static DigitalInput ClimbSwitch = new DigitalInput(0);

    //public static BooleanSupplier ClimbSwitchOff = () -> ClimbSwitch.get();
    //public Trigger ClimbeSwitchOffTrigger = new Trigger(ClimbSwitchOff);

    SparkClosedLoopController pid;

    

    SparkMax Climber = new SparkMax(41, MotorType.kBrushless);
    SparkMaxConfig ClimberConfig = new SparkMaxConfig();

    

    AnalogInput ClimberSwitch = Climber.getAnalog();

    private RelativeEncoder ClimbEncoder;

    double ForwardSpeed;
    double BackwardSpeed;
    CommandXboxController m_driverController;


    
    public Climber (CommandXboxController m) {
          m_driverController = m;
          ClimbEncoder = Climber.getEncoder();
          ClimberConfig.inverted(false);


          pid = Climber.getClosedLoopController();




          
    }

    public void RunClimber(double speed){
        Climber.set(speed);
    }

    public void Hold(){
        pid.setReference(ClimbEncoder.getPosition(), ControlType.kPosition);
    }

    public boolean Switch(){
       return (ClimberSwitch.getPosition()<6);
    }

   

    @Override
    public void periodic() {
        System.out.println(Switch());

    }
}
