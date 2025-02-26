package frc.robot;

import java.util.function.BooleanSupplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class AutoRoutines {
    private final AutoFactory m_factory;
    public boolean startLeft;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine testPath() {
        final AutoRoutine routine = m_factory.newRoutine("test path");
        final AutoTrajectory path = routine.trajectory("testPath");
        routine.active().onTrue(
            path.resetOdometry()
            .andThen(path.cmd())
        );
        
        return routine;
    }
    
}
