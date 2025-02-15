package frc.robot.autonomous.tasks;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveTrain;

public class Task3 implements AutoTask {

    DriveTrain driveTrain = DriveTrain.getInstance();

    @Override
    public Command getCommand() {
       String autoName = "Task3";
       return new PathPlannerAuto(autoName);
    }
    
}
