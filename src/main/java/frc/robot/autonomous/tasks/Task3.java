package frc.robot.autonomous.tasks;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral.Coral;
import frc.robot.subsystems.Drive.DriveTrain;

public class Task3 implements AutoTask {

    DriveTrain driveTrain = DriveTrain.getInstance();
    @Override
    public Command getCommand() {
       String autoName = "Task3";
       PathPlannerAuto auto = new PathPlannerAuto(autoName);
       Coral coral = new Coral();

       auto.event("Reef").onTrue(coral.new scoreCommand());

       return auto;
    }
}