package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive.DriveTrain;

public class Task1 implements AutoTask {

    private final DriveTrain driveTrain = DriveTrain.getInstance();

    @Override
    public Command getCommand() {
        return new InstantCommand(() -> driveTrain.drive(0.5, 0), driveTrain);
    }
    
}
