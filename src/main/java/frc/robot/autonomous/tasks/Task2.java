package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive.DriveTrain;

public class Task2 implements AutoTask {

    DriveTrain driveTrain = DriveTrain.getInstance();

    @Override
    public Command getCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveTrain.drive(0.5, 0), driveTrain),
            new WaitCommand(2),
            new InstantCommand(() -> driveTrain.drive(0, 0), driveTrain),
            new InstantCommand(() -> driveTrain.drive(0, 0.5), driveTrain),
            new WaitCommand(1),
            new InstantCommand(() -> driveTrain.drive(0, 0), driveTrain)
        );
    }
}
