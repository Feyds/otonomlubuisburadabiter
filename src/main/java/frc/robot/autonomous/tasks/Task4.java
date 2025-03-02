package frc.robot.autonomous.tasks;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Coral.Coral;
import frc.robot.subsystems.Drive.DriveTrain;

public class Task4 implements AutoTask {

    DriveTrain drive = DriveTrain.getInstance();
    Coral coral = Coral.getInstance();

    AHRS navx = new AHRS(NavXComType.kMXP_SPI);
    PIDController turnController = new PIDController(0.02, 0.0001, 0.002);  //TODO: PID DEĞERLERİ AYARLANACAK
    PIDController distanceController = new PIDController(0.1, 0.0005, 0.01);

    double angleSetpoint, distanceSetpoint, initYaw;

    public Task4() {
        navx.reset();
        turnController.setTolerance(2.0);
        distanceController.setTolerance(0.02);
    }

    @Override
    public Command getCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                distanceSetpoint = 0.88;
                initYaw = navx.getYaw();
                distanceController.setSetpoint(distanceSetpoint);
            }),
            new RunCommand(() -> {
                double speed = distanceController.calculate(navx.getDisplacementX());
                drive.drive(speed, 0);
            }, drive).withTimeout(3.0),
            new WaitCommand(0.5),
            new InstantCommand(coral::score),
            new WaitCommand(0.5),
            new InstantCommand(coral::stop),
            new InstantCommand(() -> drive.drive(0, 0))
        );
    }
}
