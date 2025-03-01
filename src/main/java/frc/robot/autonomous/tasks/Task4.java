package frc.robot.autonomous.tasks;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive.DriveTrain;

public class Task4 implements AutoTask {

    DriveTrain drive = DriveTrain.getInstance();

    AHRS navx = new AHRS(NavXComType.kMXP_SPI);
    PIDController turnController = new PIDController(0.02, 0.0001, 0.002);  //TODO: PID DEĞERLERİ AYARLANACAK
    PIDController distanceController = new PIDController(0.1, 0.0005, 0.01);

    double angleSetpoint, distanceSetpoint, initYaw;

    public Task4() {
        navx.reset();
        turnController.setTolerance(2.0);
        distanceController.setTolerance(2.0);
    }

    @Override
    public Command getCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                angleSetpoint = 90;
                turnController.setSetpoint(angleSetpoint);

                distanceSetpoint = 50;
                initYaw = navx.getYaw();
            }),
            new WaitCommand(0.5),
            new InstantCommand(() -> {
                while(!turnController.atSetpoint()) {
                    double turnSpeed = turnController.calculate(navx.getAngle());
                    drive.drive(0, turnSpeed);
                    drive.updatePosition(0, turnSpeed);
                }
            }),
            new WaitCommand(0.5),
            new InstantCommand(() -> {
                while(!distanceController.atSetpoint()) {
                    double speed = distanceController.calculate(navx.getDisplacementX());
                    drive.drive(speed, 0);
                    drive.updatePosition(speed, 0);
                }
            }),
            new InstantCommand(() -> {
                drive.drive(0, 0);
                drive.updatePosition(0, 0);
            })
        );
    }
}
