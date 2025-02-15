package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.tasks.AutoTask;

public class AutoRunner {
    private static AutoTask currentTask;

    public static void start() {
        currentTask = AutoChooser.getSelectedTask();
        if (currentTask != null) {
            System.out.println("Seçilen Otonom: " + currentTask.getClass().getSimpleName());
            CommandScheduler.getInstance().schedule(currentTask.getCommand());
        }
    }

    public static void stop() {
        if (currentTask != null) {
            System.out.println("Otonom iptal ediliyor: " + currentTask.getClass().getSimpleName());
            currentTask.getCommand().cancel();
        }
    }
}
