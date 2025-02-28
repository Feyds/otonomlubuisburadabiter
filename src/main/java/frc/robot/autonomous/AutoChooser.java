package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.tasks.*;

public class AutoChooser {
    private static AutoChooser autochooser = null;
    private static final SendableChooser<String> chooser = new SendableChooser<>();

    public static AutoChooser getInstance() {
        if (autochooser == null) {
            autochooser = new AutoChooser();
        }
        return autochooser;
    }

    public static void init() {
        chooser.setDefaultOption("Ileri git", "Task1");
        chooser.addOption("Ileri git ve saga dön", "Task2");
        chooser.addOption("CALISTIRMA!!!", "Task3");
        chooser.addOption("Otonom deneme", "Task4");

        SmartDashboard.putData("Auto Chooser", chooser);
    }

    public static AutoTask getSelectedTask() {
        String selectedTask = chooser.getSelected();
        switch (selectedTask) {
            case "Task1":
                return new Task1();
            case "Task2":
                return new Task2();
            case "Task3":
                return new Task3();
            case "Task4":
                return new Task4();
            default:
                return new Task1();
        }
    }
}
