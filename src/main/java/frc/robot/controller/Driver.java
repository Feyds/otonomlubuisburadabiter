package frc.robot.controller;

import edu.wpi.first.wpilibj.PS4Controller;

public class Driver {
    PS4Controller controller;

    public Driver(int port) {
        controller = new PS4Controller(port);
    }

    public double getSpeed() {
        return -controller.getLeftY();
    }

    public double getRotation() {
        return -controller.getRightX();
    }
    
    public boolean getWantsToScoreCoral() {
        return controller.getCrossButton();
    }

    public boolean getWantsToIntakeAlgae() {
        return controller.getSquareButton();
    }

    public boolean getWantsToScoreAlgae() {
        return controller.getCircleButton();
    }
}
