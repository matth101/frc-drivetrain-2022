package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxTrigger extends Trigger {
    
    private final XboxController controller;

    public XboxTrigger(XboxController controller) {
        this.controller = controller;
    }

    public boolean getRight() {
        return controller.getRightTriggerAxis() > 0.5;
    }

    public boolean getLeft() {
        return controller.getLeftTriggerAxis() > 0.5;
    }
} 
