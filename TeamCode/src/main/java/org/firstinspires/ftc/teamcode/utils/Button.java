package org.firstinspires.ftc.teamcode.utils;

public class Button {
    boolean lastButton = false;

    public Button() {
    }

    public boolean isClicked(boolean button) {
        boolean clicked = button && !lastButton;
        lastButton = button;
        return clicked;
    }
}
