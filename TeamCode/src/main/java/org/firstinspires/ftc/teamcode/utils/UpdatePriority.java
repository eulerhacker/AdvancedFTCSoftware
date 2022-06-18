package org.firstinspires.ftc.teamcode.utils;

public class UpdatePriority {
    double basePriority;
    double priorityScale;
    double lastPower = 0;
    public double power = 0;
    long lastUpdateTime;

    public UpdatePriority(double basePriority, double priorityScale) {
        this.basePriority = basePriority;
        this.priorityScale = priorityScale;
        lastUpdateTime = System.currentTimeMillis();
    }

    public void setTargetPower(double targetPower) {
        power = targetPower;
    }

    public double getPriority() {
        if(power - lastPower == 0) {
            lastUpdateTime = System.currentTimeMillis();
            return 0;
        }
        // weighs the power change and time since last change
        return basePriority + Math.abs(power - lastPower) * (System.currentTimeMillis() - lastUpdateTime) * priorityScale;
    }

    public void update() {
        lastUpdateTime = System.currentTimeMillis();
        lastPower = power;
    }
}
