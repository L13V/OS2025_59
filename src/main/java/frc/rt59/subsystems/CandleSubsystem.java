// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.rt59.subsystems;


import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CandleSubsystem extends SubsystemBase {
    /** Creates a new CandleSubsystem. */
    public CANdle robotCANdle = new CANdle(59);

    // Starting
    ColorFlowAnimation starting = new ColorFlowAnimation(100, 50, 0, 0, 2, 8, Direction.Forward, 0);
    SingleFadeAnimation noAuto = new SingleFadeAnimation(100, 0, 0, 660, 7, 8, 0);
    SingleFadeAnimation ready = new SingleFadeAnimation(0, 0, 100, 0, 240, 8, 0);
    RGBWColor inMatch = new RGBWColor(0, 0, 100, 0);

    public CandleSubsystem() {
        CANdleConfiguration robotCANdleConfig = new CANdleConfiguration();
        robotCANdleConfig.brightnessScalar = 0.5;
        robotCANdleConfig.disableWhenLOS = true;
        robotCANdle.configAllSettings(robotCANdleConfig);
    }

    @Override
    public void periodic() { // TODO: ADD AUTO SELECTOR
        // This method will be called once per scheduler run
        if (DriverStation.isTeleop() && DriverStation.getMatchTime() <= 20) {
            robotCANdle.animate(new RainbowAnimation());
        } else if (DriverStation.isEnabled()) {
            robotCANdle.setLEDs(0, 0, 100);
        } else if (DriverStation.isDSAttached()) {
            robotCANdle.animate(ready);
        } else if (!DriverStation.isDSAttached()) {
            robotCANdle.animate(new ColorFlowAnimation(100, 50, 0, 0, 2, 7, Direction.Forward, 0));
        }

    }
}
