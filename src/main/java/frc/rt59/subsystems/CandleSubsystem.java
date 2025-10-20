package frc.rt59.subsystems;

import com.ctre.phoenix6.controls.RainbowAnimation;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CandleSubsystem extends SubsystemBase {
    public CANdle robotCANdle = new CANdle(59);

    // Starting
    ColorFlowAnimation starting = new ColorFlowAnimation(0, 7).withColor(new RGBWColor(100, 50, 0, 0)).withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(2);

    // SingleFadeAnimation noAuto = new SingleFadeAnimation(100, 0, 0, 660, 7, 8,
    // 0);
    SingleFadeAnimation noAuto = new SingleFadeAnimation(0, 7).withColor(new RGBWColor(100, 0, 0, 0)).withFrameRate(660).withSlot(0);

    SingleFadeAnimation ready = new SingleFadeAnimation(0, 7).withColor(new RGBWColor(0, 0, 100, 0)).withFrameRate(240).withSlot(0);

    SingleFadeAnimation inMatch = new SingleFadeAnimation(0, 7).withColor(new RGBWColor(0, 100,0, 0)).withFrameRate(100).withSlot(0);

    RainbowAnimation endGame = new RainbowAnimation(0, 7).withSlot(0);

    public CandleSubsystem() {
        CANdleConfiguration robotCANdleConfig = new CANdleConfiguration();
        robotCANdle.getConfigurator().refresh(robotCANdleConfig);
        robotCANdleConfig.LED.BrightnessScalar = 0.5;
        robotCANdleConfig.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        robotCANdleConfig.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
        robotCANdle.getConfigurator().apply(robotCANdleConfig);

    }

    @Override
    public void periodic() { // TODO: ADD AUTO SELECTOR
        if (DriverStation.isTeleop() && DriverStation.isEnabled() && DriverStation.getMatchTime() <= 20) {
            robotCANdle.setControl(endGame);
            SmartDashboard.putString("status", "endgame");
        } else if (DriverStation.isEnabled()) {
            robotCANdle.setControl(inMatch);
            SmartDashboard.putString("status", "enabled");
        } else if (!DriverStation.isEnabled()) {
            robotCANdle.setControl(ready);
            SmartDashboard.putString("status", "ready");
        } else if (!DriverStation.isDSAttached()) {
            robotCANdle.setControl(starting);
            SmartDashboard.putString("status", "starting");
        }

    }
}
