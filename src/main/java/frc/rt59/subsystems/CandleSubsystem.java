package frc.rt59.subsystems;


import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CandleSubsystem extends SubsystemBase {
    public CANdle robotCANdle = new CANdle(59);

    // Starting
    ColorFlowAnimation starting = new ColorFlowAnimation(100, 50, 0, 0, 2, 8, Direction.Forward, 0);
    SingleFadeAnimation noAuto = new SingleFadeAnimation(100, 0, 0, 660, 7, 8, 0);
    SingleFadeAnimation ready = new SingleFadeAnimation(0, 0, 100, 0, 240, 8, 0);

    public CandleSubsystem() {
        CANdleConfiguration robotCANdleConfig = new CANdleConfiguration();
        robotCANdleConfig.brightnessScalar = 0.5;
        robotCANdleConfig.disableWhenLOS = true;
        robotCANdle.configAllSettings(robotCANdleConfig);
    }

    @Override
    public void periodic() { // TODO: ADD AUTO SELECTOR
        if (DriverStation.isTeleop() && DriverStation.getMatchTime() <= 20) {
            setAnimation(new RainbowAnimation());
        } else if (DriverStation.isEnabled()) {
            solidColor(0,0,100);
        } else if (DriverStation.isDSAttached()) {
            setAnimation(ready);
        } else if (!DriverStation.isDSAttached()) {
            setAnimation(starting);
        }

    }
    public void setAnimation(Animation animation) {
        robotCANdle.animate(animation, 0);
    }
    public void solidColor(int r,int g, int b) {
        robotCANdle.setLEDs(r,g,b);
    }
}
