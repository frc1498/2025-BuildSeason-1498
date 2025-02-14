package frc.robot.subsystems;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.LEDConstants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class LED extends SubsystemBase{

    private final CANdle m_candle = new CANdle(LEDConstants.kCANdleCANID, "rio");
    private final int LedCount = 300;  //This is a placeholder.  Need the correct number of leds.
    private Animation m_toAnimate = null;
    private Animation larson = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
    private Animation colorflow = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
    private Animation fireanimation = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
    private Animation rainbow = new RainbowAnimation(1, 0.1, LedCount);
    private Animation rgbfade = new RgbFadeAnimation(0.7, 0.4, LedCount);
    private Animation singlefade = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
    private Animation strobe = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
    private Animation twinkle = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
    private Animation twinkleoff = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);

    /* Wrappers so we can access the CANdle from the subsystem */
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    public LED() {
        //Constructor
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
        m_candle.animate(larson);
    }
    //====================================================================
    //=========================Configuration==============================
    //====================================================================



    //=====================================================================
    //==============================Private================================
    //=====================================================================
    private void LED_Larson() {
        m_candle.clearAnimation(0);
        m_candle.animate(larson,0);
    }

    private void LED_colorflow() {
        m_candle.clearAnimation(0);
        m_candle.animate(colorflow,0);
    }
    private void LED_FireAnimation() {
        m_candle.clearAnimation(0);
        m_candle.animate(fireanimation,0);
    }
    private void LED_Rainbow() {
        m_candle.clearAnimation(0);
        m_candle.animate(rainbow,0);
    }
    private void LED_rgbfade() {
        m_candle.clearAnimation(0);
        m_candle.animate(rgbfade,0);
    }
    private void LED_singlefade() {
        m_candle.clearAnimation(0);
        m_candle.animate(singlefade,0);
    }
    private void LED_strobe() {
        m_candle.clearAnimation(0);
        m_candle.animate(strobe,0);
    }
    private void LED_twinkle() {
        m_candle.clearAnimation(0);
        m_candle.animate(twinkle,0);
    }
    private void LED_TwinkleOff() {
        m_candle.clearAnimation(0);
        m_candle.animate(twinkleoff,0);
    }
    private void LEDon() {
        m_candle.clearAnimation(0);
        m_candle.setLEDs(255, 255, 255);  //LEDs go white
    }
    
    //=====================================================
    //======================Commands ======================
    //=====================================================
    public Command LEDsOn() {
        return run(
            () -> {this.LEDon();}
        );
    }

    public Command LEDsMode() {
        return run(
            () -> {this.LED_Larson();}  //We'll use Larson as the default when we aren't using the LEDs to signal
        );
    }

    //============================================================
    //=======================Triggers=============================
    //============================================================

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
    }    

}
