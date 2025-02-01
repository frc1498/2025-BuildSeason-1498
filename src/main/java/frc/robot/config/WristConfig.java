package frc.robot.config;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class WristConfig {

    public TalonFXConfiguration WristConfig = new TalonFXConfiguration();

    public ClosedLoopGeneralConfigs WristGains;

    public WristConfig() {

        WristConfig.Slot0.kP = 0.003;
        WristConfig.Slot0.kI = 0.0;
        WristConfig.Slot0.kD = 0.0;

    }
}
