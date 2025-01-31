package frc.robot.config;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ClimberConfig {

    public TalonFXConfiguration ClimberConfig = new TalonFXConfiguration();

    public ClosedLoopGeneralConfigs ClimberGains;

    public ClimberConfig() {

        ClimberConfig.Slot0.kP = 10;
        ClimberConfig.Slot0.kI = 0.0;
        ClimberConfig.Slot0.kD = 0.0;

    }
}
