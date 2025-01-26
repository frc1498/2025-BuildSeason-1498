package frc.robot.config;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ElevatorConfig {

    public TalonFXConfiguration frontConfig = new TalonFXConfiguration();
    public TalonFXConfiguration backConfig = new TalonFXConfiguration();

    public ClosedLoopGeneralConfigs frontGains;

    public ElevatorConfig() {

        frontConfig.Slot0.kP = 10;
        frontConfig.Slot0.kI = 0.0;
        frontConfig.Slot0.kD = 0.0;

        backConfig.Slot0.kP = 10;
        backConfig.Slot0.kI = 0.0;
        backConfig.Slot0.kD = 0.0;
    }
}
