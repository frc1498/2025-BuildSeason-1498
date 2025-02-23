package frc.robot.constants;

public class EndEffectorConstants {

    public static final String kEndEffectorModeAlgae="Algae";  //Set mode of endeffector to Algae
    public static final String kEndEffectorModeCoral="Coral";  //Set mode of endeffector to Coral

    public static enum movementState {
        FRONT_PAST_REAR_SAFE,
        REAR_PAST_FRONT_SAFE,
        FRONT_MIDDLE,
        REAR_MIDDLE,
        MIDDLE_PAST_FRONT_SAFE,
        MIDDLE_PAST_REAR_SAFE,
        MIDDLE_MIDDLE,
        NONE
    }

}
