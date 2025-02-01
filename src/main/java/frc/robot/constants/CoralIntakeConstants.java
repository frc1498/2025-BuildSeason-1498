package frc.robot.constants;

public class CoralIntakeConstants {
    public static final int kIntakePivotCANID = 9;
    public static final int kIntakeRollerCANID = 10;

    //These numbers are completely made up.
    public static final double kIntakeStowPosition = 0.0;
    public static final double kIntakeFloorPosition = 1.0;
    public static final double kIntakeRaisedPosition = 0.5;

    public static final double kSuckSpeed = 1.0;
    public static final double kSpitSpeed = -0.5;

    //Units are in rotations.
    public static final double kDeadband = 0.05;

    //==================Sim Values================
    public static final double kIntakeRotateGearing = 1.0;
    public static final double kIntakeRotateMomentOfInertia = 1.0;
    public static final double kIntakeLength = 1.0;
    public static final double kIntakeMinAngle = 0.0; //Should be radians
    public static final double kIntakeMaxAngle = 360.0; //Should be radians
    public static final double kIntakeStartingAngle = 0.0; //Should be radians
    public static final double kIntakeRollerMomentOfInertia = 1.0;
    public static final double kIntakeRollerGearing = 1.0;    

}
