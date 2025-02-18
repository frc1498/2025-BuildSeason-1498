package frc.robot.constants;

public class CoralIntakeConstants {

    //These numbers are completely made up.
    public static final double kIntakeStowPosition = 0.0;
    public static final double kIntakeFloorPosition = 1.0;
    public static final double kIntakeRaisedPosition = 0.5;

    public static final double kSuckSpeed = 1.0;
    public static final double kSpitSpeed = -0.5;
    public static final double kStopSpeed = 0.0;

    //Units are in rotations.
    public static final double kDeadband = 0.05;
    public static final boolean kCoralIntakePrint = false;
    public static final boolean kCoralIntakePrintTriggers = false;

    //==================Sim Values================
    public static final double kIntakeRotateGearing = 112.0; //112:1 reduction
    public static final double kIntakeRotateMomentOfInertia = 0.01;  //kg-m^2 - Absolutely no idea
    public static final double kIntakeLength = 0.4191; //meters - Intake is ~ 16.5 in. long.
    public static final double kIntakeMinAngle = 0.0; //radians - starts at 0 for now.
    public static final double kIntakeMaxAngle = 2.09; //radians - assuming 120 degrees of movement
    public static final double kIntakeStartingAngle = 0.0; //radians
    public static final double kIntakeRollerMomentOfInertia = 0.01; //kg-m^2 - Absolutely no idea
    public static final double kIntakeRollerGearing = 3.0; //3:1 reduction    

}
