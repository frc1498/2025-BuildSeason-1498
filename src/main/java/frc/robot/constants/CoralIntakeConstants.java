package frc.robot.constants;

public class CoralIntakeConstants {

    //These numbers are completely made up.
    public static final double kIntakeFloorPosition = -0.209;
    public static final double kIntakeRaisedPosition = 0.19;
    public static final double kIntakeStowPosition = 0.23;

    //Safeties - the allowable range of movement.
    public static final double kIntakeMinPosition = -0.098145;
    public static final double kIntakeMaxPosition = 0.216309;

    public static final double kSuckSpeed = 40;
    public static final double kSpitSpeed = -20;
    public static final double kStopSpeed = 0.0;

    //Units are in rotations.
    public static final double kDeadband = 0.01;


    //==================Sim Values================
    public static final double kIntakeRotateGearing = 112.5; //112.5:1 reduction
    public static final double kIntakeRotateMomentOfInertia = 0.01;  //kg-m^2 - Absolutely no idea
    public static final double kIntakeLength = 0.4191; //meters - Intake is ~ 16.5 in. long.
    public static final double kIntakeMinAngle = 0.0; //radians - starts at 0 for now.
    public static final double kIntakeMaxAngle = 2.09; //radians - assuming 120 degrees of movement
    public static final double kIntakeStartingAngle = 0.0; //radians
    public static final double kIntakeRollerMomentOfInertia = 0.01; //kg-m^2 - Absolutely no idea
    public static final double kIntakeRollerGearing = 3.0; //3:1 reduction    

    //Motion Magic Constants
    public static final double kCruiseVelocity = 100 / kIntakeRotateGearing; //100 / gearingRatio - in rotations per second
    public static final double kAcceleration = 0;
    public static final double kJerk = 0;
}
