package org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TeleConstants {
    public static  double SHOOTER_TARGET_RPM = 1500.0;


    public static  double TURRET_MIN_ANGLE = -180.0;
    public static  double TURRET_MAX_ANGLE = 180.0;


    public static  double TURRET_KP = 0.05;
    public static  double TURRET_KI = 0.0;
    public static  double TURRET_KD = 0.001;


    public static  double SHOOTER_KP = 0.0005;
    public static  double SHOOTER_KI = 0.0;
    public static  double SHOOTER_KD = 0.0;
    public static  double SHOOTER_KF = 0.0002;
}
