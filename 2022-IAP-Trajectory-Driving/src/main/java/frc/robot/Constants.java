// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int leftPort = 1;
    public static final int rightPort = 2;
    public static final double radiusinInch = 6.5;
    public static final double InchtoMeters = 0.0254;
    public static final double radius = InchtoMeters*radiusinInch;
    public static final double pi  = 3.14159;
    public static final double circum = 2*pi*radius;
    public static final double ticksToMeters= (2*3.14159*0.075)/4096; //divivde this by ticks to get meters
    public static final class PIDConstants{
        public static final double kP = 6.6544;
        public static final double kI = 0;
        public static final double kD = 4.833;
    }
}

