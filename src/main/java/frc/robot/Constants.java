package frc.robot;

public final class Constants {
    public static final class ModuleConstants {
        // Because having magic numbers scattered throughout your code is totally not a terrible idea
        public static final double kWheelRadius = 0.0508; // meters
        public static final int kEncoderResolution = 4096;
        
        // Falcon 500 has 2048 ticks per revolution
        public static final int kFalconEncoderResolution = 2048;
    }

    public static final class CANIDs {
        // Front Left Module
        public static final int kFrontLeftDriveMotorId = 1;
        public static final int kFrontLeftTurningMotorId = 2;
        
        // Front Right Module
        public static final int kFrontRightDriveMotorId = 3;
        public static final int kFrontRightTurningMotorId = 4;
        
        // Back Left Module
        public static final int kBackLeftDriveMotorId = 5;
        public static final int kBackLeftTurningMotorId = 6;
        
        // Back Right Module
        public static final int kBackRightDriveMotorId = 7;
        public static final int kBackRightTurningMotorId = 8;
    }
} 