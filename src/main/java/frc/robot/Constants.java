package frc.robot;

public class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }
    public static class ElevatorConstants {
        public static enum ElevatorHeights{DOWN, L1, L2, L3, L4, ALGEE_LOW, ALGEE_HIGH, LOAD};
        //Devices
        public static final int kLeftElevatorMotor = 20;
        public static final int kRightElevatorMotor = 21;
        public static final int kCANdi = 22;
        //Ratios & Offsets
        public static final double kElevatorGearDiameter = 2.551;
        public static final int kIntEncoderToExtRatio = 64;
        public static final double kElevatorEncoderOffset = -0.269;
        //Current Limits
        public static final double kElevatorStatorCurrentLimit = 20;
        //Limits
        public static final double kElevatorMaxHeight = 26.5;
        public static final double kElevatorMinHeight = 0.25;
        //Heights
        public static final double kElevatorL1Height = 0.25; //TODO: Find this
        public static final double kElevatorL2Height = 5.54;
        public static final double kElevatorL3Height = 13.66;
        public static final double kElevatorL4Height = 26.04;
        public static final double kElevatorAlgeeLowHeight = 0; //TODO: Find this
        public static final double kElevatorAlgeeHighHeight = 0; //TODO: Find this
        public static final double kElevatorLoadHeight = 0.30; //TODO: Find this
    }
    public static class ShooterConstants{
        public static final int kUpperRollerMotor = 32;
        public static final int kLowerRollerMotor = 33;
    }
    public static class ClimberConstants{
        //Devices
        public static final int kClimberMotor = 40;
    }
}
