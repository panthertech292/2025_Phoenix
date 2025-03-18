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
        public static final double kElevatorEncoderOffset = 0; //TODO: Find this
        //Current Limits
        public static final double kElevatorStatorCurrentLimit = 20; //TODO: Tune this
        //Limits
        public static final double kElevatorMaxHeight = 23.5; //TODO: Find this
        public static final double kElevatorMinHeight = 0.25;
        //Heights
        public static final double kElevatorL1Height = 0; //TODO: Find this
        public static final double kElevatorL2Height = 0; //TODO: Find this
        public static final double kElevatorL3Height = 0; //TODO: Find this
        public static final double kElevatorL4Height = 0; //TODO: Find this
        public static final double kElevatorAlgeeLowHeight = 0; //TODO: Find this
        public static final double kElevatorAlgeeHighHeight = 0; //TODO: Find this
        public static final double kElevatorLoadHeight = 0; //TODO: Find this
    }
    public static class ShooterConstants{

    }
    public static class ClimberConstants{
        //Devices
        public static final int kClimberMotor = 40;
      }
}
