package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kButtonBoardRightSide = 2;
        public static final int kButtonBoardLeftSide = 3;
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
        public static final double kElevatorEncoderOffset = -0.0632;
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
        public static final int kCANdi = 34;
    }
    public static class ClimberConstants{
        //Devices
        public static final int kClimberMotor = 40;
        public static final int kClimberIntakeMotor = 41;
        public static final int kClimberServo = 3; //PWM
        //Agles
        public static final double kServoLocked = 30;
        public static final double kServoUnlocked = 4;
    }
    public static class PositionConstants{
        public static enum Positions{TOPMID, BOTTOMMID};
        public static class BluePositions{
            public static class CoralReefPositions{
                public static final Pose2d alpha = new Pose2d(3.18, 4, new Rotation2d(Units.degreesToRadians(90)));
                public static final Pose2d bravo = new Pose2d(3.18, 3.67, new Rotation2d(Units.degreesToRadians(90)));
                public static final Pose2d charlie = new Pose2d();
                public static final Pose2d delta = new Pose2d();
                public static final Pose2d echo = new Pose2d();
                public static final Pose2d foxtrot = new Pose2d();
                public static final Pose2d golf = new Pose2d();
                public static final Pose2d hotel = new Pose2d();
                public static final Pose2d india = new Pose2d(5.14, 5.17, new Rotation2d(Units.degreesToRadians(-30)));
                public static final Pose2d juliett = new Pose2d(4.75, 5.38,new Rotation2d(Units.degreesToRadians(-30)));
                public static final Pose2d kilo = new Pose2d();
                public static final Pose2d lima = new Pose2d();
            }
            public static class coralStationPositions{
                //public static final Pose2d topClose = new Pose2d(0.62, 7.24, new Rotation2d(Units.degreesToRadians(37.51)));
                //public static final Pose2d topFar = new Pose2d(1.09, 7.58, new Rotation2d(Units.degreesToRadians(36.74)));
                public static final Pose2d topMid = new Pose2d(1.090, 7.49, new Rotation2d(Units.degreesToRadians(35)));
                //public static final Pose2d bottomClose = new Pose2d();
                public static final Pose2d bottomMid = new Pose2d(.950, 0.770, new Rotation2d(Units.degreesToRadians(145)));
                //public static final Pose2d bottomFar = new Pose2d();
            }
        }
        public static class RedPositions{
            public static class RedCoralStationPositions{ 
                public static final Pose2d topMid = new Pose2d(16.45, 0.82, new Rotation2d(Units.degreesToRadians(-145)));
                public static final Pose2d bottomMid = new Pose2d(16.45, 7.274, new Rotation2d(Units.degreesToRadians(-35)));
            }
        }  
    }
}
