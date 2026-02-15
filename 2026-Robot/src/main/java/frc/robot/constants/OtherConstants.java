package frc.robot.constants;

public final class OtherConstants {
    public static class IntakeConstants{
        public static int kIntakeID=16;

        public static final double kIntakeV = 0.12;
        public static final double kIntakeS = 0.1;
        public static final double kIntakeP = 0.2;
        public static final double kIntakeI = 0;
        public static final double kIntakeD = 0;

        public static double intakeSpeed = -.6;
        public static double outtakeSpeed = .5;
        public static double passiveIntakeSpeed = 0;
    }

    public static class ShooterConstants{
        public static int kShooterRID=11;
        public static int kShooterLID=12;

        public static final double kShooterP = 0.2;
        public static final double kShooterI = 0;
        public static final double kShooterD = 0;

        public static double shooterSpeed=.4;
        public static double passiveShooterSpeed=0
        ;

        public static double shooterRPSMinimum=400;
    }

    public static class ClimbConstants{
        public static int kClimbID=14;

        public static final double kClimbP = 0.2;
        public static final double kClimbI = 0;
        public static final double kClimbD = 0;

        public static double climbSpeed=.6;
        public static double climbUpAngle=50;
        public static double climbDownAngle=40;
    }

    public static class IndexerConstants{
        public static int kIndexerID=13;

        public static final double kIntakeV = 0.12;
        public static final double kIntakeS = 0.1;
        public static final double kIndexerP = 0.2;
        public static final double kIndexerI = 0;
        public static final double kIndexerD = 0;

        public static double indexerSpeed=.6;
        public static double passiveIndexerSpeed=0;
    }

    public static class SpindexerConstants{
        public static int kSpindexerID=45;

        public static final double kSpindexerP = 0.2;
        public static final double kSpindexerI = 0;
        public static final double kSpindexerD = 0;

        public static double spindexerSpeed=.5;
        public static double passiveSpindexerSpeed=0;
    }
}
