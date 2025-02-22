package frc.zones;
import static edu.wpi.first.units.Units.*;

public class ZoneTypes {

    public static final Zone NULL = new Zone();

    public static final Zone FREE = new Zone() {{
        setMaxSpeed(5);
        setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
    }};

    public static final Zone HP_TOP = new Zone(){{
        setMaxSpeed(3.5);
        setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
    }};

    public static final Zone HP_BOTTOM = new Zone(){{
        setMaxSpeed(3.5);
        setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
    }};

    public static final Zone ALGAE_PROC = new Zone(){{
        setMaxSpeed(2);
        setTargetID(16);
        setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));

    }};

    public class REEF{
        public static final Zone REEF_AB = new Zone(){{
            setMaxSpeed(2);
            setTargetID(7);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};
        
        public static final Zone REEF_CD = new Zone(){{
            setMaxSpeed(2);
            setTargetID(8);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};

        public static final Zone REEF_EF = new Zone(){{
            setMaxSpeed(2);
            setTargetID(9);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};

        public static final Zone REEF_GH = new Zone(){{
            setMaxSpeed(2);
            setTargetID(10);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};

        public static final Zone REEF_IJ = new Zone(){{
            setMaxSpeed(2);
            setTargetID(11);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};

        public static final Zone REEF_KL = new Zone(){{
            setMaxSpeed(2);
            setTargetID(6);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};
    }
    public static final Zone OPPONENT = new Zone(){{
            setMaxSpeed(1);
            setMaxRotation(RotationsPerSecond.of(0.25).in(RadiansPerSecond));
    }};

}
