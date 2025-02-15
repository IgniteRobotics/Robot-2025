package frc.zones;
import static edu.wpi.first.units.Units.*;

public class ZoneTypes {

    public final Zone NULL = new Zone();

    public final Zone FREE = new Zone() {{
        setMaxSpeed(5);
        setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
    
    }};

    public final Zone BLUE_HP_TOP = new Zone(){{
        setMaxSpeed(3.5);
        setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
    }};

    public final Zone BLUE_HP_BOTTOM = new Zone(){{
        setMaxSpeed(3.5);
        setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
    }};

    public final Zone RED_HP_TOP = new Zone(){{
        setMaxSpeed(3.5);
        setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
    }};

    public final Zone RED_HP_BOTTOM = new Zone(){{
        setMaxSpeed(3.5);
        setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
    }};

    public final Zone BLUE_ALGAE_PROC = new Zone(){{
        setMaxSpeed(2);
        setTargetID(16);
        setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));

    }};

    public final Zone RED_ALGAE_PROC = new Zone(){{
        setMaxSpeed(2);
        setTargetID(3);
        setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
    }};


    public class RED_REEF{
        public final Zone RED_REEF_AB = new Zone(){{
            setMaxSpeed(2);
            setTargetID(7);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};
        
        public final Zone RED_REEF_CD = new Zone(){{
            setMaxSpeed(2);
            setTargetID(8);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};

        public final Zone RED_REEF_EF = new Zone(){{
            setMaxSpeed(2);
            setTargetID(9);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};

        public final Zone RED_REEF_GH = new Zone(){{
            setMaxSpeed(2);
            setTargetID(10);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};

        public final Zone RED_REEF_IJ = new Zone(){{
            setMaxSpeed(2);
            setTargetID(11);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};

        public final Zone RED_REEF_KL = new Zone(){{
            setMaxSpeed(2);
            setTargetID(6);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};
    }

    public class BLUE_REEF{
        
        public final Zone BLUE_REEF_AB = new Zone(){{
            setMaxSpeed(2);
            setTargetID(18);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};

        public final Zone BLUE_REEF_CD = new Zone(){{
            setMaxSpeed(2);
            setTargetID(17);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};

        public final Zone BLUE_REEF_EF = new Zone(){{
            setMaxSpeed(2);
            setTargetID(22);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};

        public final Zone BLUE_REEF_GH = new Zone(){{
            setMaxSpeed(2);
            setTargetID(21);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};

        public final Zone BLUE_REEF_IJ = new Zone(){{
            setMaxSpeed(2);
            setTargetID(20);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};

        public final Zone BLUE_REEF_KL= new Zone(){{
            setMaxSpeed(2);
            setTargetID(19);
            setMaxRotation(RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        }};
     }

}
