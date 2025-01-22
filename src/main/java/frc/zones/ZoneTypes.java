package frc.zones;

public class ZoneTypes {

    public final Zone FREE = new Zone() {{
        setMaxSpeed(5);
    
    }};

    public final Zone BLUE_HP_TOP = new Zone(){{
        setMaxSpeed(3.5);
    }};

    public final Zone BLUE_HP_BOTTOM = new Zone(){{
        setMaxSpeed(3.5);
    }};

    public final Zone RED_HP_TOP = new Zone(){{
        setMaxSpeed(3.5);
    }};

    public final Zone RED_HP_BOTTOM = new Zone(){{
        setMaxSpeed(3.5);
    }};

    public class RED_REEF{
        public final Zone RED_REEF_AB = new Zone(){{
            setMaxSpeed(2);
            setTargetID(7);
        }};
        
        public final Zone RED_REEF_CD = new Zone(){{
            setMaxSpeed(2);
            setTargetID(8);
        }};

        public final Zone RED_REEF_EF = new Zone(){{
            setMaxSpeed(2);
            setTargetID(9);
        }};

        public final Zone RED_REEF_GH = new Zone(){{
            setMaxSpeed(2);
            setTargetID(10);
        }};

        public final Zone RED_REEF_IJ = new Zone(){{
            setMaxSpeed(2);
            setTargetID(11);
        }};

        public final Zone RED_REEF_KL = new Zone(){{
            setMaxSpeed(2);
            setTargetID(6);
        }};
    }

    public class BLUE_REEF{
        
        public final Zone BLUE_REEF_AB = new Zone(){{
            setMaxSpeed(2);
            setTargetID(18);
        }};

        public final Zone BLUE_REEF_CD = new Zone(){{
            setMaxSpeed(2);
            setTargetID(17);
        }};

        public final Zone BLUE_REEF_EF = new Zone(){{
            setMaxSpeed(2);
            setTargetID(22);
        }};

        public final Zone BLUE_REEF_GH = new Zone(){{
            setMaxSpeed(2);
            setTargetID(21);
        }};

        public final Zone BLUE_REEF_IJ = new Zone(){{
            setMaxSpeed(2);
            setTargetID(20);
        }};

        public final Zone BLUE_REEF_KL= new Zone(){{
            setMaxSpeed(2);
            setTargetID(19);
        }};
     }

}
