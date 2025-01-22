package frc.zones;

public class ZoneTypes {
    public final Zone OPPONENT_SIDE = new Zone() {{
        setMaxSpeed(3.5);
    }};

    public final Zone FREE = new Zone() {{
        setMaxSpeed(5);
    }};

    public final Zone HP = new Zone(){{
        setMaxSpeed(3);
        setTargetID(0);
    }};
}
