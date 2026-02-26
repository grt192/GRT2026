package frc.robot.subsystems.shooter;

import java.util.HashMap;
import java.util.Map;


import java.util.TreeMap;

public class table {

    private final TreeMap<Double, Double> table = new TreeMap<>();

    public table() {
        table.put(2.0, 2400.0);
        table.put(2.5, 2700.0);
        table.put(3.0, 3050.0);
        table.put(3.5, 3400.0);
        
    }
}