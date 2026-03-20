package frc.robot.util;

import java.util.ArrayDeque;
import java.util.Deque;

public class RollingAverage {

    private final int maxSize;
    private final Deque<Double> window = new ArrayDeque<>();
    private double sum = 0.0;

    public RollingAverage(int maxSize) {
        if (maxSize <= 0) {
            throw new IllegalArgumentException("maxSize must be positive");
        }
        this.maxSize = maxSize;
    }

    public void addSample(double value) {
        window.addLast(value);
        sum += value;
        if (window.size() > maxSize) {
            sum -= window.removeFirst();
        }
    }

    public double getAverage() {
        if (window.isEmpty()) {
            return 0.0;
        }
        return sum / window.size();
    }

    public int getRoundedAverage() {
        return (int) Math.round(getAverage());
    }

    public boolean isEmpty() {
        return window.isEmpty();
    }

    public int size() {
        return window.size();
    }

    public void clear() {
        window.clear();
        sum = 0.0;
    }
}
