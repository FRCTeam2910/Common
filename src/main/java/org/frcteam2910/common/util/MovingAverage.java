package org.frcteam2910.common.util;

import java.util.ArrayList;
import java.util.List;

public class MovingAverage {
    private final int maxSize;
    private final List<Double> values = new ArrayList<>();
    private int index = 0;

    public MovingAverage(int maxSize) {
        this.maxSize = maxSize;
    }

    public void add(double number) {
        if (values.size() != maxSize) {
            values.add(number);
        } else {
            values.set(index++, number);

            if (index == maxSize) {
                index = 0;
            }
        }
    }

    public double get() {
        double average = 0;
        for (Double value : values) {
            average += value;
        }

        return average / values.size();
    }

    public void clear() {
        values.clear();
    }
}
