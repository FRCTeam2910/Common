package org.frcteam2910.common.util;

import java.io.Serializable;
import java.util.TreeMap;

public class InterpolatingTreeMap<K extends InverseInterpolable<K> & Comparable<K>, V extends Interpolable<V>>
        extends TreeMap<K, V> implements Serializable {
    private static final long serialVersionUID = -2284755214507163440L;

    private final int maxSize;

    public InterpolatingTreeMap() {
        maxSize = Integer.MAX_VALUE;
    }

    public InterpolatingTreeMap(int maxSize) {
        this.maxSize = maxSize;
    }

    @Override
    public V put(K key, V value) {
        if (maxSize > 0 && maxSize <= size()) {
            remove(firstKey());
        }

        super.put(key, value);

        return value;
    }

    public V getInterpolated(K key) {
        V value = get(key);

        if (value != null) {
            return value;
        }

        K floor = floorKey(key);
        K ceiling = ceilingKey(key);

        if (floor == null && ceiling == null) {
            // If the floor and ceiling keys are not present, no keys are in the map and there is nothing to
            // interpolate.
            return null;
        } else if (floor == null) {
            // The key is before the first entry in the map
            return get(ceiling);
        } else if (ceiling == null) {
            // The key is after the last entry in the map
            return get(floor);
        }

        V floorVal = get(floor);
        V ceilingVal = get(ceiling);

        return floorVal.interpolate(ceilingVal, floor.inverseInterpolate(ceiling, key));
    }
}
