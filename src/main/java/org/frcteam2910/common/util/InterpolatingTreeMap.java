package org.frcteam2910.common.util;

import java.util.TreeMap;

public class InterpolatingTreeMap<K extends InverseInterpolable<K> & Comparable<K>, V extends Interpolable<V>>
        extends TreeMap<K, V> {
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
