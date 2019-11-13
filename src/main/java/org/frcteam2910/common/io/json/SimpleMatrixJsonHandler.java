package org.frcteam2910.common.io.json;

import com.google.gson.*;
import org.ejml.simple.SimpleMatrix;

import java.lang.reflect.Type;

public final class SimpleMatrixJsonHandler implements JsonSerializer<SimpleMatrix>, JsonDeserializer<SimpleMatrix> {
    @Override
    public JsonElement serialize(SimpleMatrix src, Type typeOfSrc, JsonSerializationContext context) {
        JsonArray root = new JsonArray();
        for (int row = 0; row < src.numRows(); row++) {
            JsonArray r = new JsonArray();
            for (int col = 0; col < src.numCols(); col++) {
                r.add(src.get(row, col));
            }

            root.add(r);
        }

        return root;
    }

    @Override
    public SimpleMatrix deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context) throws JsonParseException {
        double[][] arr = context.deserialize(json, double[][].class);

        return new SimpleMatrix(arr);
    }
}
