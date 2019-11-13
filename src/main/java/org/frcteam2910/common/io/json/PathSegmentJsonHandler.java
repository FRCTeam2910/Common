package org.frcteam2910.common.io.json;

import com.google.gson.*;
import org.ejml.simple.SimpleMatrix;
import org.frcteam2910.common.control.PathSegment;
import org.frcteam2910.common.control.SplinePathSegment;
import org.frcteam2910.common.math.spline.Spline;

import java.lang.reflect.Type;

public final class PathSegmentJsonHandler implements JsonSerializer<PathSegment>, JsonDeserializer<PathSegment> {
    @Override
    public JsonElement serialize(PathSegment src, Type typeOfSrc, JsonSerializationContext context) {
        JsonObject root = new JsonObject();
        String type;
        if (typeOfSrc == SplinePathSegment.class) {
            type = "spline";
            Spline spline = ((SplinePathSegment) src).getSpline();

            root.add("basis", context.serialize(spline.getBasisMatrix()));
            root.add("weights", context.serialize(spline.getBasisWeightMatrix()));
        } else {
            throw new IllegalArgumentException("Tried to serialize unknown path segment type " + typeOfSrc.getTypeName());
        }

        root.addProperty("type", type);

        return root;
    }

    @Override
    public PathSegment deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context) throws JsonParseException {
        JsonObject root = json.getAsJsonObject();
        if (!root.has("type")) {
            throw new JsonParseException("Segment does not have a type");
        }

        String type = root.get("type").getAsString();

        if (type.equals("spline")) {
            SimpleMatrix basis = context.deserialize(root.get("basis"), SimpleMatrix.class);
            SimpleMatrix weights = context.deserialize(root.get("weights"), SimpleMatrix.class);

            return new SplinePathSegment(new Spline(basis, weights));
        } else {
            throw new JsonParseException(String.format("Unknown segment type \"%s\"", type));
        }
    }
}
