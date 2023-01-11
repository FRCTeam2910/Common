package org.frcteam2910.common.io.json;

import com.google.gson.*;
import edu.wpi.first.math.geometry.Rotation2d;

import java.lang.reflect.Type;

public final class Rotation2JsonHandler implements JsonDeserializer<Rotation2d>, JsonSerializer<Rotation2d> {
    @Override
    public JsonElement serialize(Rotation2d src, Type typeOfSrc, JsonSerializationContext context) {
        return new JsonPrimitive(src.getDegrees());
    }

    @Override
    public Rotation2d deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context) throws JsonParseException {
        return Rotation2d.fromDegrees(json.getAsDouble());
    }
}
