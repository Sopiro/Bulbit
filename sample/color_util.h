#pragma once

#include "bulbit/math.h"

namespace bulbit
{

Point3 HSLtoRGB(float h, float s, float l)
{
    float r, g, b;

    const static auto hue2rgb = [](float p, float q, float t) {
        if (t < 0.0f) t += 1.0f;
        if (t > 1.0f) t -= 1.0f;
        if (t < 1.0f / 6.0f) return p + (q - p) * 6.0f * t;
        if (t < 1.0f / 2.0f) return q;
        if (t < 2.0f / 3.0f) return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;
        return p;
    };

    if (s == 0.0f)
    {
        r = g = b = l;
    }
    else
    {
        float q = l < 0.5f ? (l * (1.0f + s)) : (l + s - l * s);
        float p = 2.0f * l - q;
        r = hue2rgb(p, q, h + 1.0f / 3.0f);
        g = hue2rgb(p, q, h);
        b = hue2rgb(p, q, h - 1.0f / 3.0f);
    }

    return Point3(r, g, b);
}

Point3 RGBtoHSL(float r, float g, float b)
{
    float max = std::fmax(r, std::fmax(g, b));
    float min = std::fmin(r, std::fmin(g, b));
    float h, s, l;

    l = (max + min) * 0.5f;

    if (max == min)
    {
        h = 0.0f;
        s = 0.0f;
    }
    else
    {
        float d = max - min;
        s = l > 0.5f ? d / (2.0f - max - min) : d / (max + min);

        if (max == r)
        {
            h = (g - b) / d + (g < b ? 6.0f : 0.0f);
        }
        else if (max == g)
        {
            h = (b - r) / d + 2.0f;
        }
        else
        { // max == b
            h = (r - g) / d + 4.0f;
        }

        h /= 6.0f;
    }

    return Point3(h, s, l);
}

} // namespace bulbit
