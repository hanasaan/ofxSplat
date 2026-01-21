// Adapted from https://github.com/antimatter15/splat
#version 150
//precision mediump float;

in vec4 vColor;
in vec2 vPosition;
out vec4 outColor;

const float EXP4 = exp(-4.0);
const float INV_EXP4 = 1.0 / (1.0 - EXP4);

float normExp(float x) {
    return (exp(x * -4.0) - EXP4) * INV_EXP4;
}

void main () {
    float A = dot(vPosition, vPosition) * 0.25;
    if (A > 1.0) discard;
    float alpha = normExp(A) * vColor.a;
    outColor = vec4(vColor.rgb * alpha, alpha);
}
