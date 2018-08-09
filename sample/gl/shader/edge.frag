#version 420 core

in vec2 uv;
out vec4 color;

uniform sampler2D depth;

uniform float nearPlane;
uniform float farPlane;

uniform float dx;
uniform float dy;

float getDepth(const in vec2 vec) {
    float d = float(texture2D(depth, vec));
    return farPlane * nearPlane / (farPlane - d * (farPlane - nearPlane));
}
float diffX(const in vec2 vec) {
    return getDepth(vec + vec2(dx, 0.0)) - getDepth(vec - vec2(dx, 0.0));
}
float diffY(const in vec2 vec) {
    return getDepth(vec + vec2(0.0, dy)) - getDepth(vec - vec2(0.0, dy));
}

void main(){
    float v0 = getDepth(uv + vec2(0.0, 0.0));
    float v1 = getDepth(uv + vec2(-dx, 0.0));
    float v2 = getDepth(uv + vec2(+dx, 0.0));
    float v3 = getDepth(uv + vec2(0.0, -dy));
    float v4 = getDepth(uv + vec2(0.0, +dy));

    float xx, yy;

    if(abs(v1 - v0) < abs(v2 - v0)) {
	    xx = abs(v2 - (2.0 * v0 - v1));
    }
    else {
	    xx = abs(v1 - (2.0 * v0 - v2));
    }
    if(abs(v3 - v0) < abs(v4 - v0)) {
        yy = abs(v4 - (2.0 * v0 - v3));
    }
    else {
        yy = abs(v3 - (2.0 * v0 - v4));
    }

    float d = sqrt(xx * xx + yy * yy);

    float thresh = getDepth(uv) * 0.005;

    if (d > thresh) {
	    float a = (d - thresh) / thresh;
        a = min(a, 1.0);
	    color = vec4(a, a, a, a);
    }
    else{
	    color = vec4(1.0, 1.0, 1.0, 0.0);
    }
}