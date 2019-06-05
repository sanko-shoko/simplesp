SP_SHADER(
#version 420 core \n

in vec2 uv;
out vec4 color;

uniform sampler2D depth;

uniform int pers;

uniform float nearPlane;
uniform float farPlane;

uniform float dx;
uniform float dy;

float getDepth(const in vec2 vec) {
    const float zbf = float(texture2D(depth, vec));

    float d = 0.0;
    if (pers == 1) {
        const float div = (farPlane - zbf * (farPlane - nearPlane));
        if (div > 0.001) {
            d = farPlane * nearPlane / div;
        }
    }
    else {
        const float p2 = 2.0 / (farPlane - nearPlane);
        const float p3 = -(farPlane + nearPlane) / (farPlane - nearPlane);
        d = (zbf * 2 - 1 - p3) / p2;
    }
    return (d > nearPlane && d < farPlane) ? d : 0.0;
}

float sobelX(const in vec2 vec) {
    return getDepth(vec + vec2(dx, 0.0)) - getDepth(vec - vec2(dx, 0.0));
}
float sobelY(const in vec2 vec) {
    return getDepth(vec + vec2(0.0, dy)) - getDepth(vec - vec2(0.0, dy));
}
float laplacian(const in vec2 vec) {
    float ret =
    + 1.0 * getDepth(vec - vec2(-dx, -dy)) + 1.0 * getDepth(vec - vec2(0.0, -dy)) + 1.0 * getDepth(vec - vec2(+dx, -dy))
    + 1.0 * getDepth(vec - vec2(-dx, 0.0)) - 8.0 * getDepth(vec - vec2(0.0, 0.0)) + 1.0 * getDepth(vec - vec2(+dx, 0.0))
    + 1.0 * getDepth(vec - vec2(-dx, +dy)) + 1.0 * getDepth(vec - vec2(0.0, +dy)) + 1.0 * getDepth(vec - vec2(+dx, +dy));
    return ret;
} 

void main(){

    float val = abs(laplacian(uv));

    float thresh = getDepth(uv) * 0.001;

    if (val > thresh) {
	    float a = (val - thresh) / (10.0 * thresh);
        a = min(a, 1.0);
	    color = vec4(a, a, a, a);
    }
    else{
	    color = vec4(1.0, 1.0, 1.0, 0.0);
    }
}
)