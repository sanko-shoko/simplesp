SP_SHADER(
#version 420 core \n

layout(location = 0) in vec2 vtx;
out vec2 uv;

void main(void){
    uv = (vtx + 1.0) * 0.5;
    gl_Position = vec4(vtx, 0.0, 1.0);
}
)