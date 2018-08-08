#version 420 core
layout(location = 0) in vec2 vtx_in;
out vec2 vtx;

void main(void){
  vtx.x = vtx_in.x;
  vtx.y = vtx_in.y;
  gl_Position = vec4(vtx_in * 2.0 - 1.0, 0.0, 1.0);
}