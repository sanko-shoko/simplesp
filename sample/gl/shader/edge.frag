#version 420 core

in vec2 vtx;
out vec3 color;

layout(binding=0) uniform sampler2D depth;
layout(binding=1) uniform sampler2D image;

const float dx = 0.001953125;
const float dy = 0.001953125;

float peek(const in float x, const in float y)
{
  return texture2D(depth, vec2(x, y));
}
void main(){
  float x = vtx.x;
  float y = vtx.y;
  float v = peek(x, y);

  if(v > 0.9999999){
    color = vec3(0.0, 0.0, 1.0);
  }
  else{
    color = vec3(1.0, 0.0, 1.0);
  }
}