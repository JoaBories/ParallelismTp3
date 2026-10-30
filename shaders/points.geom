#version 330 core
layout(points) in;
layout(triangle_strip, max_vertices = 3) out;

in vec2 vVel[];
uniform mat4 uView;
uniform mat4 uProj;

void main() {
  vec2 p = gl_in[0].gl_Position.xy;
  vec2 d = vVel[0];
  float len = length(d);
  vec2 fwd = len > 1e-4 ? d / len : vec2(1.0, 0.0);
  vec2 side = vec2(-fwd.y, fwd.x);
  float nose = 0.14;
  float tail = 0.07;
  float halfw = 0.055;

  gl_Position = uProj * uView * vec4(p + fwd * nose, 0.0, 1.0);
  EmitVertex();
  gl_Position = uProj * uView * vec4(p - fwd * tail + side * halfw, 0.0, 1.0);
  EmitVertex();
  gl_Position = uProj * uView * vec4(p - fwd * tail - side * halfw, 0.0, 1.0);
  EmitVertex();
  EndPrimitive();
}
