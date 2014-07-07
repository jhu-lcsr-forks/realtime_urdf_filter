#version 120
varying vec3 normal;

void main() {
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

  vec3 temp = gl_NormalMatrix * gl_Normal;
  normal = vec3 (-temp.x, temp.y, -temp.z);
}

