#version 120
in vec3 vertex;
varying out vec3 normal;

void main() {
  gl_Position = gl_ModelViewProjectionMatrix * vec4(vertex, 1.0);

  vec3 temp = gl_NormalMatrix * gl_Normal;
  normal = vec3 (-temp.x, temp.y, -temp.z);
}

/*
  // For inflating mesh in the shader instead of with the cpu
  float padding_distance = 0.0;
  vec4 vertex = gl_ModelViewMatrix * gl_Vertex;
  vec3 n = gl_NormalMatrix * gl_Normal;

  gl_Position = gl_ProjectionMatrix * vec4(vertex.xyz + padding_distance * n, 1.0);

  normal = vec3 (-n.x, n.y, -n.z);
*/
