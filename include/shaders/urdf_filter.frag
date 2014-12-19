#version 140
varying vec3 normal;
uniform int width;
uniform int height;
uniform samplerBuffer depth_texture;

uniform float replace_value;

uniform float z_near;
uniform float z_far;

uniform float max_diff;

float to_linear_depth (float d)
{
  return (z_near * z_far / (z_near - z_far)) / (d - z_far / (z_far - z_near));

}

void main(void)
{
  float sensor_depth = max(0.0, texelFetch (depth_texture, int(gl_FragCoord.y)*width + int(gl_FragCoord.x)).x);
  //float virtual_depth = to_linear_depth (gl_FragCoord.z);
  float virtual_depth = (gl_FragCoord.z / gl_FragCoord.w);
  bool should_filter = (sensor_depth - virtual_depth) > max_diff || sensor_depth == 0.0;

  // first color attachment: sensor depth image
  gl_FragData[0] = vec4 (sensor_depth, sensor_depth, sensor_depth, 1.0);

  // second color attachment: opengl depth image
  gl_FragData[1] = should_filter ? vec4 (replace_value, 0.0, 0.0, 1.0) : vec4 (sensor_depth, sensor_depth, sensor_depth, 1.0);

  // third color attachment: normal visualization
  gl_FragData[2] = vec4 ((normal.x + 1.0) * 0.5,
                         (normal.y + 1.0) * 0.5,
                         (normal.z + 1.0) * 0.5,
                         1.0);

  // fourth color attachment: mask image
  gl_FragData[3] = vec4(should_filter, 0.0, 0.0, 1.0);

  // fourth color attachment: label image
  gl_FragData[4] = should_filter ? vec4(16.0, 0.0, 0.0, 1.0) : vec4(0.0, 1.0, 1.0, 1.0);
}

		
