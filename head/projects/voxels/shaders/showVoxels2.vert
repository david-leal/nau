#version 440

in vec4 position;
in vec3 normal;

uniform mat4 PVM;
uniform int GridSize;

out vec3 normalV;
flat out ivec3 posW;

void main()
{
	ivec4 pos;
	vec4 posF;
	int inst = gl_InstanceID;
	int k = GridSize * GridSize;
	pos.x = inst / (k);
	pos.y = (inst - pos.x * (k)) / GridSize;
	pos.z = inst - pos.x * (k) - pos.y * GridSize;
	pos.w = 1;
	posF = pos + position* 0.95;
	posF /= GridSize;
	posF = 2 * posF - vec4(1.0, 1.0, 1.0, 0);
	posF.w = 1;
    gl_Position = PVM * posF;
	posW = pos.xyz;
	normalV = normal;
}