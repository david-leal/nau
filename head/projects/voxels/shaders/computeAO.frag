#version 440

in vec2 texCoordV;

out vec4 outColor;

uniform sampler2D texUnit;
uniform sampler3D grid;
uniform sampler3D gridNormal;

float voxelConeTrace(vec3 origin, vec3 dir, float coneRatio, float maxDist) {

	vec3 samplePos = origin;
	float accum = 0.0;
	
	float minDiameter = 1.0/1024.0;
	
	float startDist =  2 * minDiameter;
	float dist = startDist;
	
	while(dist < maxDist && accum < 1.0) {
	
		float sampleDiameter = max(minDiameter, coneRatio * dist);
		
		// convert diameter to LOD
		// for example:
		// -log2(1/256) = 8
		// -log2(1/128) = 7
		// -log2(1/64) = 6
		float sampleLOD = -log2(sampleDiameter);
		
		vec3 samplePos = origin + dir * dist;
		float level =  min(0, 9 - sampleLOD);
		//if (sampleLOD >= 5 && sampleLOD < 6) 
		{
		vec4 sampleValue = textureLod(grid, samplePos, level);
		//sampleValue.a = 1.0 - pow(1.0 - sampleValue.a, sampleDiameter/(minDiameter));
		//vec4 sampleValue = texelFetch(grid, ivec3(samplePos*512/pow(2.0,level)), level);	
		//sampleValue.a /= sampleDiameter/minDiameter;
		accum += sampleValue.a;
		}
		dist += sampleDiameter/2.0;
	}
	return accum;
}

void main()
{
	vec3 coord = texture(texUnit, texCoordV).xyz;
	ivec3 coordi = ivec3(coord * 512);
	vec4 texel = texelFetch(grid, coordi, 0);
	if (texel.w == 0)
		discard;
	vec3 normal = texelFetch(gridNormal, coordi, 0).xyz;
	normal = normalize(normal*2.0 - 1.0);
	vec3 tangent, bitangent;
	vec3 c1 = cross(normal, vec3(0,0,1));
	vec3 c2 = cross(normal, vec3(0,1,0));
	if (length(c1) > length(c2)) 
		tangent = c1;
	else 
		tangent = c2;
		
	tangent = normalize(tangent);
	bitangent = cross(normal, tangent);
	
	float coneRatio = 1;
	float maxDist = 0.005;
	float ao;
	
	ao  = voxelConeTrace(coord, normalize(normal), coneRatio, maxDist);
	ao += 0.707 * voxelConeTrace(coord, normalize(normal+tangent), coneRatio, maxDist);
	ao += 0.707 * voxelConeTrace(coord, normalize(normal-tangent), coneRatio, maxDist);
	ao += 0.707 * voxelConeTrace(coord, normalize(normal+bitangent), coneRatio, maxDist);
	ao += 0.707 * voxelConeTrace(coord, normalize(normal-bitangent), coneRatio, maxDist);
	
	outColor = vec4(1 - 2* ao);
	//outColor = vec4(normal,0);
	//texel = texelFetch(grid, coordi/4, 2);	
	//outColor = vec4(texel.a);
	//outColor = vec4(texel);
}