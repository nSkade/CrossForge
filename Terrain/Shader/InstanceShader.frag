#version 330 core 

layout(location = 0) out vec4 gPosition;
layout(location = 1) out vec4 gNormal;
layout(location = 2) out vec4 gAlbedoSpec;

layout (std140) uniform MaterialData{
	vec4 Color;
	float Metallic;
	float Roughness;
	float AO; // ambient occlusion
	float Padding;
}Material;

uniform sampler2D TexAlbedo;
uniform sampler2D TexDepth;

in vec3 Pos;
in vec3 N;
in vec2 UV;
in vec3 test;
in vec3 worldSpacePos;

void main(){
	gPosition = vec4(worldSpacePos, 0.0); //texture(TexAlbedo, UV);
	gNormal = vec4(N, 0.0);
	gAlbedoSpec = texture(TexAlbedo, UV);
	gAlbedoSpec.a = 0.1;
}