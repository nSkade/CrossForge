#version 420 core

layout (std140) uniform CameraData{
	mat4 ViewMatrix;
	mat4 ProjectionMatrix;
	vec4 Position;
}Camera;

layout(std140) uniform ModelData{
	mat4 ModelMatrix;
};

layout (location = 0) in vec3 Position;
layout (location = 1) in vec3 Normal;
layout (location = 2) in vec3 Tangent;
layout (location = 3) in vec3 UVW;
layout (location = 4) in mat4 instanceMatrix;

out vec3 Pos;
out vec3 N;
out vec3 B;
out vec3 T;
out vec2 UV;
out vec3 CameraPos;


void main(){
	mat4 m = mat4(vec4(1, 0, 0, 0), vec4(0, 1, 0, 0), vec4(0, 0, 1, 0), vec4(0, 0, 0, 1));

	m = instanceMatrix;

	// Normal, Tangent, BiTangent, Position, CameraPosition in WorldSpace
	N = normalize(mat3(m) * Normal);
	T = normalize(mat3(m) * Tangent);
	B = normalize(cross(N, T));
	Pos = (Camera.ProjectionMatrix * Camera.ViewMatrix * m * vec4(Position, 1)).rgb;
	CameraPos = Camera.Position.xyz;


	UV = UVW.xy;
	gl_Position = Camera.ProjectionMatrix * Camera.ViewMatrix * m * vec4(Position, 1);
}