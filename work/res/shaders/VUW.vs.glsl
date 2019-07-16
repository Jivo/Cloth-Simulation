#version 330 core

uniform mat4 modelMat;
uniform mat4 viewMat;
uniform mat4 projectionMat;

in vec3 vertPosition;

void main() {
    gl_Position = projectionMat * viewMat * modelMat * vec4(vertPosition, 1.0);
}
