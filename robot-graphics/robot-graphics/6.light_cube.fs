#version 330 core

uniform vec4 obj_color;

out vec4 FragColor;

void main()
{
    FragColor = obj_color; // set alle 4 vector values to 1.0
}