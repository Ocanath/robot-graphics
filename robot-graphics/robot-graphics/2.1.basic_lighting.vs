#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

out vec3 FragPos;
out vec3 Normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    FragPos = vec3(model * vec4(aPos, 1.0));
	
	for(int r = 0; r < 3; r++)	//apply model rotation to the normal
	{
		float tmp = 0;
		for(int c = 0; c < 3; c++)
		{
			tmp = tmp + model[c][r] * aNormal[c];
		}
		Normal[r] = tmp;
	}

    gl_Position = projection * view * vec4(FragPos, 1.0);
}