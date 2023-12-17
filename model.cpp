#include "model.h"

unsigned int TextureFromFile(const char* path, const string& directory, bool gamma)
{
    string filename = string(path);
    filename = directory + '/' + filename;

    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char* data = stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}

/*
recursive DFS implementation for rendering a
kinematic tree robot
*/
void render_robot(mat4_t* hw_b, Shader* shader, link_t* node)
{
    if (node == NULL)
        return;

    link_t* node_to_render = node;
    mat4_t hw_lnk;
    mat4_t_mult_pbr(hw_b, &node_to_render->h_base_us, &hw_lnk);
    glm::mat4 model = ht_matrix_to_mat4_t(hw_lnk);
    shader->setMat4("model", model);
    ((AssetModel*)node_to_render->model_ref)->Draw(*shader, NULL);

    for (int i = 0; i < node->num_children; i++)
    {
        joint2* j = &node->joints[i];
        render_robot(hw_b, shader, j->child);
    }
}
