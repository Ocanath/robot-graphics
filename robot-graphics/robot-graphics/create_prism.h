#ifndef  CREATE_PRISM_H
#define CREATE_PRISM_H

void create_prism_voxel(float triangularprism_vertices[8 * 3 * 8]);
void draw_prism(Shader* shader, unsigned int prismVAO);

#endif // ! CREATE_PRISM_H
