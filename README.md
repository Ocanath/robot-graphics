Getting started/Compiling:
0. This project should compile standalone in Visual Studio 2019. If it does not (or you need to regenerate the libraries 
for some reason) follow the remaining steps:

For GLFW:
1. clone GLFW
2. Get CMake. type "cmake CMakeLists.txt" in the command line and it'll create some visual studio projects for you,
	compile them there.
3. Copy the .lib files to lib-debug and lib-release
4. Copy the header information to external

For glm:
1. GLM is a header only library, so no compilation is necessary. Just pull it and copy the relevant files to external

For GLAD:
1. GLAD pretty much consists of two header files and a .c file. Look up GLAD online, their website has a little 
	web client that lets you select files for up-to-date OpenGL extensions (currently on 4.something)

For Bullet3:

