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

For Assimp:
1. Clone repository 
2. Open CMAKE and navigate to the cloned root directory
3. make a new folder called 'build' as the cmake target, inside the assimp directory
4. generate, open VS project, build all. Be sure to do a 'release' and a 'debug' build
5. link dlls and headers to the project (headers should be added in this distro)

For Bullet3:

1. Pull Bullet
2. Use built in script (or cmake) to make visual studio projects
3. Use example code to figure out what .libs you need
4. compile the code for each lib. be sure to switch to MD (multithreaded-dll) for compatibility
5. match EXACTLY the same project settings. The one that caused linker errors just now was the 'preprocessor definitions'. 
