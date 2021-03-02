#include <iostream>             // for cout
#include <fstream>
#include <GL/glew.h>     // has to be included before gl.h, or any header that includes gl.h
#include <GL/freeglut.h>

GLuint LoadShader(const char *vertex_path, const char *fragment_path);