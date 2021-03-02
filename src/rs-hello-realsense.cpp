// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <iostream>             // for cout
#include <fstream>
#include <Eigen/Dense>

#include <GL/glew.h>     // has to be included before gl.h, or any header that includes gl.h
#include <GL/freeglut.h>

#include "shader.h"
#include "realsense_handler.h"

int draw_width = -1;
int draw_height = -1;    
RealsenseHandler realsense_handler = RealsenseHandler();
auto spatial_filter = rs2::spatial_filter();

GLdouble intrinsics_cv[3*3];
GLdouble intrinsics[4*4];
GLdouble extrinsics[4*4];
GLuint verts_vao = 0;
GLuint verts_vbo = 0;
GLuint colors_vao = 0;
GLuint colors_vbo = 0;
GLuint program = 0;
int num_verts_in_vbo  = 0;

float vertices[] = {
     0.0f,  0.5f, // Vertex 1 (X, Y)
     0.5f, -0.5f, // Vertex 2 (X, Y)
    -0.5f, -0.5f  // Vertex 3 (X, Y)
};

bool load_data_file(GLdouble * out_array, int total_size, std::string path){
    // Load in file with space-and-newline separated floats.
    std::ifstream file(path);
    double in;
    int k = 0;
    while (file >> in){
        out_array[k] = in;
        k += 1;
    }
    if (k != total_size){
        fprintf(stdout, "Didn't load expected elements (%d loaded vs %d expected) from %s.\n",
        k, total_size, path.c_str());
    }
    return k == total_size;
}

bool load_projector_calibration(){
    std::string intr_file = "../projector_intrinsics.csv";
    std::string extr_file = "../extrinsics.csv";
    
    if (!load_data_file(intrinsics_cv, sizeof(intrinsics_cv)/sizeof(GLdouble), intr_file))
        return false;
    if (!load_data_file(extrinsics, sizeof(extrinsics)/sizeof(GLdouble), extr_file))
        return false;

    // Convert opencv-style intrinsics (standard camera projection matrix)
    // to opengl intrinsics matrix.
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> K(intrinsics_cv);
    double near = 0.01;
    double far = 20.0;
    double fx = K(0, 0);
    double fy = K(1, 1);
    double cx = K(0, 2);
    double cy = K(1, 2);
    double width = cx * 2;
    double height = cy * 2;
    Eigen::Matrix<double, 4, 4> K_gl;
    K_gl << -fx, 0., -cx, 0.,
        0., -fy, -cy, 0.,
        0., 0., near + far, near*far,
        0., 0., 1., 0.;
    Eigen::Matrix<double, 4, 4> NDC;
    NDC << 2. / width, 0., 0., -1.,
        0., 2 / height, 0., -1.,
        0., 0., -2. / (far - near), -(far + near)/(far - near),
        0., 0., 0., 1.;
    Eigen::Matrix<double, 4, 4> intr_gl = NDC * K_gl;
    for (int k = 0; k < 4*4; k++){
        intrinsics[k] = intr_gl.data()[k];
    }
    return true;
}

bool setup_vbos(){
    int w = realsense_handler.get_width();
    int h = realsense_handler.get_height();

    program = LoadShader("../shaders/vert.glsl", "../shaders/frag.glsl");
    printf("Done compiling\n");

    num_verts_in_vbo = w*h;
    glGenVertexArrays(1, &verts_vao);
    glBindVertexArray(verts_vao);
    glGenBuffers(1, &verts_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, verts_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * w * h, NULL, GL_DYNAMIC_DRAW);
    //glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * 3, vertices, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    //glVertexAttribPointer(vertexAttrib, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glBindVertexArray(0);
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glGenVertexArrays(1, &colors_vao);
    glBindVertexArray(colors_vao);
    glGenBuffers(1, &verts_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, verts_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * w * h, NULL, GL_DYNAMIC_DRAW);
    //glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * 3, vertices, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    //glVertexAttribPointer(vertexAttrib, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    return true;
}

void set_projector_intrinsics(){
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMultMatrixd(intrinsics);
}

void set_projector_extrinsics(){
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotated(180., 0., 1., 0.);
    glMultMatrixd(extrinsics);
}


void draw()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LINE_SMOOTH);

    glViewport(0, 0, draw_width, draw_height);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // Set view intrinsics to exactly projector intrinsics
    set_projector_intrinsics();
    // Set camera pos to precisely match projector pos in
    // RGBD frame
    set_projector_extrinsics();    

    // Draw points from buffer.
    glColor3f(1, 1, 1);
    glEnable(GL_MULTISAMPLE);
    glDisable(GL_LIGHTING);

    glBindVertexArray(verts_vao);
    glBindBuffer(GL_ARRAY_BUFFER, verts_vbo);
    
    //glUseProgram(program);
    glDrawArrays(GL_POINTS, 0, num_verts_in_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glutSwapBuffers();   // swapping image buffer for double buffering
    glutPostRedisplay(); // redrawing. Omit this line if you don't want constant redraw
}

void idle()
{
    realsense_handler.get_and_process_frame();
    auto points = realsense_handler.get_current_pointcloud();
    float * verts = (float *) points.get_vertices();
    // Spatial filtering
    // points = points.apply_filter(spatial_filter);
    glBindBuffer(GL_ARRAY_BUFFER, verts_vbo);
    fprintf(stdout, "Points size %d, data size %d\n", points.size(), points.get_data_size());
    glBufferData(verts_vbo, points.size()*3*sizeof(float), verts, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    fprintf(stdout, "Framerate %f, %lu points\n",
        realsense_handler.get_current_framerate(),
        points.size());
}

bool setup_window(int argc, char** argv){
    // Start GL window
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA); // enabling double buffering and RGBA
    glutInitWindowSize(1280, 768);
    
    glutGameModeString("1280x768:24");
    glutEnterGameMode(); 
    glutSetCursor(GLUT_CURSOR_NONE);

    glutDisplayFunc(draw); 
    glutIdleFunc(idle);
    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        /* Problem: glewInit failed, something is seriously wrong. */
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
    }
    fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));

    draw_width = glutGet(GLUT_WINDOW_WIDTH);
    draw_height = glutGet(GLUT_WINDOW_HEIGHT);

    fprintf(stdout, "Fullscreen at res %d x %d\n", draw_width, draw_height);

    return GLEW_OK == err;
}


int main(int argc, char * argv[]) try
{
    if (!setup_window(argc, argv)){
        return -1;
    }
    if (!load_projector_calibration()){
        fprintf(stdout, "Failed to load intr/extr: bad formatting?\n");
        return -1;
    }
    if (!setup_vbos()){
        fprintf(stdout, "Failed to setup vbos.\n");
        return -1;
    }
    glutMainLoop();
    return EXIT_SUCCESS;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
