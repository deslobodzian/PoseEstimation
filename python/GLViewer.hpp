//
// Created by DSlobodzian on 11/13/2021.
//
#pragma once

//ZED sdk stuff
#include <sl/Camera.hpp>
#include <GL/glew.h>
#include <GL/freeglut.h>
// C++ stuff
#include <iostream>


class Shader {
private:
    bool compile(GLuint &shaderId, GLenum type, GLchar* src);
    GLuint vertexId_;
    GLuint fragmentId_;
    GLuint programId_;
public:
    Shader() {}
    Shader(GLchar* vs, GLchar* fs);
    ~Shader();

    GLuint getProgramId();

    static const GLint ATTRIB_VERTICES_POS = 0;
    static const GLint ATTRIB_COLOR_POS = 1;
};

struct ShaderData {
    Shader shader_;
    GLuint MVP_Mat;
    GLuint shColorLoc;
};

class Simple2dObject {
private:
    std::vector<float> vertices_;
    std::vector<float> colors_;

    bool isStatic_;

    GLenum drawingType_;
    GLuint vaoID_;
    /*
     Vertex buffer IDs:
     - [0]: Vertices coordinates.
     - [1]: RGB color values.
     */
    GLuint vboID_[2];

    sl::Translation position_; // coordinate is [x y z] but z will be 0 in 2D case.
    sl::Orientation rotation_;

public:
    Simple2dObject();
    Simple2dObject(sl::float2 position, bool isStatic);
    ~Simple2dObject();

    void addPoint(float x, float y, float r, float g, float b);
    void addLine(sl::float2 p1, sl::float2 p2, sl::float3 clr);
    void addPoint(sl::float2 position, sl::float3 color);
    void pushToGPU();
    void clear();

    void setDrawingType(GLenum type);
    void addCircle(float cx, float cy, float r, int num_seg, Simple2dObject &obj);

    void draw();

    void translate(const sl::Translation &translation);
    void setPosition(const sl::Translation &pos);

    void rotate(const sl::Rotation &rot);
    void rotate(const sl::Orientation &rot);

    sl::Transform getModelMatrix() const;

    void generateCircle(float radius, int numSides);
};

class ImageHandler {
private:
    GLuint texID_;
    GLuint imageTex_;
    cudaGraphicsResource* cuda_gl_resource;
    ShaderData shader;
    GLuint quad_vb_;
public:
    ImageHandler();
    ~ImageHandler();

    bool initialize(sl::Resolution res);
    void pushNewImage(sl::Mat& image);
    void draw();
    void close();
};


class GLViewer {
private:
    // Rendering loop method called each frame
    void render();
    // Everything that needs to be updated before rendering must be done in this method
    void update();
    // Once everything is updated, every render-able objects must be drawn in this method.
    void draw();

    static void drawCallback();
    static void reshapeCallback(int width, int height);
    static void keyPressedCallback(unsigned char c, int x, int y);
    static void keyReleasedCallback(unsigned char c, int x, int y);
    static void idle();

    std::mutex mtx_;

    bool available_;

    enum KEY_STATE {
        UP = 'u',
        DOWN = 'd',
        FREE = 'f'
    };

    KEY_STATE keyStates_[256];

    Simple2dObject box_;
    Simple2dObject circle_;
    ImageHandler image_handler_;
    int window_width_, window_height_;

    ShaderData mainShader_;
public:
    GLViewer();
    ~GLViewer();

    bool isAvailable();

    void setPosition(const sl::Translation &pos);

    void updateImage(sl::Mat &image);
    bool init(int argc, char** argv, sl::CameraParameters cameraLeft);

    void exit();

};

