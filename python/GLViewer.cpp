//
// Created by DSlobodzian on 11/13/2021.
//
#include "GLViewer.hpp"
#include <cuda_gl_interop.h>

GLchar* VERTEX_SHADER =
        "#version 330 core\n"
        "layout(location = 0) in vec3 in_Vertex;\n"
        "layout(location = 1) in vec3 in_Color;\n"
        "uniform mat4 u_mvpMatrix;\n"
        "out vec3 b_color;\n"
        "void main() {\n"
        "   b_color = in_Color;\n"
        "	gl_Position = u_mvpMatrix * vec4(in_Vertex, 1);\n"
        "}";

GLchar* FRAGMENT_SHADER =
        "#version 330 core\n"
        "in vec3 b_color;\n"
        "layout(location = 0) out vec4 out_Color;\n"
        "void main() {\n"
        "   out_Color = vec4(b_color, 1);\n"
        "}";

GLchar* IMAGE_FRAGMENT_SHADER =
        "#version 330 core\n"
        " in vec2 UV;\n"
        " out vec4 color;\n"
        " uniform sampler2D texImage;\n"
        " uniform bool revert;\n"
        " uniform bool rgbflip;\n"
        " void main() {\n"
        "    vec2 scaler  =revert?vec2(UV.x,1.f - UV.y):vec2(UV.x,UV.y);\n"
        "    vec3 rgbcolor = rgbflip?vec3(texture(texImage, scaler).zyx):vec3(texture(texImage, scaler).xyz);\n"
        "    color = vec4(rgbcolor,1);\n"
        "}";

GLchar* IMAGE_VERTEX_SHADER =
        "#version 330\n"
        "layout(location = 0) in vec3 vert;\n"
        "out vec2 UV;"
        "void main() {\n"
        "   UV = (vert.xy+vec2(1,1))/2;\n"
        "	gl_Position = vec4(vert, 1);\n"
        "}\n";


GLViewer* currentInstance_ = nullptr;

GLViewer::GLViewer() : available_(false) {
    currentInstance_ = this;
}

GLViewer::~GLViewer() {}

void GLViewer::exit() {
    if (available_) {
        image_handler_.close();
    }
    available_ = false;
}

bool GLViewer::isAvailable() {
    if (available_) {
        glutMainLoopEvent();
    }
    return available_;
}

void CloseFunc(void) {
    if (currentInstance_) {
        currentInstance_ ->exit();
    }
}

bool GLViewer::init(int argc, char **argv, sl::CameraParameters cameraLeft) {
    glutInit(&argc, argv);
    int window_width_ = glutGet(GLUT_SCREEN_WIDTH);
    int window_height_ = glutGet(GLUT_SCREEN_HEIGHT);
    int width = window_height_ * 0.9;
    int height = window_height_ * 0.9;

    glutInitWindowSize(width, height);
    glutInitWindowPosition(window_width_ * 0.05, window_height_ * 0.05);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutCreateWindow("Test Window");

    reshapeCallback(width, height);

    GLenum err = glewInit();
    if (GLEW_OK != err) {
        std::cout << "ERROR: glew init failed: " << glewGetErrorString(err) << std::endl;
        return true;
    }
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    mainShader_.shader_ = Shader(VERTEX_SHADER, FRAGMENT_SHADER);
    mainShader_.MVP_Mat = glGetUniformLocation(mainShader_.shader_.getProgramId(), "u_mvpMatrix");

    box_ = Simple2dObject(sl::float2(0,0), false);
    box_.setDrawingType(GL_TRIANGLE_FAN);
    box_.addCircle(0,0, 0.1, 200, box_);

//    box_.addPoint(-1.0, -1.0, 0, 0, 255);
//    box_.addPoint(-1.0, 0.0, 0, 0, 255);
//    box_.addPoint(0.0, 0.0, 0, 0, 255);
    box_.pushToGPU();



    bool status_ = image_handler_.initialize(cameraLeft.image_size);
    if (!status_) {
        std::cout << "ERROR: Failed to initialize Image Renderer" << std::endl;
        return true;
    }

    glutDisplayFunc(GLViewer::drawCallback);
    glutReshapeFunc(GLViewer::reshapeCallback);
    glutKeyboardFunc(GLViewer::keyPressedCallback);
    glutKeyboardUpFunc(GLViewer::keyReleasedCallback);
    glutCloseFunc(CloseFunc);

    available_ = true;
    return false;
}

void GLViewer::drawCallback() {
    currentInstance_ -> render();
}

void GLViewer::idle() {
    glutPostRedisplay();
}

void GLViewer::reshapeCallback(int width, int height) {
    glViewport(0, 0, width, height);
    currentInstance_->window_width_ = width;
    currentInstance_->window_height_ = height;
}

void GLViewer::keyPressedCallback(unsigned char c, int x, int y) {
    currentInstance_->keyStates_[c] = KEY_STATE::DOWN;
    glutPostRedisplay();
}

void GLViewer::keyReleasedCallback(unsigned char c, int x, int y) {
    currentInstance_->keyStates_[c] = KEY_STATE::UP;
}

void GLViewer::updateImage(sl::Mat &image) {
    if (mtx_.try_lock()) {
        if (available_) {
            image_handler_.pushNewImage(image);
        }
        mtx_.unlock();
    }
}

void GLViewer::render() {
    if (available_) {
        mtx_.lock();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0, 0, 0, 1.f);
        update();
        draw();
        mtx_.unlock();
        glutSwapBuffers();
        glutPostRedisplay();
    }
}

void GLViewer::update() {
    if (keyStates_['w'] == KEY_STATE::DOWN) {
        box_.translate(sl::Translation(0, 0.01, 0));
    }
    if (keyStates_['s'] == KEY_STATE::DOWN) {
        box_.translate(sl::Translation(0, -0.01, 0));
    }
    if (keyStates_['d'] == KEY_STATE::DOWN) {
        box_.translate(sl::Translation(0.01, 0, 0));
    }
    if (keyStates_['a'] == KEY_STATE::DOWN) {
        box_.translate(sl::Translation(-0.01, 0, 0));
    }
    if (keyStates_['l'] == KEY_STATE::DOWN) {
        box_.rotate(sl::Rotation(0.01, sl::Translation(0,0,1)));
    }
    if (keyStates_['k'] == KEY_STATE::DOWN) {
        box_.rotate(sl::Rotation(-0.01, sl::Translation(0,0,1)));
    }
}

void GLViewer::draw() {
    const sl::Transform vpMatrix = sl::Transform();
    if (available_) {
        image_handler_.draw();

        glUseProgram(mainShader_.shader_.getProgramId());
        glUniformMatrix4fv(mainShader_.MVP_Mat, 1, GL_FALSE, (sl::Transform::transpose(box_.getModelMatrix())).m);
        box_.draw();
    }
}


Shader::Shader(GLchar* vs, GLchar* fs) {
    if (!compile(vertexId_, GL_VERTEX_SHADER, vs)) {
        std::cout << "ERROR: while compiling vertex shader" << std::endl;
    }
    if (!compile(fragmentId_, GL_FRAGMENT_SHADER, fs)) {
        std::cout << "ERROR: while compiling fragment shader" << std::endl;
    }

    programId_ = glCreateProgram();

    glAttachShader(programId_, vertexId_);
    glAttachShader(programId_, fragmentId_);

    glBindAttribLocation(programId_, ATTRIB_VERTICES_POS, "in_vertex");

    glLinkProgram(programId_);

    GLint errorlk(0);
    glGetProgramiv(programId_, GL_LINK_STATUS, &errorlk);
    if (errorlk != GL_TRUE) {
        std::cout << "ERROR: while linking Shader: " << std::endl;
        GLint errorSize(0);
        glGetProgramiv(programId_, GL_INFO_LOG_LENGTH, &errorSize);

        char *error = new char[errorSize + 1];
        glGetShaderInfoLog(programId_, errorSize, &errorSize, error);
        error[errorSize] = '\0';
        std::cout << error << std::endl;

        delete[] error;
        glDeleteProgram(programId_);
    }
}

Shader::~Shader() {
    if (vertexId_ != 0) {
        glDeleteShader(vertexId_);
    }
    if (fragmentId_ != 0) {
        glDeleteShader(fragmentId_);
    }
    if (programId_ != 0) {
        glDeleteShader(programId_);
    }
}

GLuint Shader::getProgramId() {
    return programId_;
}

bool Shader::compile(GLuint &shaderId, GLenum type, GLchar *src) {
    shaderId = glCreateShader(type);
    if (shaderId == 0) {
        std::cout << "ERROR: shader type (" << type << ") does not exist" << std::endl;
        return false;
    }

    glShaderSource(shaderId, 1, (const char**) &src, 0) ;
    glCompileShader(shaderId);

    GLint errorCp(0);
    glGetShaderiv(shaderId, GL_COMPILE_STATUS, &errorCp);
    if (errorCp != GL_TRUE) {
        std::cout << "ERROR: while compiling Shader :" << std::endl;
        GLint errorSize(0);
        glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &errorSize);
        char *error = new char[errorSize + 1];
        glGetShaderInfoLog(shaderId, errorSize, &errorSize, error);
        error[errorSize] = '\0';
        std::cout << error << std::endl;

        delete[] error;
        glDeleteShader(shaderId);
        return false;
    }
    return true;
}

Simple2dObject::Simple2dObject() : isStatic_(false) {
    vaoID_ = 0;
    drawingType_ = GL_TRIANGLES;
    position_ = sl::float3(0, 0, 0);
    rotation_.setIdentity();
}

Simple2dObject::Simple2dObject(sl::float2 position, bool isStatic) : isStatic_(isStatic) {
    vaoID_ = 0;
    drawingType_ = GL_TRIANGLES;
    position_ = sl::float3(position.x, position.y, 0);
    rotation_.setIdentity();
}

void Simple2dObject::addPoint(sl::float2 position, sl::float3 colors) {
    addPoint(position.x, position.y, colors.r, colors.g, colors.b);
}

void Simple2dObject::addPoint(float x, float y, float r, float g, float b) {
    vertices_.push_back(x);
    vertices_.push_back(y);
    vertices_.push_back(0);
    colors_.push_back(r);
    colors_.push_back(g);
    colors_.push_back(b);
}

void Simple2dObject::addLine(sl::float2 p1, sl::float2 p2, sl::float3 clr) {
    vertices_.push_back(p1.x);
    vertices_.push_back(p1.y);
    vertices_.push_back(0);

    vertices_.push_back(p2.x);
    vertices_.push_back(p2.y);
    vertices_.push_back(0);

    colors_.push_back(clr.r);
    colors_.push_back(clr.g);
    colors_.push_back(clr.b);

    colors_.push_back(clr.r);
    colors_.push_back(clr.g);
    colors_.push_back(clr.b);
}

void Simple2dObject::pushToGPU() {
    if (!isStatic_ || vaoID_ == 0) {
        if (vaoID_ == 0) {
            glGenVertexArrays(1, &vaoID_);
            glGenBuffers(2, vboID_);
        }

        glBindVertexArray(vaoID_);
        if (vertices_.size()) {
            glBindBuffer(GL_ARRAY_BUFFER, vboID_[0]);
            glBufferData(
                    GL_ARRAY_BUFFER,
                    vertices_.size() * sizeof(float),
                    &vertices_[0],
                    isStatic_ ? GL_STATIC_DRAW : GL_DYNAMIC_DRAW);
            glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 2, GL_FLOAT, GL_FALSE, 0, 0);
            glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);
        }

        if (colors_.size()) {
            glBindBuffer(GL_ARRAY_BUFFER, vboID_[1]);
            glBufferData(
                    GL_ARRAY_BUFFER,
                    vertices_.size() * sizeof(float),
                    &colors_[0],
                    isStatic_ ? GL_STATIC_DRAW : GL_DYNAMIC_DRAW);
            glVertexAttribPointer(Shader::ATTRIB_COLOR_POS, 2, GL_FLOAT, GL_FALSE, 0, 0);
            glEnableVertexAttribArray(Shader::ATTRIB_COLOR_POS);
        }

        glBindVertexArray(0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

}

void Simple2dObject::clear() {
    vertices_.clear();
    colors_.clear();
}

void Simple2dObject::setDrawingType(GLenum type) {
    drawingType_ = type;
}

void Simple2dObject::addCircle(float cx, float cy, float r, int num_seg, Simple2dObject &obj) {
    for (int i = 0; i < num_seg; i++)   {
        float theta = (2.0f * 3.1415926f * float(i)) / float(num_seg);//get the current angle
        float x = r * cosf(theta);//calculate the x component
        float y = r * sinf(theta);//calculate the y component
        obj.addPoint(x + cx, y + cy, 254, 0, 0);
    }
}

void Simple2dObject::draw() {
    if (vaoID_) {
        glBindVertexArray(vaoID_);
        glDrawArrays(drawingType_, 0, vertices_.size());
        glBindVertexArray(0);
    }
}

Simple2dObject::~Simple2dObject() {
    if (vaoID_ != 0) {
        glDeleteBuffers(2, vboID_);
        glDeleteVertexArrays(1, &vaoID_);
    }
}

void Simple2dObject::translate(const sl::Translation &translation) {
    position_ = position_ + translation;
}

void Simple2dObject::setPosition(const sl::Translation &pos) {
    position_ = pos;
}

void Simple2dObject::rotate(const sl::Orientation &rot) {
    rotation_ = rot * rotation_;
}

void Simple2dObject::rotate(const sl::Rotation &rot) {
    this->rotate(sl::Orientation(rot));
}

sl::Transform Simple2dObject::getModelMatrix() const {
    sl::Transform tmp = sl::Transform::identity();
    tmp.setOrientation(rotation_);
    tmp.setTranslation(position_);
    return tmp;
}

ImageHandler::ImageHandler() {}
ImageHandler::~ImageHandler() {
    close();
}

void ImageHandler::close() {
    glDeleteTextures(1, &imageTex_);
}

bool ImageHandler::initialize(sl::Resolution res) {
    shader.shader_ = Shader(IMAGE_VERTEX_SHADER, IMAGE_FRAGMENT_SHADER);
    texID_ = glGetUniformLocation(shader.shader_.getProgramId(), "texImage");
    static const GLfloat g_quad_vertex_buffer_data[] = {
            -1.0f, -1.0f, 0.0f,
            0.0f, -1.0f, 0.0f,
            -1.0f, 0.0f, 0.0f,
            -1.0f, 0.0f, 0.0f,
            0.0f, -1.0f, 0.0f,
            0.0f, 0.0f, 0.0f
    };

    glGenBuffers(1, &quad_vb_);
    glBindBuffer(GL_ARRAY_BUFFER, quad_vb_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_quad_vertex_buffer_data), g_quad_vertex_buffer_data, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &imageTex_);
    glBindTexture(GL_TEXTURE_2D, imageTex_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, res.width, res.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glBindTexture(GL_TEXTURE_2D, 0);
    cudaError_t err = cudaGraphicsGLRegisterImage(&cuda_gl_resource, imageTex_, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsWriteDiscard);
    return (err == cudaSuccess);
}

void ImageHandler::pushNewImage(sl::Mat& image) {
    cudaArray_t ArrIm;
    cudaGraphicsMapResources(1, &cuda_gl_resource, 0);
    cudaGraphicsSubResourceGetMappedArray(&ArrIm, cuda_gl_resource, 0, 0);
    cudaMemcpy2DToArray(
            ArrIm,
            0,
            0,
            image.getPtr<sl::uchar1>(sl::MEM::GPU),
            image.getStepBytes(sl::MEM::GPU),
            image.getPixelBytes() * image.getWidth(),
            image.getHeight(),
            cudaMemcpyDeviceToDevice);
}

void ImageHandler::draw() {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glEnable(GL_TEXTURE_2D);
    glUseProgram(shader.shader_.getProgramId());
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, imageTex_);
    glUniform1i(texID_, 0);
    //invert y axis and color for this image (since its reverted from cuda array)
    glUniform1i(glGetUniformLocation(shader.shader_.getProgramId(), "revert"), 1);
    glUniform1i(glGetUniformLocation(shader.shader_.getProgramId(), "rgbflip"), 1);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, quad_vb_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glDisableVertexAttribArray(0);
    glUseProgram(0);
    glDisable(GL_TEXTURE_2D);
}




