#pragma once

#include "opengl.hpp"

#include "glm/glm.hpp"

namespace cgra {

    class Program {
        // The OpenGL object representing the program
        GLuint m_program;

        Program(GLuint prog) : m_program(prog) { }
    public:
        Program() : m_program(0) { }

        // Load the program from two files, a vertex shader and
        // a fragment shader.
        static Program load_program(const char *vertex_shader_file,
                                    const char *fragment_shader_file);

        // Tells OpenGL to use this shader program
        void use();

        // Sets the model matrix for the program.
        // This controls the position, orientation and
        // scale of the model.
        void setModelMatrix(const glm::mat4 &);

        // Sets the view matrix for the program.
        // This controls the position and orientation
        // of the camera.
        void setViewMatrix(const glm::mat4 &);
        // Sets the projection matrix for the program.
        // This specifies how to convert from 3D coordinates to
        // 2D coordinates
        void setProjectionMatrix(const glm::mat4 &);
    };
}
