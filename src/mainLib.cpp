#include <windows.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <thread>
#include <GL/freeglut.h>
#include <GL/freeglut_ext.h>
#include <iostream>
#include "GLShader.hpp"


#define BUFFER_OFFSET(i) ((void*)(i))
float*  left_uv_to_rect_x = new float[16]{-0.7530364531010308, 0.8806592947908687, -0.8357813137161849, 0.3013989721607643, 0.9991764544369446, -0.2578159567698274, 0.3278667335649757, -0.4602577277109663, -0.23980700925448195, -0.056891370605734376, -0.1248008903440144, 0.7641381600051023, 0.20935445281014292, -0.06256983016261788, 0.25844580123833516, -0.5098143951663658};
float*  left_uv_to_rect_y = new float[16]{ 0.5612597403791647, -1.1899589356849427, 0.4652815794139322, -0.2737933233160801, 0.3774305703820774, -0.8110333901413378, 1.2705775357104372, -0.7461290557575936, -0.19222925521894155, 0.936404121235537, -1.7109388784623627, 0.9147182510080394, 0.33073407860855586, -1.1463700238163494, 1.4965795269835196, -0.7090919632511286};
float* right_uv_to_rect_x = new float[16]{ -0.2117125319456463, -0.432262579698108, 0.41675063901331316, -0.14650788483832153, 1.0941580384494245, -0.30628109185189906, 0.109119134429531, 0.11642874201014344, -0.2761527408488216, -0.4335709010559027, 0.9626491769528918, -0.5572405188216735, 0.18342869894719088, 0.37981945016058366, -0.8718621504058989, 0.5218968716935535};
float* right_uv_to_rect_y = new float[16]{ 1.0129568069314265, -2.110976542118192, 1.4108474581893895, -0.7746290913232183, -0.746419837008027, 1.747642287758405, -1.5753294007072252, 0.7143402603200871, 0.5607717274125551, -1.5019493985594772, 1.2539128525783017, -0.42999735712430215, -0.21517910830152714, 0.5965062719847273, -0.5664205050494074, 0.18545738302854597};
int unloadshader(GLubyte** ShaderSource)
{
    if (*ShaderSource != 0)
        delete[] * ShaderSource;
    *ShaderSource = 0;
    return 0;
}

enum class Color { Red, Green, Blue, Black, White, Yellow, Orange };

class  Debug
{
public:
    static void Log(const char* message, Color color = Color::Black);
    static void Log(const std::string message, Color color = Color::Black);
    static void Log(const int message, Color color = Color::Black);
    static void Log(const char message, Color color = Color::Black);
    static void Log(const float message, Color color = Color::Black);
    static void Log(const double message, Color color = Color::Black);
    static void Log(const bool message, Color color = Color::Black);

private:
    static void send_log(const std::stringstream& ss, const Color& color);
};



//-------------------------------------------------------------------

#define DLL_EXPORT __declspec(dllexport)
unsigned long getFileLength(std::ifstream& file)
{
    if (!file.good()) return 0;

    unsigned long pos = file.tellg();
    file.seekg(0, std::ios::end);
    unsigned long len = file.tellg();
    file.seekg(std::ios::beg);

    return len;
}
std::string loadFile(const char* fname)
{
    std::ifstream file(fname);
    if (!file.is_open())
    {
        Debug::Log("Unable to open file ", Color::Red);// << fname << endl;
        exit(1);
    }

    std::stringstream fileData;
    fileData << file.rdbuf();
    file.close();

    return fileData.str();
}


// printShaderInfoLog
// From OpenGL Shading Language 3rd Edition, p215-216
// Display (hopefully) useful error messages if shader fails to compile
void printShaderInfoLog(GLint shader)
{
    int infoLogLen = 0;
    int charsWritten = 0;
    char* infoLog;

    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogLen);

    if (infoLogLen > 0)
    {
        infoLog = new char[infoLogLen];
        // error check for fail to allocate memory omitted
        glGetShaderInfoLog(shader, infoLogLen, &charsWritten, infoLog);
        Debug::Log(infoLog,Color::Red);
        delete[] infoLog;
    }
}
int LoadShader(const char* pfilePath_vs, const char* pfilePath_fs, bool bindTexCoord0, bool bindNormal, bool bindColor, GLuint& shaderProgram, GLuint& vertexShader, GLuint& fragmentShader)
{
    shaderProgram = 0;
    vertexShader = 0;
    fragmentShader = 0;

    // load shaders & get length of each
    int vlen;
    int flen;
    std::string vertexShaderString = loadFile(pfilePath_vs);
    std::string fragmentShaderString = loadFile(pfilePath_fs);
    vlen = vertexShaderString.length();
    flen = fragmentShaderString.length();

    if (vertexShaderString.empty())
    {
        return -1;
    }

    if (fragmentShaderString.empty())
    {
        return -1;
    }

    vertexShader = glCreateShader(GL_VERTEX_SHADER);
    fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    const char* vertexShaderCStr = vertexShaderString.c_str();
    const char* fragmentShaderCStr = fragmentShaderString.c_str();
    glShaderSource(vertexShader, 1, (const char**)&vertexShaderCStr, &vlen);
    glShaderSource(fragmentShader, 1, (const char**)&fragmentShaderCStr, &flen);

    GLint compiled;

    glCompileShader(vertexShader);
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &compiled);
    if (compiled == FALSE)
    {
        Debug::Log("Vertex shader not compiled.", Color::Red);
        printShaderInfoLog(vertexShader);

        glDeleteShader(vertexShader);
        vertexShader = 0;
        glDeleteShader(fragmentShader);
        fragmentShader = 0;

        return -1;
    }

    glCompileShader(fragmentShader);
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &compiled);
    if (compiled == FALSE)
    {
        Debug::Log("fragment shader not compiled.", Color::Red);
        printShaderInfoLog(fragmentShader);

        glDeleteShader(vertexShader);
        vertexShader = 0;
        glDeleteShader(fragmentShader);
        fragmentShader = 0;

        return -1;
    }

    shaderProgram = glCreateProgram();

    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);

    glBindAttribLocation(shaderProgram, 0, "InVertex");

    if (bindTexCoord0)
        glBindAttribLocation(shaderProgram, 1, "InTexCoord0");

    if (bindNormal)
        glBindAttribLocation(shaderProgram, 2, "InNormal");

    if (bindColor)
        glBindAttribLocation(shaderProgram, 3, "InColor");

    glLinkProgram(shaderProgram);

    GLint IsLinked;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, (GLint*)&IsLinked);
    if (IsLinked == FALSE)
    {
        Debug::Log("Failed to link shader.", Color::Red);

        GLint maxLength;
        glGetProgramiv(shaderProgram, GL_INFO_LOG_LENGTH, &maxLength);
        if (maxLength > 0)
        {
            char* pLinkInfoLog = new char[maxLength];
            glGetProgramInfoLog(shaderProgram, maxLength, &maxLength, pLinkInfoLog);
            Debug::Log(pLinkInfoLog, Color::Red);
            delete[] pLinkInfoLog;
        }

        glDetachShader(shaderProgram, vertexShader);
        glDetachShader(shaderProgram, fragmentShader);
        glDeleteShader(vertexShader);
        vertexShader = 0;
        glDeleteShader(fragmentShader);
        fragmentShader = 0;
        glDeleteProgram(shaderProgram);
        shaderProgram = 0;

        return -1;
    }

    return 1;		//Success
}



#ifdef __cplusplus  
extern "C" {
    //Globals
    struct TVertex_VT
    {
        float	x, y, z;
        float	s0, t0;
        float	padding[3];
    };

    //Vertex, normal, tex0
    //
    //SIZE : 4+4+4 +4+4+4 +4+4 = 4*8 = 32 bytes
    struct TVertex_VNT
    {
        float	x, y, z;
        float	nx, ny, nz;
        float	s0, t0;
    };

    //Vertex, color
    //
    //SIZE : 4+4+4 +4 = 4*4 = 16 bytes
    //It's better to make it multiple of 32
    //32-16 = 16 bytes (of garbage should be added)
    //16/4 = 4 floats should be added
    struct TVertex_VC
    {
        float	x, y, z;
        unsigned int	color;
        float	padding[4];
    };

//A quad
    GLushort	pindex_quad[6];
    TVertex_VC	pvertex_quad[4];

    //A triangle
    GLushort		pindex_triangle[3];
    TVertex_VNT		pvertex_triangle[3];
    //1 VAO for the quad
    //1 VAO for the triangle
    GLuint VAOID[2];
    //1 IBO for the quad (Index Buffer Object)
    //1 IBO for the triangle
    GLuint IBOID[2];
    //1 IBO for the quad (Vertex Buffer Object)
    //1 IBO for the triangle
    GLuint VBOID[2];

    //1 shader for the quad
    //1 shader for the triangle
    GLuint	ShaderProgram[2];
    GLuint	VertexShader[2];
    GLuint	FragmentShader[2];

    int ProjectionModelviewMatrix_Loc[2];
    float rot = 0;
    bool isLooping = false;
    void timer(int) {
        glutPostRedisplay();
        glutTimerFunc(1000 / 120, timer, 0);
    }
#endif
    void CreateGeometry()
    {
        //A quad
        pvertex_quad[0].x = -1;
        pvertex_quad[0].y = -1;
        pvertex_quad[0].z = -1;
        pvertex_quad[0].color = 0xFFFFFFFF;

        pvertex_quad[1].x = 1;
        pvertex_quad[1].y = -1;
        pvertex_quad[1].z = -1;
        pvertex_quad[1].color = 0xFFFF0000;

        pvertex_quad[2].x = -1;
        pvertex_quad[2].y = 1;
        pvertex_quad[2].z = -1;
        pvertex_quad[2].color = 0xFF00FF00;

        pvertex_quad[3].x = 1;
        pvertex_quad[3].y = 1;
        pvertex_quad[3].z = -1;
        pvertex_quad[3].color = 0xFF0000FF;

        pindex_quad[0] = 0;
        pindex_quad[1] = 1;
        pindex_quad[2] = 2;
        pindex_quad[3] = 2;
        pindex_quad[4] = 1;
        pindex_quad[5] = 3;

        //The triangle

        //Create the IBO for the quad
        //16 bit indices
        glGenBuffers(1, &IBOID[0]);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBOID[0]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(GLushort), pindex_quad, GL_STATIC_DRAW);

        GLenum error = glGetError();

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        error = glGetError();

        //Create VBO for the quad
        glGenBuffers(1, &VBOID[0]);
        glBindBuffer(GL_ARRAY_BUFFER, VBOID[0]);
        glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(TVertex_VC), pvertex_quad, GL_STATIC_DRAW);

        error = glGetError();

        //Just testing
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        error = glGetError();

        //VAO for the quad *********************
        glGenVertexArrays(1, &VAOID[0]);
        glBindVertexArray(VAOID[0]);

        //Bind the VBO and setup pointers for the VAO
        glBindBuffer(GL_ARRAY_BUFFER, VBOID[0]);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(TVertex_VC), BUFFER_OFFSET(0));
        glVertexAttribPointer(3, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(TVertex_VC), BUFFER_OFFSET(sizeof(float) * 3));
        glEnableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glDisableVertexAttribArray(2);
        glEnableVertexAttribArray(3);

        //Bind the IBO for the VAO
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBOID[0]);        

        //Just testing
        glBindVertexArray(0);
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glDisableVertexAttribArray(2);
        glDisableVertexAttribArray(3);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }
    void ExitFunction()
    {
        Debug::Log("Exit called.",Color::Red);

        glBindVertexArray(0);
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glDisableVertexAttribArray(2);
        glDisableVertexAttribArray(3);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        glUseProgram(0);

        glDeleteBuffers(1, &IBOID[0]);
        glDeleteBuffers(1, &IBOID[1]);
        glDeleteBuffers(1, &VBOID[0]);
        glDeleteBuffers(1, &IBOID[1]);
        glDeleteVertexArrays(1, &VAOID[0]);
        glDeleteVertexArrays(1, &VAOID[1]);

        glDetachShader(ShaderProgram[0], VertexShader[0]);
        glDetachShader(ShaderProgram[0], FragmentShader[0]);
        glDeleteShader(VertexShader[0]);
        glDeleteShader(FragmentShader[0]);
        glDeleteProgram(ShaderProgram[0]);

        glDetachShader(ShaderProgram[1], VertexShader[1]);
        glDetachShader(ShaderProgram[1], FragmentShader[1]);
        glDeleteShader(VertexShader[1]);
        glDeleteShader(FragmentShader[1]);
        glDeleteProgram(ShaderProgram[1]);
    }

    bool shouldShow = true;
    DLL_EXPORT void renderFunction() {
        Debug::Log("Rendering", Color::Red);
        float projectionModelviewMatrix[16];
        //Just set it to identity matrix
        memset(projectionModelviewMatrix, 0, sizeof(float) * 16);
        projectionModelviewMatrix[0] = 1.0;
        projectionModelviewMatrix[5] = 1.0;
        projectionModelviewMatrix[10] = 1.0;
        projectionModelviewMatrix[15] = 1.0;

        //Clear all the buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        //Bind the shader that we want to use
        glUseProgram(ShaderProgram[0]);
        //Setup all uniforms for your shader
        glUniformMatrix4fv(ProjectionModelviewMatrix_Loc[0], 1, FALSE, projectionModelviewMatrix);
        //Bind the VAO
        glBindVertexArray(VAOID[0]);
        //At this point, we would bind textures but we aren't using textures in this example

        //Draw command
        //The first to last vertex is 0 to 3
        //6 indices will be used to render the 2 triangles. This make our quad.
        //The last parameter is the start address in the IBO => zero
        if (shouldShow) {
            shouldShow = false;
            glDrawRangeElements(GL_TRIANGLES, 0, 3, 6, GL_UNSIGNED_SHORT, NULL);
        }
        else {
            shouldShow = true;
        }

        //Bind the shader that we want to use
//        glUseProgram(ShaderProgram[1]);
        //Setup all uniforms for your shader
//        glUniformMatrix4fv(ProjectionModelviewMatrix_Loc[1], 1, FALSE, projectionModelviewMatrix);
        //Bind the VAO
//        glBindVertexArray(VAOID[1]);
        //At this point, we would bind textures but we aren't using textures in this example

        //Draw command
        //The first to last vertex is 0 to 3
        //3 indices will be used to render 1 triangle.
        //The last parameter is the start address in the IBO => zero
//        glDrawRangeElements(GL_TRIANGLES, 0, 3, 3, GL_UNSIGNED_SHORT, NULL);

        glutSwapBuffers();
    }
    int WindowID = 0;
    std::thread* t1;
    int x, y, width, height = 0;
    void DoFunction() {
        char* myargv[1];
        int myargc = 1;
        int i;
        int NumberOfExtensions;
        int OpenGLVersion[2];
        myargv[0] = strdup("libProjectCupboard");
        glutInit(&myargc, myargv);
        glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_BORDERLESS | GLUT_CAPTIONLESS);
        glutInitWindowSize(width, height);
        glutInitWindowPosition(x, y);
        WindowID = glutCreateWindow("Window 1");
        // Display Callback Function 
        glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
        glewExperimental = TRUE;
        GLenum err = glewInit();
        if (err != GLEW_OK)
        {
            Debug::Log("glewInit failed, aborting.", Color::Red);
            //Problem: glewInit failed, something is seriously wrong.
            exit(1);
        }
        glGetIntegerv(GL_MAJOR_VERSION, &OpenGLVersion[0]);
        glGetIntegerv(GL_MINOR_VERSION, &OpenGLVersion[1]);

        glGetIntegerv(GL_NUM_EXTENSIONS, &NumberOfExtensions);

        //We don't need any extensions. Useless code.
        for (i = 0; i < NumberOfExtensions; i++)
        {
            const GLubyte* ccc = glGetStringi(GL_EXTENSIONS, i);
        }
        if (LoadShader("Shader1.vert", "Shader1.frag", false, false, true, ShaderProgram[0], VertexShader[0], FragmentShader[0]) == -1)
        {
            exit(1);
        }
        else
        {
            ProjectionModelviewMatrix_Loc[0] = glGetUniformLocation(ShaderProgram[0], "ProjectionModelviewMatrix");
        }
        if (LoadShader("Shader2.vert", "Shader2.frag", true, true, false, ShaderProgram[1], VertexShader[1], FragmentShader[1]) == -1)
        {
            exit(1);
        }
        else
        {
            ProjectionModelviewMatrix_Loc[1] = glGetUniformLocation(ShaderProgram[1], "ProjectionModelviewMatrix");
        }

        CreateGeometry();
        glutDisplayFunc(&renderFunction);
        timer(0);
        glutMainLoop();
        ExitFunction();
    }
    DLL_EXPORT void stop() {
        glutLeaveMainLoop();

    }
    DLL_EXPORT void initialize(int xPos, int yPos, int w, int h) {
        x = xPos;
        y = yPos;
        width = w;
        height = h;
        isLooping = true;
        t1 = new std::thread(DoFunction);
    }
    typedef void(*FuncCallBack)(const char* message, int color, int size);
    static FuncCallBack callbackInstance = nullptr;
    DLL_EXPORT void RegisterDebugCallback(FuncCallBack cb);
}
void Debug::Log(const char* message, Color color) {
    if (callbackInstance != nullptr)
        callbackInstance(message, (int)color, (int)strlen(message));
}

void  Debug::Log(const std::string message, Color color) {
    const char* tmsg = message.c_str();
    if (callbackInstance != nullptr)
        callbackInstance(tmsg, (int)color, (int)strlen(tmsg));
}

void  Debug::Log(const int message, Color color) {
    std::stringstream ss;
    ss << message;
    send_log(ss, color);
}

void  Debug::Log(const char message, Color color) {
    std::stringstream ss;
    ss << message;
    send_log(ss, color);
}

void  Debug::Log(const float message, Color color) {
    std::stringstream ss;
    ss << message;
    send_log(ss, color);
}

void  Debug::Log(const double message, Color color) {
    std::stringstream ss;
    ss << message;
    send_log(ss, color);
}

void Debug::Log(const bool message, Color color) {
    std::stringstream ss;
    if (message)
        ss << "true";
    else
        ss << "false";

    send_log(ss, color);
}

void Debug::send_log(const std::stringstream& ss, const Color& color) {
    const std::string tmp = ss.str();
    const char* tmsg = tmp.c_str();
    if (callbackInstance != nullptr)
        callbackInstance(tmsg, (int)color, (int)strlen(tmsg));
}

//Create a callback delegate
void RegisterDebugCallback(FuncCallBack cb) {
    callbackInstance = cb;
}