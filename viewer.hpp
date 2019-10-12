#ifndef VIEWER_HPP
#define VIEWER_HPP

//GLFW is used to do the video output for the viewer window
#include <GLFW/glfw3.h>

//Intel's RealSense library is used to process video data into OpenGL textures
#include <librealsense2/rs.hpp>

//cmath is needed for cos and sin functions for drawing circles
#include <cmath>

//map is needed because the show function is given the frames as a map of frameIDs to frame data
#include <map>

//viewer class, manages preview window using GLFW and OpenGL
class viewer
{
    public:
        //constructor creates and activates a GLFW window
        viewer();
        //destructor closes GLFW window
        ~viewer();
        //close hooks into the close button to set shouldClose flag to 1, so bool operator can return correct value to main program window
        void close();
        //bool returns true to main program while window is open, enabling the "while(rscViewer)" loop to operate
        operator bool();
        //show takes a map of frames and displays them in a mosaic
        void show(const std::map<int, rs2::frame> rscFrames);

    private:
        //the GLFW window handle for this window
        GLFWwindow *previewWindow;
        //various GLFW functions require pointers to frame size ints, so those are declared here
        int windowWidth;
        int windowHeight;

        //showMotionFrame and showVideoFrame each require a texture map ID
        //creating them here rather than a new one each time one of the functions is called
        //reduces calls to "glGenTextures" and helps rendering performance
        GLuint motionFrameHandle;
        GLuint videoFrameHandle;

        //showMotionFrame outputs motionFrame data as a line in a set of axes
        void showMotionFrame(const rs2::motion_frame &rscFrame, const int xLoc, const int yLoc);
        //drawMotionFrameAxes handles drawing the x, y, and z axes for the display
        static void drawMotionFrameAxes();
        //drawMotionFrameCircles handles drawing a set of circles for the x, y, and z axes
        static void drawMotionFrameCircle(float xx, float xy, float xz, float yx, float yy, float yz);

        //showVideoFrame outputs videoFrame data from either a depth stream or an rgb stream
        void showVideoFrame(const rs2::video_frame &rscFrame, const int xLoc, const int yLoc);
};

#endif
