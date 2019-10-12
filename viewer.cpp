#include "viewer.hpp"

//public: constructor
//initializes private class variables
//creates empty GLFW window
viewer::viewer()
{
    //TODO?: change from hard coded height and width
    windowHeight = 1280;
    windowWidth = 720;
    //frame handles are used by the show frame functions
    motionFrameHandle = 0;
    videoFrameHandle = 0;

    //Create a GLFW window and set it as the current context
    glfwInit();
    previewWindow = glfwCreateWindow(1280, 960, "Camera Output Test", nullptr, nullptr);
    if(!previewWindow)
    {
        throw std::runtime_error("Could not open OpenGL window");
    }
    glfwMakeContextCurrent(previewWindow);
}

//public: destructor
//terminates GLFW window
viewer::~viewer()
{
    glfwDestroyWindow(previewWindow);
    glfwTerminate();
}

//public: close
//intercepts default destructor functionality to set should close flag
//this flag is used by the bool function to let the calling program know when to stop sending this class frame data
void viewer::close()
{
    glfwSetWindowShouldClose(previewWindow, 1);
}

//public: bool
//allows the viewer class to return a proper bool value when tested
//used to allow "while(rscViewer)" in main program
viewer::operator bool()
{
    return !glfwWindowShouldClose(previewWindow);
}

//public: show
//renders a set of frames in the preview window
void viewer::show(const std::map<int, rs2::frame> rscFrames)
{
    //check to ensure the frame map has frames
    if(rscFrames.size())
    {
        //streamNum is used to do math to determine where each frame displays
        //    streamNum    | frameLocationX | frameLocationY
        //  streamNum%4==0 |       0        |       0
        //  streamNum%4==1 |      640       |       0
        //  streamNum%4==2 |       0        |      480
        //  streamNum%4==3 |      640       |      480
        int streamNum = 0;

        //run through all the frames in the map
        for(auto& rscFrame : rscFrames)
        {
            //if it's a video frame, render it using the showVideoFrame function
            if(auto rscVideoFrame = rscFrame.second.as<rs2::video_frame>())
            {
                showVideoFrame(rscVideoFrame, 640*(streamNum%2), 480*(streamNum/2));
            }
            //if it's a motion frame, render it using the showMotionFrame function
            if(auto rscMotionFrame = rscFrame.second.as<rs2::motion_frame>())
            {
                showMotionFrame(rscMotionFrame, 640*(streamNum%2), 480*(streamNum/2));
            }

            streamNum++;
        }

        //empty previous mosaic
        glPopMatrix();
        glfwSwapBuffers(previewWindow);

        //wait on events and calculate buffer size
        glfwPollEvents();
        glfwGetFramebufferSize(previewWindow, &windowWidth, &windowHeight);

        //clear the framebuffer
        glClear(GL_COLOR_BUFFER_BIT);
        glViewport(0, 0, windowWidth, windowHeight);

        //draw the frames
        glPushMatrix();
        glfwGetWindowSize(previewWindow, &windowWidth, &windowHeight);
        glOrtho(0, windowWidth, windowHeight, 0, -1, +1);

    }
}

//private: showMotionFrame
//handles display of motion frame data, either accelerometer or gyroscope
void viewer::showMotionFrame(const rs2::motion_frame &rscFrame, const int xLoc, const int yLoc)
{
    //if there's somehow no frame, return before everything breaks
    if(!rscFrame)
    {
        return;
    }

    //if this is the first time calling this function, generate a texture set, otherwise reuse the existing one
    if(!motionFrameHandle)
    {
        glGenTextures(1, &motionFrameHandle);
    }

    //perform OpenGL matrix math
    //TODO: better comments here - Jordan
    glViewport(xLoc, yLoc, 640, 480);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glOrtho(0, 640, 480, 0, -1, +1);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-2.8, 2.8, -2.4, 2.4, -7, 7);
    glRotatef(25, 1, 0, 0);
    glTranslatef(0, -0.33, -1);
    glRotatef(-135, 0, 1, 0);
    glRotatef(180, 0, 0, 1);
    glRotatef(-90, 0, 1, 0);

    //draw the RGB axis arrows
    drawMotionFrameAxes();

    //draw the axis circles
    //perpendicular to x axis
    drawMotionFrameCircle(0, 1, 0, 0, 0, 1);
    //perpendicular to y axis
    drawMotionFrameCircle(1, 0, 0, 0, 0, 1);
    //perpendicular to z axis
    drawMotionFrameCircle(1, 0, 0, 0, 1, 0);

    //grab the x, y, and z components of the motion data
    float rscMotionX = rscFrame.get_motion_data().x;
    float rscMotionY = rscFrame.get_motion_data().y;
    float rscMotionZ = rscFrame.get_motion_data().z;

    //rscMotionNormal is a measure of the absolute magnitude of motion data being sent
    float rscMotionNormal = std::sqrt((rscMotionX * rscMotionX) + (rscMotionY * rscMotionY) + (rscMotionZ * rscMotionZ));

    //low values of rscMotionNormal are ignored to reduce jittery output from small movements
    if(rscMotionNormal > 0.1)
    {
        //setup and draw a line from 0, 0, 0 to a point based on the relative magnitude of our x, y, and z values
        glLineWidth(3);
        glColor3f(1, 1, 1);
        glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(rscMotionX / rscMotionNormal, rscMotionY / rscMotionNormal, rscMotionZ / rscMotionNormal);
        glEnd();
    }

    //pop matrix to clear stack
    glPopMatrix();
}

//private: drawMotionFrameAxes
//draws a red, green, and blue set of arrows along the x, y, and z axes
void viewer::drawMotionFrameAxes()
{
    //set line width
    glLineWidth(4);

    //draw red X axis
    glColor3f(1, 0, 0);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 0, 0);
    glEnd();
    glBegin(GL_TRIANGLES);
        glVertex3f(1.1, 0, 0);
        glVertex3f(1, -0.05, 0);
        glVertex3f(1, 0.05, 0);
        glVertex3f(1.1, 0, 0);
        glVertex3f(1, 0, -0.05);
        glVertex3f(1, 0, 0.05);
    glEnd();

    //draw green Y axis
    glColor3f(0, 1, 0);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 1, 0);
    glEnd();
    glBegin(GL_TRIANGLES);
        glVertex3f(0, 1.1, 0);
        glVertex3f(0, 1, 0.05);
        glVertex3f(0, 1, -0.05);
        glVertex3f(0, 1.1, 0);
        glVertex3f(0.05, 1, 0);
        glVertex3f(-0.05, 1, 0);
    glEnd();

    //draw blue Z axis
    glColor3f(0, 0, 1);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 1);
    glEnd();
    glBegin(GL_TRIANGLES);
        glVertex3f(0, 0, 1.1);
        glVertex3f(0, 0.05, 1);
        glVertex3f(0, -0.05, 1);
        glVertex3f(0, 0, 1.1);
        glVertex3f(0.05, 0, 1);
        glVertex3f(-0.05, 0, 1);
    glEnd();
}

//private: drawMotionFrameCircle
//draws a circle, skewed and stretched to appear 3d
void viewer::drawMotionFrameCircle(float xx, float xy, float xz, float yx, float yy, float yz)
{
    glColor3f(0.5, 0.5, 0.5);
    glLineWidth(2);

    glBegin(GL_LINE_STRIP);
        //theta is the angle(in radians) per line segment on the outside of the circle
        float theta = 0.125663;
        //Loop 50 times, one for each line segment
        for(int i = 0; i <= 50; i++)
        {
            //the vertex is based on the skew of the circle and how far along in drawing it we are
            glVertex3f(1.1 * (xx * cos(theta*i) + yx * sin(theta*i)),
                       1.1 * (xy * cos(theta*i) + yy * sin(theta*i)),
                       1.1 * (xz * cos(theta*i) + yz * sin(theta*i)));
        }
    glEnd();
}

//private: showVideoFrame
//handles display of video frame data, either depth or rgb
void viewer::showVideoFrame(const rs2::video_frame &rscFrame, const int xLoc, const int yLoc)
{
    //if there's somehow no frame, return before everything breaks
    if(!rscFrame)
    {
        return;
    }

    //if this is the first time calling this function, generate a texture set, otherwise reuse the existing one
    if (!videoFrameHandle)
    {
        glGenTextures(1, &videoFrameHandle);
    }
    glBindTexture(GL_TEXTURE_2D, videoFrameHandle);

    //do the OpenGL voodoo
    //TODO: Better comments here - Jordan
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rscFrame.get_width(), rscFrame.get_height(), 0, GL_RGB, GL_UNSIGNED_BYTE, rscFrame.get_data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glViewport(xLoc, yLoc, 640, 480);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glOrtho(0, 640, 480, 0, -1, +1);

    glBindTexture(GL_TEXTURE_2D, videoFrameHandle);
    glColor4f(1, 1, 1, 1);

    glEnable(GL_TEXTURE_2D);
        glBegin(GL_QUADS);
            glTexCoord2f(0, 0); glVertex2f(0, 0);
            glTexCoord2f(0, 1); glVertex2f(0, 480);
            glTexCoord2f(1, 1); glVertex2f(640, 480);
            glTexCoord2f(1, 0); glVertex2f(640, 0);
        glEnd();
    glDisable(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, 0);
}
