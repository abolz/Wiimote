// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include <cassert>
#include <chrono>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <set>
#include <sstream>
#include <string>

#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>

#include "Wiimote/Wiimote.h"

#include "lib/Math/math.h"
#include "lib/Math/util/timer.h"

#include "Track.h"

#ifdef _MSC_VER
#include <thread>
#include <mutex>
using std::thread;
using std::mutex;
using std::lock_guard;
#else
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
using boost::thread;
using boost::mutex;
using boost::lock_guard;
#endif

using namespace math;

#define SHOW_GRAPH 0

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

static const float kBackColor[] = { 0.1f, 0.2f, 0.7f, 1.0f };
static const float kGridColor[] = { 0.2f, 0.4f, 0.6f, 1.0f };
static const float kFov = math::to_radians(45.0f);
static const float kZNear = 0.1f;
static const float kZFar = 1000.0f;

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

static sf::RenderWindow     window;

static thread               wiimoteThread;
static bool                 wiimoteThreadRunning = false;
static bool                 wiimoteThreadShutdown = false;

static Track track;

static LowPassFilter accBias;
static LowPassFilter gyroBias;

#if SHOW_GRAPH

// Show the last N raw values
// FIXME: TIME!!!
struct Graph
{
    // Raw input from the last 10 seconds
    std::deque<float> inputs;
    // Range of input values
    vec2i range;
    // Mutex to protect the queue
    mutex lock;

    void add(float x)
    {
        lock_guard<mutex> guard(lock);

        inputs.push_back(x);

        if (inputs.size() > 500)
            inputs.pop_front();
    }

    void render(vec3 const& color = vec3(1.0f, 0.0f, 0.0f))
    {
        lock_guard<mutex> guard(lock);

        int N = (int)inputs.size();

        if (N == 0)
            return;

        //int w = window.getSize().x;
        //int h = window.getSize().y;

        float x = 0.0f;
        float dx = 1.0f / 500.0f;
        float sh = 1.0f / 1.0f;

        //mat4 P = math::mat4::ortho(0.0f, (float)w, (float)h, 0.0f, -1.0f, 1.0f);
        mat4 P = math::mat4::ortho(0.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f);

        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(P.data());
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glColor3f(color.x, color.y, color.z);

        glBegin(GL_LINE_STRIP);
        for (float y : inputs)
        {
            glVertex2f(x, y * sh);
            x += dx;
        }
        glEnd();
    }
};

static Graph graphs[3];

#endif

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

static unsigned AlphaDelta227529()
{
    static unsigned leds[] = {
        0x10,
        0x10, 0x30, 0x70, 0xF0, 0xE0, 0xC0, 0x80,
        0x80,
        0x80, 0xC0, 0xE0, 0xF0, 0x70, 0x30, 0x10,
    };
    static unsigned counter = 0;

    unsigned r = leds[counter];

    counter = (counter + 1) % (sizeof(leds) / sizeof(leds[0]));

    return r;
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

static void WiimoteThreadProc();

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
static bool Init()
{
    glEnable(GL_DEPTH_TEST);

    glClearColor(0.3f, 0.5f, 0.7f, 1.0f);

    wiimoteThreadRunning = true;
    wiimoteThread = thread(WiimoteThreadProc);

    return true;
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
static void Finish()
{
    wiimoteThreadRunning = false;
    wiimoteThread.join();
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
static void WiimoteThreadProc()
{
    math::FrameCounter fps;
    math::Timer timer;
    math::Timesteps AD(1.0/10.0);
    math::Timesteps MP(1.0f/50.0f);

    bool calibrating = true;

    using namespace wii;

    LowPassFilter accelFilter(0.1f);

    Wiimote wiimote;

    if (!wiimote.Connect())
    {
        printf("Could not connect to Wiimote\n");
        return;
    }

    wiimote.SetReportMode(Wiimote::ReportMode::ButtonsAccelExt, false);
    //wiimote.SetLEDs(State::LED1);

    State prev = wiimote.GetState();

    while (wiimote.Poll())
    {
        fps.update();

        ////printf("FPS: %f\n", fps.getFPS());

        State state = wiimote.GetState();

        if (calibrating)
        {
            if (state.data & State::Accel)
            {
                auto dt = state.time - prev.time;
                auto a = vec3::from_coords(state.accel.normalized);

                track.calibrateAccel(a, (float)dt);
            }

            if (state.data & State::MotionPlus)
            {
                auto dt = state.extension.motionPlus.time - prev.extension.motionPlus.time;
                auto w = vec3::from_coords(state.extension.motionPlus.raw);

                track.calibrateGyro(w, (float)dt);
            }

            if (timer.elapsed() > 5.0)
            {
                track.init();
                calibrating = false;

                MP.restart();

                wiimote.SetLEDs(1,1,1,1);
            }
            else
            {
                AD.update();

                while (AD.consume())
                {
                    wiimote.SetLEDs(AlphaDelta227529());
                }
            }
        }
        else
        {
            if (state.data & State::Buttons)
            {
                if (state.buttonsPressed & State::Home)
                {
                    track.home();
                }

                if (state.buttonsPressed & State::A)
                {
                    wiimote.SetRumble(!state.rumble);
                }
            }

            if (state.data & State::Accel)
            {
                double dt = state.time - prev.time;

                auto a = vec3::from_coords(state.accel.normalized);

                track.handleAccel(a, (float)dt);
            }

            if (state.data & State::MotionPlus)
            {
#if 0
                auto w = vec3::copy(state.extension.motionPlus.raw);
                auto fast = bvec3::copy(state.extension.motionPlus.fast);

                MP.update();

                while (MP.consume())
                {
                    track.handleGyros(w, fast, MP.delta());
                }
#else
                double dt = state.extension.motionPlus.time - prev.extension.motionPlus.time;

                auto w = vec3::from_coords(state.extension.motionPlus.raw);
                auto fast = vec3b::from_coords(state.extension.motionPlus.fast);

                track.handleGyros(w, fast, (float)dt);

#if SHOW_GRAPH
#if 1
                //graphs[0].add(track.state().omega.x / 1000.0f);
                //graphs[1].add(track.state().omega.y / 1000.0f);
                graphs[0].add(state.extension.motionPlus.raw.z / 16384.0f - 0.5f);
                graphs[1].add(state.extension.motionPlus.raw.y / 16384.0f - 0.5f);
                graphs[2].add(track.state().omega.y / 12.5f);
#else
                auto const& r = state.extension.motionPlus.raw;

                graphs[0].add(r.x / 16384.0f);
                graphs[1].add(r.y / 16384.0f);
                graphs[2].add(r.z / 16384.0f);
#endif
#endif // SHOW_GRAPH
#endif

//                wiimote.SetLEDs(fast.x, fast.y, fast.z, 0);
            }

            if (wiimoteThreadRunning == false)
            {
                if (!wiimoteThreadShutdown)
                {
                    wiimoteThreadShutdown = true;
                    wiimote.Shutdown();
                }
            }
        }

        prev = state;
    }

    wiimote.Disconnect();
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
//static void RenderVector(vec3 const& v, float size = 2.0f)
//{
////  vec3 u = normalize(v);
//    vec3 u = v;
//
//    glColor3f(1.0f, 1.0f, 1.0f);
//
//    glBegin(GL_LINES);
//    glVertex3f(0,0,0); glVertex3f(u.x * size, u.y * size, u.z * size);
//    glEnd();
//}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
static void RenderCoords(float x = 0.0f, float y = 0.0f, float z = 0.0f, float size = 1.0f)
{
    glBegin(GL_LINES);
    glColor3f(1, 0, 0); glVertex3f(x, y, z); glVertex3f(x + size, y, z);
    glColor3f(0, 1, 0); glVertex3f(x, y, z); glVertex3f(x, y + size, z);
    glColor3f(0, 0, 1); glVertex3f(x, y, z); glVertex3f(x, y, z + size);
    glEnd();
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
static void RenderGrid(float y = 0.0f, float size = 20.0f, bool coords = true)
{
    glBegin(GL_LINES);
    glNormal3f(0.0f, 1.0f, 0.0f);
    glColor3fv(kGridColor);
    if (coords)
    {
        glColor3fv(kGridColor); glVertex3f(0, y, 0); glVertex3f(-size, y, 0);
        glColor3fv(kGridColor); glVertex3f(0, y, 0); glVertex3f(0, y, -size);

        glColor3f(1, 0, 0); glVertex3f(0, y, 0); glColor3fv(kGridColor); glVertex3f(size, y, 0);
        glColor3f(0, 1, 0); glVertex3f(0, y, 0); glColor3fv(kBackColor); glVertex3f(0, size + y, 0);
        glColor3f(0, 0, 1); glVertex3f(0, y, 0); glColor3fv(kGridColor); glVertex3f(0, y, size);
    }
    for (int i = -(int)size; i <= (int)size; ++i)
    {
        if (coords && i == 0)
            continue;

        auto f = static_cast<GLfloat>(i);

        glVertex3f(    f, y, -size);
        glVertex3f(    f, y,  size);
        glVertex3f(-size, y,     f);
        glVertex3f( size, y,     f);
    }
    glEnd();
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
static void RenderWiimote()
{
    static float const sx = 0.5f;
    static float const sy = 0.5f;
    static float const sz = 3.0f;

    static float const vertices[8][3] = {
        { -1.0f*sx, -1.0f*sy, -1.0f*sz }, //0 000
        { -1.0f*sx, -1.0f*sy,  1.0f*sz }, //1 001
        { -1.0f*sx,  1.0f*sy, -1.0f*sz }, //2 010
        { -1.0f*sx,  1.0f*sy,  1.0f*sz }, //3 011
        {  1.0f*sx, -1.0f*sy, -1.0f*sz }, //4 100
        {  1.0f*sx, -1.0f*sy,  1.0f*sz }, //5 101
        {  1.0f*sx,  1.0f*sy, -1.0f*sz }, //6 110
        {  1.0f*sx,  1.0f*sy,  1.0f*sz }, //7 111
    };

    static int const faces[6][4] = {
        { 0, 4, 5, 1, }, // bottom
        { 2, 3, 7, 6, }, // top
        { 5, 4, 6, 7, }, // right
        { 0, 1, 3, 2, }, // left
        { 1, 5, 7, 3, }, // front
        { 0, 2, 6, 4, }, // back
    };

    static float const colors[6][3] = {
        { 0.6f, 0.6f, 0.0f },
        { 0.6f, 0.6f, 0.0f },
        { 0.5f, 0.5f, 0.0f },
        { 0.5f, 0.5f, 0.0f },
        { 0.4f, 0.4f, 0.0f },
        { 0.4f, 0.4f, 0.0f },
    };

    // Draw wiimote
    for (int i = 0; i < 6; i++)
    {
        glBegin(GL_QUADS);
        glColor3fv(colors[i]);
        glVertex3fv(vertices[faces[i][0]]);
        glVertex3fv(vertices[faces[i][1]]);
        glVertex3fv(vertices[faces[i][2]]);
        glVertex3fv(vertices[faces[i][3]]);
        glEnd();
    }

    // Draw pointer
    glBegin(GL_LINES);
    glColor3f(1.0f, 1.0f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, -12.0f);
    glEnd();
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
static void Render()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    int w = window.getSize().x;
    int h = window.getSize().y;

    auto P = mat4::perspective(kFov, static_cast<float>(w) / static_cast<float>(h), kZNear, kZFar);
    auto V = mat4::lookAt(vec3(0,5,10), vec3(0,0,0), vec3(0,1,0));

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(P.data());
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(V.data());

    RenderGrid();

#if 0
    auto T = mat4::rotation(quat(track.state().rotAccel));
#else
    auto T = mat4::rotation(track.state().rotGyro);
#endif

    glMatrixMode(GL_MODELVIEW);
    glMultMatrixf(T.data());

    RenderWiimote();
    RenderCoords(0.0f, 0.0f, 0.0f, 4.0f);
    RenderGrid();

#if SHOW_GRAPH
    graphs[0].render({1,0,0});
    graphs[1].render({0,1,0});
    graphs[2].render({0,0,1});
#endif
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
static void OnKeyPressed(sf::Event::KeyEvent const& e)
{
    switch (e.code)
    {
    case sf::Keyboard::Num1:
        break;
    case sf::Keyboard::Num2:
        break;
    case sf::Keyboard::Num3:
        break;
    case sf::Keyboard::H:
        break;
    case sf::Keyboard::V:
        break;
    default:
        break;
    }
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
static void OnMouseWheelMoved(sf::Event::MouseWheelEvent const& /*e*/)
{
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
static void OnMouseButtonDown(sf::Event::MouseButtonEvent const& /*e*/)
{
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
static void OnMouseButtonUp(sf::Event::MouseButtonEvent const& /*e*/)
{
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
static void OnMouseMove(sf::Event::MouseMoveEvent const& /*e*/)
{
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
static void OnResize(sf::Event::SizeEvent const& e)
{
    int w = static_cast<int>(e.width);
    int h = static_cast<int>(e.height);

    glViewport(0, 0, w, h);
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
static bool ProcessEvent(sf::Event const& e)
{
    switch (e.type)
    {
    case sf::Event::Closed:
        return false;
    case sf::Event::Resized:
        OnResize(e.size);
        break;
    case sf::Event::KeyPressed:
        OnKeyPressed(e.key);
        break;
    case sf::Event::KeyReleased:
        break;
    case sf::Event::MouseWheelMoved:
        OnMouseWheelMoved(e.mouseWheel);
        break;
    case sf::Event::MouseButtonPressed:
        OnMouseButtonDown(e.mouseButton);
        break;
    case sf::Event::MouseButtonReleased:
        OnMouseButtonUp(e.mouseButton);
        break;
    case sf::Event::MouseMoved:
        OnMouseMove(e.mouseMove);
        break;
    case sf::Event::MouseEntered:
        break;
    case sf::Event::MouseLeft:
        break;
    default:
        return true;
    }

    return true;
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------
int main()
{
    sf::ContextSettings settings;

    settings.majorVersion       = 3;
    settings.minorVersion       = 3;
    settings.depthBits          = 24;
    settings.stencilBits        = 8;
    settings.antialiasingLevel  = 8;

    // Create the window
    window.create(sf::VideoMode(1000, 600), "OpenGL", sf::Style::Default, settings);

    // Enable/Disable vsync
    window.setVerticalSyncEnabled(true);

    // Load resources, initialize the OpenGL states, ...
    if (!Init())
    {
        std::cout << "Failed to initialize.\n";
        return -1;
    }

    //bool stillOpen = true;

    // Run the main loop
    while (window.isOpen())
    {
        sf::Event e;

        // Process events
        while (window.pollEvent(e))
        {
            if (e.type == sf::Event::Closed)
            {
                window.close();
                break;
            }

            ProcessEvent(e);
        }

        // Closed?
        if (!window.isOpen())
            break;

        // Render the scene
        Render();

        // End the current frame (internally swaps the front and back buffers)
        window.display();
    }

    // Release resources...
    Finish();

    return 0;
}
