/*
 * This example creates an SDL window and renderer, and then draws a 3D cube
 * using the Object and ReferenceFrame classes.
 * W/S keys control rotation speed.
 *
 * This code is public domain. Feel free to use it for any purpose!
 */

#define SDL_MAIN_USE_CALLBACKS 1 /* use the callbacks instead of main() */
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <memory>
#include "msd-sim/src/Environment/Object.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Utils/GeometryFactory.hpp"

/* We will use this renderer to draw into this window every frame. */
static SDL_Window* window = NULL;
static SDL_Renderer* renderer = NULL;

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

/* The 3D cube object */
static std::unique_ptr<msd_sim::Object> cube;

/* Viewer reference frame (camera) */
static msd_sim::ReferenceFrame viewerFrame;

/* Rotation parameters */
static float rotation_speed = 0.5f; /* radians per second */
static const float ROTATION_SPEED_CHANGE = 0.2f;
static const float MIN_ROTATION_SPEED = 0.0f;
static const float MAX_ROTATION_SPEED = 3.0f;

/* Track which keys are currently pressed */
static bool w_pressed = false;
static bool s_pressed = false;
static Uint64 last_time = 0;

/* This function runs once at startup. */
SDL_AppResult SDL_AppInit(void** appstate, int argc, char* argv[])
{
  SDL_SetAppMetadata("3D Rotating Cube", "1.0", "com.example.3d-cube");

  if (!SDL_Init(SDL_INIT_VIDEO))
  {
    SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  if (!SDL_CreateWindowAndRenderer("3D Rotating Cube (W/S to change speed)",
                                   WINDOW_WIDTH,
                                   WINDOW_HEIGHT,
                                   SDL_WINDOW_RESIZABLE,
                                   &window,
                                   &renderer))
  {
    SDL_Log("Couldn't create window/renderer: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }
  SDL_SetRenderLogicalPresentation(
    renderer, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_LOGICAL_PRESENTATION_LETTERBOX);

  /* Create the cube geometry (100 unit cube) */
  auto cubeVertices = msd_sim::GeometryFactory::createCube(100.0);

  /* Create the cube object at the global origin */
  msd_sim::ReferenceFrame cubeFrame{msd_sim::Coordinate{0.0, 0.0, 0.0}};
  cube = std::make_unique<msd_sim::Object>(cubeVertices, cubeFrame);

  /* Set projection parameters */
  cube->setProjectionParameters(500.0f, WINDOW_WIDTH, WINDOW_HEIGHT);

  /* Set cube color (cyan) */
  cube->setColor(100, 200, 255, 255);

  /* Position the viewer/camera back from the origin */
  viewerFrame.setOrigin(msd_sim::Coordinate{-400.0, 0.0, 0.});

  last_time = SDL_GetTicks();

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

/* This function runs when a new event (mouse input, keypresses, etc) occurs. */
SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event)
{
  if (event->type == SDL_EVENT_QUIT)
  {
    return SDL_APP_SUCCESS; /* end the program, reporting success to the OS. */
  }
  else if (event->type == SDL_EVENT_KEY_DOWN)
  {
    if (event->key.key == SDLK_W)
    {
      w_pressed = true;
    }
    else if (event->key.key == SDLK_S)
    {
      s_pressed = true;
    }
  }
  else if (event->type == SDL_EVENT_KEY_UP)
  {
    if (event->key.key == SDLK_W)
    {
      w_pressed = false;
    }
    else if (event->key.key == SDLK_S)
    {
      s_pressed = false;
    }
  }
  return SDL_APP_CONTINUE; /* carry on with the program! */
}

/* This function runs once per frame, and is the heart of the program. */
SDL_AppResult SDL_AppIterate(void* appstate)
{
  const Uint64 now = SDL_GetTicks();
  const float elapsed =
    ((float)(now - last_time)) / 1000.0f; /* seconds since last iteration */

  /* Update rotation speed based on which keys are pressed */
  if (w_pressed)
  {
    rotation_speed += ROTATION_SPEED_CHANGE * elapsed;
    if (rotation_speed > MAX_ROTATION_SPEED)
    {
      rotation_speed = MAX_ROTATION_SPEED;
    }
  }
  if (s_pressed)
  {
    rotation_speed -= ROTATION_SPEED_CHANGE * elapsed;
    if (rotation_speed < MIN_ROTATION_SPEED)
    {
      rotation_speed = MIN_ROTATION_SPEED;
    }
  }

  /* Update the cube's rotation */
  auto& cubeEuler = cube->getReferenceFrame().getEulerAngles();
  cubeEuler.yaw.setRad(cubeEuler.yaw.getRad() + rotation_speed * elapsed);
  cubeEuler.pitch.setRad(cubeEuler.pitch.getRad() +
                         rotation_speed * 0.7f * elapsed);
  cubeEuler.roll.setRad(cubeEuler.roll.getRad() +
                        rotation_speed * 0.5f * elapsed);

  /* Update the cube's reference frame rotation matrix */
  cube->getReferenceFrame().setRotation(cubeEuler);

  /* Transform cube vertices to viewer frame */
  cube->transformToViewerFrame(viewerFrame);

  last_time = now;

  /* Clear the screen with black */
  SDL_SetRenderDrawColor(
    renderer, 0, 0, 0, SDL_ALPHA_OPAQUE); /* black, full alpha */
  SDL_RenderClear(renderer);              /* start with a blank canvas. */

  /*
   * Render the 3D cube:
   * The cube has been triangulated into 36 vertices (12 triangles).
   */
  // cube->render(renderer);               // Render filled triangles
  cube->renderWireframe(renderer);  // Render wireframe edges in white

  SDL_RenderPresent(renderer); /* put it all on the screen! */

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

/* This function runs once at shutdown. */
void SDL_AppQuit(void* appstate, SDL_AppResult result)
{
  /* SDL will clean up the window/renderer for us. */
}
