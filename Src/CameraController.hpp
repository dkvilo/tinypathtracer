#pragma once

#include <SDL2/SDL.h>

#include "Camera.hpp"

const float MOVE_SPEED        = 0.1f;
const float MOUSE_SENSITIVITY = 0.1f;

static inline void cameraControllerRotate( Camera &camera, float deltaYaw, float deltaPitch )
{
	camera.rotate( deltaYaw, deltaPitch );
}

static inline void cameraControllerMove( Camera &camera )
{
	const Uint8 *keystate = SDL_GetKeyboardState( nullptr );
	Vec3         moveDir( 0, 0, 0 );

	if ( keystate[SDL_SCANCODE_W] )
	{
		moveDir = moveDir + camera.forward;
	}

	if ( keystate[SDL_SCANCODE_S] )
	{
		moveDir = moveDir - camera.forward;
	}

	if ( keystate[SDL_SCANCODE_A] )
	{
		moveDir = moveDir - camera.right;
	}

	if ( keystate[SDL_SCANCODE_D] )
	{
		moveDir = moveDir + camera.right;
	}

	if ( keystate[SDL_SCANCODE_SPACE] )
	{
		moveDir = moveDir + Vec3( 0, 1, 0 );
	}

	if ( keystate[SDL_SCANCODE_LCTRL] )
	{
		moveDir = moveDir - Vec3( 0, 1, 0 );
	}

	if ( moveDir.length() > 0 )
	{
		moveDir = moveDir.normalize() * MOVE_SPEED;
		camera.move( moveDir );
	}
}