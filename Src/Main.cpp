#include <SDL2/SDL.h>
#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "Camera.hpp"
#include "CameraController.hpp"
#include "Math.hpp"
#include "RenderUtils.hpp"
#include "Utils.hpp"

const int WINDOW_WIDTH  = 1080;
const int WINDOW_HEIGHT = 720;

const int RENDER_TARGET_WIDTH  = WINDOW_WIDTH;
const int RENDER_TARGET_HEIGHT = WINDOW_HEIGHT;

const int MAX_DEPTH = 5;
const int THREADS   = std::thread::hardware_concurrency();

Vec3 trace( const Ray                             &ray,
            std::mt19937                          &rng,
            std::uniform_real_distribution<float> &dist,
            const SphereBVH                       &sphereBVH,
            const std::vector<Mesh>               &meshes,
            int                                    depth = 0 )
{
	if ( depth >= MAX_DEPTH )
		return Vec3( 0 );

	Hit closestHit;
	closestHit.t = 1e9;
	bool hit     = false;

	if ( sphereBVH.intersect( ray, closestHit ) )
	{
		hit = true;
	}

	for ( const auto &mesh : meshes )
	{
		Hit meshHit = closestHit;
		if ( intersectMesh( ray, mesh, meshHit ) && meshHit.t < closestHit.t )
		{
			closestHit = meshHit;
			hit        = true;
		}
	}

	Hit groundHit;
	groundHit.t = 1e9;
	if ( intersectGround( ray, groundHit ) && groundHit.t < closestHit.t )
	{
		closestHit = groundHit;
		hit        = true;
	}

	if ( hit )
	{
		Vec3 p = ray.origin + ray.dir * closestHit.t;
		if ( closestHit.reflective )
		{
			Vec3 reflectDir = ray.dir - closestHit.normal * 2.0f * ray.dir.dot( closestHit.normal );
			return trace( Ray( p + reflectDir * 0.001f, reflectDir.normalize() ),
			              rng,
			              dist,
			              sphereBVH,
			              meshes,
			              depth + 1 ) *
			       closestHit.color;
		}

		float r1  = dist( rng );
		float r2  = dist( rng );
		float phi = 2 * M_PI * r1;
		float r   = std::sqrt( r2 );
		float x = r * std::cos( phi ), y = r * std::sin( phi ), z = std::sqrt( 1 - r2 );
		Vec3  u = closestHit.normal
		             .cross( std::abs( closestHit.normal.x ) > 0.1f ? Vec3( 0, 1, 0 ) : Vec3( 1, 0, 0 ) )
		             .normalize();
		Vec3 v   = closestHit.normal.cross( u );
		Vec3 dir = ( u * x + v * y + closestHit.normal * z ).normalize();

		return closestHit.color *
		       trace( Ray( p + dir * 0.001f, dir ), rng, dist, sphereBVH, meshes, depth + 1 );
	}

	return Vec3( 0.2f, 0.3f, 0.6f );
}

void renderBlock( std::vector<Vec3>       &accum,
                  std::vector<uint32_t>   &pixels,
                  Camera                  &camera,
                  const SphereBVH         &sphereBVH,
                  const std::vector<Mesh> &meshes,
                  int                      startY,
                  int                      endY,
                  std::atomic<int>        &frameCount )
{
	std::mt19937                          rng( SDL_GetTicks() + startY );
	std::uniform_real_distribution<float> dist( 0, 1 );
	for ( int y = startY; y < endY; ++y )
	{
		for ( int x = 0; x < RENDER_TARGET_WIDTH; ++x )
		{
			float u = ( x + dist( rng ) ) / RENDER_TARGET_WIDTH * 2 - 1;
			float v = ( y + dist( rng ) ) / RENDER_TARGET_HEIGHT * 2 - 1;
			u *= (float)RENDER_TARGET_WIDTH / RENDER_TARGET_HEIGHT;
			Ray  ray   = camera.getRay( u, -v );
			Vec3 color = trace( ray, rng, dist, sphereBVH, meshes );
			int  idx   = y * RENDER_TARGET_WIDTH + x;
			accum[idx] += color;
			Vec3 avg = accum[idx] * ( 1.0f / frameCount.load() );
			avg.x    = std::pow( std::clamp( avg.x, 0.0f, 1.0f ), 1 / 2.2f );
			avg.y    = std::pow( std::clamp( avg.y, 0.0f, 1.0f ), 1 / 2.2f );
			avg.z    = std::pow( std::clamp( avg.z, 0.0f, 1.0f ), 1 / 2.2f );
			pixels[idx] =
			    ( uint8_t( avg.x * 255 ) << 16 ) | ( uint8_t( avg.y * 255 ) << 8 ) | uint8_t( avg.z * 255 );
		}
	}
}

int main( int argc, char *argv[] )
{
	SDL_Init( SDL_INIT_VIDEO );

	SDL_Window *win =
	    SDL_CreateWindow( "DK's Path Tracer", 100, 100, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN );
	SDL_Renderer *ren = SDL_CreateRenderer( win, -1, SDL_RENDERER_ACCELERATED );

	SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1" );
	SDL_RenderSetLogicalSize( ren, RENDER_TARGET_WIDTH, RENDER_TARGET_HEIGHT );
	SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1" );

	SDL_Texture *tex = SDL_CreateTexture( ren,
	                                      SDL_PIXELFORMAT_RGB888,
	                                      SDL_TEXTUREACCESS_STREAMING,
	                                      RENDER_TARGET_WIDTH,
	                                      RENDER_TARGET_HEIGHT );

	std::vector<uint32_t> pixels( RENDER_TARGET_WIDTH * RENDER_TARGET_HEIGHT );
	std::vector<Vec3>     accum( RENDER_TARGET_WIDTH * RENDER_TARGET_HEIGHT );
	std::atomic<int>      frameCount = 1;

	std::vector<std::tuple<Vec3, float, Vec3, bool>> spheres = {
	    { { 0, 1.0f, 0 }, 1.0f, { 1, 1.0f, 1.0f }, true },
	    { { -2, 1, -2 }, 1.0f, { 1, 0.2f, 0.2f }, false },
	    { { -3, 1, -6 }, 1.0f, { 0.2f, 1, 0.2f }, false },
	    { { 4, 1, -4 }, 1.0f, { 0.2f, 0.2f, 1 }, false },
	    { { 3, 1, -6 }, 0.5f, { 1, 1, 0.2f }, false },
	};

	SphereBVH         sphereBVH( spheres, 0, spheres.size() );
	std::vector<Mesh> meshes;

	if ( argc > 1 )
	{
		Mesh objMesh;
		if ( loadOBJ( argv[1], objMesh, Vec3( 0.9f, 0.9f, 0.9f ), false ) )
		{
			objMesh.setScale( 1.0f );
			objMesh.translate( Vec3( 0, 1.0f, -5.0f ) );
			objMesh.buildBVH();
			meshes.push_back( std::move( objMesh ) );
			std::cout << "Loaded OBJ with " << meshes.back().triangles.size() << " triangles" << std::endl;
		}
	}

	Camera camera( Vec3( 0, 2, 0 ) );

	int  prevMouseX = 0, prevMouseY = 0;
	bool mouseGrabbed          = false;
	bool showBVH               = false;
	bool showTriangles         = false;
	int  bvhVisualizationDepth = 2;

	const int                BLOCK_SIZE = RENDER_TARGET_HEIGHT / THREADS;
	std::vector<std::thread> renderThreads( THREADS );

	bool      running = true;
	SDL_Event event;
	while ( running )
	{
		while ( SDL_PollEvent( &event ) )
		{
			if ( event.type == SDL_QUIT )
				running = false;
			else if ( event.type == SDL_KEYDOWN )
			{
				if ( event.key.keysym.sym == SDLK_ESCAPE )
					running = false;
				else if ( event.key.keysym.sym == SDLK_r )
				{
					std::fill( accum.begin(), accum.end(), Vec3( 0 ) );
					frameCount = 1;
				}
				else if ( event.key.keysym.sym == SDLK_b )
				{
					showBVH = !showBVH;
				}
				else if ( event.key.keysym.sym == SDLK_t )
				{
					showTriangles = !showTriangles;
				}
				else if ( event.key.keysym.sym == SDLK_PLUS || event.key.keysym.sym == SDLK_EQUALS )
				{
					bvhVisualizationDepth = std::min( 10, bvhVisualizationDepth + 1 );
					std::cout << "BVH Visualization Depth: " << bvhVisualizationDepth << std::endl;
				}
				else if ( event.key.keysym.sym == SDLK_MINUS )
				{
					bvhVisualizationDepth = std::max( 0, bvhVisualizationDepth - 1 );
					std::cout << "BVH Visualization Depth: " << bvhVisualizationDepth << std::endl;
				}
			}
			else if ( event.type == SDL_MOUSEBUTTONDOWN )
			{
				if ( event.button.button == SDL_BUTTON_LEFT )
				{
					SDL_SetRelativeMouseMode( SDL_TRUE );
					mouseGrabbed = true;
				}
			}
			else if ( event.type == SDL_MOUSEBUTTONUP )
			{
				if ( event.button.button == SDL_BUTTON_LEFT )
				{
					SDL_SetRelativeMouseMode( SDL_FALSE );
					mouseGrabbed = false;
				}
			}
			else if ( event.type == SDL_MOUSEMOTION && mouseGrabbed )
			{
				float xrel = -event.motion.xrel * MOUSE_SENSITIVITY;
				float yrel = event.motion.yrel * MOUSE_SENSITIVITY;
				camera.rotate( xrel, -yrel );
				std::fill( accum.begin(), accum.end(), Vec3( 0 ) );
				frameCount = 1;
			}
		}

		bool  cameraChanged = false;
		Vec3  oldPosition   = camera.position;
		float oldYaw        = camera.yaw;
		float oldPitch      = camera.pitch;

		cameraControllerMove( camera );

		if ( oldPosition.x != camera.position.x || oldPosition.y != camera.position.y ||
		     oldPosition.z != camera.position.z || oldYaw != camera.yaw || oldPitch != camera.pitch )
		{
			cameraChanged = true;
		}

		if ( cameraChanged )
		{
			std::fill( accum.begin(), accum.end(), Vec3( 0 ) );
			frameCount = 1;
		}

		for ( int i = 0; i < THREADS; ++i )
		{
			int startY       = i * BLOCK_SIZE;
			int endY         = ( i == THREADS - 1 ) ? RENDER_TARGET_HEIGHT : startY + BLOCK_SIZE;
			renderThreads[i] = std::thread( renderBlock,
			                                std::ref( accum ),
			                                std::ref( pixels ),
			                                std::ref( camera ),
			                                std::ref( sphereBVH ),
			                                std::ref( meshes ),
			                                startY,
			                                endY,
			                                std::ref( frameCount ) );
		}

		for ( auto &thread : renderThreads )
		{
			thread.join();
		}

		SDL_UpdateTexture( tex, nullptr, pixels.data(), RENDER_TARGET_WIDTH * sizeof( uint32_t ) );
		SDL_RenderClear( ren );

		SDL_Rect dst = { 0, 0, RENDER_TARGET_WIDTH, RENDER_TARGET_HEIGHT };
		SDL_RenderCopy( ren, tex, nullptr, &dst );

		if ( showBVH )
		{
			debugRenderBoundingBoxes( ren,
			                          meshes,
			                          sphereBVH,
			                          camera,
			                          RENDER_TARGET_WIDTH,
			                          RENDER_TARGET_HEIGHT,
			                          bvhVisualizationDepth );
		}

		if ( showTriangles )
		{
			debugRenderTriangles( ren, meshes, camera, RENDER_TARGET_WIDTH, RENDER_TARGET_HEIGHT );
		}

		SDL_RenderPresent( ren );
		frameCount++;
	}

	SDL_DestroyTexture( tex );
	SDL_DestroyRenderer( ren );
	SDL_DestroyWindow( win );
	SDL_Quit();
	return 0;
}