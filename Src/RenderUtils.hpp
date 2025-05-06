#include "Camera.hpp"
#include "Mesh.hpp"
#include <SDL2/SDL.h>

void drawBoundingBox( SDL_Renderer *renderer,
                      const AABB   &bbox,
                      const Camera &camera,
                      int           width,
                      int           height,
                      Uint8         r,
                      Uint8         g,
                      Uint8         b )
{

	Vec3 corners[8];
	corners[0] = Vec3( bbox.min.x, bbox.min.y, bbox.min.z );
	corners[1] = Vec3( bbox.max.x, bbox.min.y, bbox.min.z );
	corners[2] = Vec3( bbox.min.x, bbox.max.y, bbox.min.z );
	corners[3] = Vec3( bbox.max.x, bbox.max.y, bbox.min.z );
	corners[4] = Vec3( bbox.min.x, bbox.min.y, bbox.max.z );
	corners[5] = Vec3( bbox.max.x, bbox.min.y, bbox.max.z );
	corners[6] = Vec3( bbox.min.x, bbox.max.y, bbox.max.z );
	corners[7] = Vec3( bbox.max.x, bbox.max.y, bbox.max.z );

	std::pair<int, int> screenCorners[8];
	for ( int i = 0; i < 8; i++ )
	{
		screenCorners[i] = projectPointToScreen( corners[i], camera, width, height );
	}

	SDL_SetRenderDrawColor( renderer, r, g, b, 255 );
	const int edges[12][2] = {
	    { 0, 1 },
	    { 0, 2 },
	    { 1, 3 },
	    { 2, 3 }, // bottom
	    { 4, 5 },
	    { 4, 6 },
	    { 5, 7 },
	    { 6, 7 }, // top
	    { 0, 4 },
	    { 1, 5 },
	    { 2, 6 },
	    { 3, 7 } // connection
	};

	for ( int i = 0; i < 12; i++ )
	{
		int a = edges[i][0];
		int b = edges[i][1];
		if ( screenCorners[a].first >= 0 && screenCorners[a].second >= 0 && screenCorners[b].first >= 0 &&
		     screenCorners[b].second >= 0 )
		{
			SDL_RenderDrawLine( renderer,
			                    screenCorners[a].first,
			                    screenCorners[a].second,
			                    screenCorners[b].first,
			                    screenCorners[b].second );
		}
	}
}

void visualizeBVHNode( SDL_Renderer  *renderer,
                       const BVHNode *node,
                       const Camera  &camera,
                       int            width,
                       int            height,
                       int            depth    = 0,
                       int            maxDepth = 5 )
{
	if ( !node )
		return;

	if ( depth > maxDepth )
		return;

	Uint8 colors[][3] = {
	    { 255, 0, 0 },   // Red
	    { 0, 255, 0 },   // Green
	    { 0, 0, 255 },   // Blue
	    { 255, 255, 0 }, // Yellow
	    { 255, 0, 255 }, // Magenta
	    { 0, 255, 255 }, // Cyan
	    { 255, 128, 0 }, // Orange
	    { 128, 0, 255 }, // Purple
	    { 0, 128, 255 }, // Light blue
	    { 255, 0, 128 }  // Pink
	};

	Uint8 r = colors[depth % 10][0];
	Uint8 g = colors[depth % 10][1];
	Uint8 b = colors[depth % 10][2];

	drawBoundingBox( renderer, node->bbox, camera, width, height, r, g, b );

	if ( node->left )
	{
		visualizeBVHNode( renderer, node->left.get(), camera, width, height, depth + 1, maxDepth );
	}

	if ( node->right )
	{
		visualizeBVHNode( renderer, node->right.get(), camera, width, height, depth + 1, maxDepth );
	}
}

void visualizeSphereBVHNode( SDL_Renderer    *renderer,
                             const SphereBVH *node,
                             const Camera    &camera,
                             int              width,
                             int              height,
                             int              depth    = 0,
                             int              maxDepth = 5 )
{
	if ( !node )
		return;

	if ( depth > maxDepth )
		return;

	Uint8 colors[][3] = {
	    { 255, 128, 128 }, // red
	    { 128, 255, 128 }, // green
	    { 128, 128, 255 }, // blue
	    { 255, 255, 128 }, // yellow
	    { 255, 128, 255 }, // magenta
	    { 128, 255, 255 }, // cyan
	    { 192, 192, 192 }, // gray
	    { 128, 64, 0 },    // brown
	    { 64, 0, 128 },    // purple
	    { 0, 64, 128 }     // dark blue
	};

	Uint8 r = colors[depth % 10][0];
	Uint8 g = colors[depth % 10][1];
	Uint8 b = colors[depth % 10][2];

	drawBoundingBox( renderer, node->bbox, camera, width, height, r, g, b );

	if ( node->left )
	{
		visualizeSphereBVHNode( renderer, node->left.get(), camera, width, height, depth + 1, maxDepth );
	}

	if ( node->right )
	{
		visualizeSphereBVHNode( renderer, node->right.get(), camera, width, height, depth + 1, maxDepth );
	}
}

void debugRenderBoundingBoxes( SDL_Renderer            *renderer,
                               const std::vector<Mesh> &meshes,
                               const SphereBVH         &sphereBVH,
                               const Camera            &camera,
                               int                      width,
                               int                      height,
                               int                      bvhDepth )
{
	for ( const auto &mesh : meshes )
	{
		if ( mesh.bvh )
		{
			drawBoundingBox( renderer, mesh.bbox, camera, width, height, 255, 0, 0 );
			if ( bvhDepth > 0 && mesh.bvh )
			{
				visualizeBVHNode( renderer, mesh.bvh.get(), camera, width, height, 0, bvhDepth );
			}
		}
	}

	drawBoundingBox( renderer, sphereBVH.bbox, camera, width, height, 0, 255, 0 );
	if ( bvhDepth > 0 )
	{
		visualizeSphereBVHNode( renderer, &sphereBVH, camera, width, height, 0, bvhDepth );
	}
}

void debugRenderTriangles( SDL_Renderer            *renderer,
                           const std::vector<Mesh> &meshes,
                           const Camera            &camera,
                           int                      width,
                           int                      height )
{

	SDL_SetRenderDrawColor( renderer, 0, 255, 0, 255 );

	for ( const auto &mesh : meshes )
	{
		for ( const auto &tri : mesh.triangles )
		{
			Vec3 v0 = ( tri.v0 * mesh.scale ) + mesh.position;
			Vec3 v1 = ( tri.v1 * mesh.scale ) + mesh.position;
			Vec3 v2 = ( tri.v2 * mesh.scale ) + mesh.position;

			auto p0 = projectPointToScreen( v0, camera, width, height );
			auto p1 = projectPointToScreen( v1, camera, width, height );
			auto p2 = projectPointToScreen( v2, camera, width, height );

			if ( p0.first >= 0 && p0.second >= 0 && p1.first >= 0 && p1.second >= 0 && p2.first >= 0 &&
			     p2.second >= 0 )
			{
				SDL_RenderDrawLine( renderer, p0.first, p0.second, p1.first, p1.second );
				SDL_RenderDrawLine( renderer, p1.first, p1.second, p2.first, p2.second );
				SDL_RenderDrawLine( renderer, p2.first, p2.second, p0.first, p0.second );
			}
		}
	}
}

void debugRenderBoundingBoxes( SDL_Renderer            *renderer,
                               const std::vector<Mesh> &meshes,
                               const Camera            &camera,
                               int                      width,
                               int                      height )
{

	auto projectPoint = [&camera, width, height]( const Vec3 &p ) -> std::pair<int, int>
	{
		Vec3  relPos = p - camera.position;
		float depth  = relPos.dot( camera.forward );
		if ( depth <= 0.1f )
		{
			return { -1, -1 };
		}

		float screenX = relPos.dot( camera.right ) / depth;
		float screenY = relPos.dot( camera.up ) / depth;

		int x = (int)( ( screenX + 1.0f ) * 0.5f * width );
		int y = (int)( ( 1.0f - ( screenY + 1.0f ) * 0.5f ) * height );

		return { x, y };
	};

	for ( const auto &mesh : meshes )
	{
		if ( !mesh.bvh )
			continue;

		Vec3 corners[8];
		corners[0] = Vec3( mesh.bbox.min.x, mesh.bbox.min.y, mesh.bbox.min.z );
		corners[1] = Vec3( mesh.bbox.max.x, mesh.bbox.min.y, mesh.bbox.min.z );
		corners[2] = Vec3( mesh.bbox.min.x, mesh.bbox.max.y, mesh.bbox.min.z );
		corners[3] = Vec3( mesh.bbox.max.x, mesh.bbox.max.y, mesh.bbox.min.z );
		corners[4] = Vec3( mesh.bbox.min.x, mesh.bbox.min.y, mesh.bbox.max.z );
		corners[5] = Vec3( mesh.bbox.max.x, mesh.bbox.min.y, mesh.bbox.max.z );
		corners[6] = Vec3( mesh.bbox.min.x, mesh.bbox.max.y, mesh.bbox.max.z );
		corners[7] = Vec3( mesh.bbox.max.x, mesh.bbox.max.y, mesh.bbox.max.z );

		std::pair<int, int> screenCorners[8];
		for ( int i = 0; i < 8; i++ )
		{
			screenCorners[i] = projectPoint( corners[i] );
		}

		SDL_SetRenderDrawColor( renderer, 255, 0, 0, 255 );
		const int edges[12][2] = {
		    { 0, 1 },
		    { 0, 2 },
		    { 1, 3 },
		    { 2, 3 }, // Bottom face
		    { 4, 5 },
		    { 4, 6 },
		    { 5, 7 },
		    { 6, 7 }, // Top face
		    { 0, 4 },
		    { 1, 5 },
		    { 2, 6 },
		    { 3, 7 } // Connecting edges
		};

		for ( int i = 0; i < 12; i++ )
		{
			int a = edges[i][0];
			int b = edges[i][1];

			if ( screenCorners[a].first >= 0 && screenCorners[a].second >= 0 && screenCorners[b].first >= 0 &&
			     screenCorners[b].second >= 0 )
			{
				SDL_RenderDrawLine( renderer,
				                    screenCorners[a].first,
				                    screenCorners[a].second,
				                    screenCorners[b].first,
				                    screenCorners[b].second );
			}
		}
	}
}
