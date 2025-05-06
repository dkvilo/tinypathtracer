#include <algorithm>
#include <math.h>

#include "Camera.hpp"

Camera::Camera( Vec3 pos ) : position( pos )
{
	updateVectors();
}
void Camera::updateVectors()
{
	float yawRad   = yaw * M_PI / 180.0f;
	float pitchRad = pitch * M_PI / 180.0f;
	forward        = Vec3( std::cos( pitchRad ) * std::cos( yawRad ),
                    std::sin( pitchRad ),
                    std::cos( pitchRad ) * std::sin( yawRad ) )
	              .normalize();
	right = Vec3( 0, 1, 0 ).cross( forward ).normalize();
	up    = forward.cross( right );
}

void Camera::move( Vec3 offset )
{
	position = position + offset;
}

void Camera::rotate( float deltaYaw, float deltaPitch )
{
	yaw += deltaYaw;
	pitch = std::clamp( pitch + deltaPitch, -89.0f, 89.0f );
	updateVectors();
}

Ray Camera::getRay( float px, float py ) const
{
	Vec3 dir = forward + right * px + up * py;
	return Ray( position, dir.normalize() );
}

std::pair<int, int> projectPointToScreen( const Vec3 &p, const Camera &camera, int width, int height )
{
	Vec3  relPos = p - camera.position;
	float depth  = relPos.dot( camera.forward );
	if ( depth <= 0.1f )
	{
		return { -1, -1 };
	}

	float screenX = relPos.dot( camera.right ) / depth;
	float screenY = relPos.dot( camera.up ) / depth;

	int x = static_cast<int>( ( screenX + 1.0f ) * 0.5f * width );
	int y = static_cast<int>( ( 1.0f - ( screenY + 1.0f ) * 0.5f ) * height );
	return { x, y };
}