#pragma once

#include "Math.hpp"

struct Camera
{
	Vec3  position, forward, right, up;
	float yaw = -90.0f, pitch = 0.0f;

	Camera( Vec3 pos );

	void updateVectors();

	void move( Vec3 offset );
	void rotate( float deltaYaw, float deltaPitch );
	Ray  getRay( float px, float py ) const;
};

std::pair<int, int> projectPointToScreen( const Vec3 &p, const Camera &camera, int width, int height );