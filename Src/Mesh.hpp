#pragma once

#include "Math.hpp"

struct Mesh
{
	std::vector<Triangle>    triangles;
	std::unique_ptr<BVHNode> bvh;
	Vec3                     position;
	float                    scale;
	AABB                     bbox;

	Mesh() : position( 0, 0, 0 ), scale( 1.0f ) {}
	Mesh( const Mesh & )            = delete;
	Mesh &operator=( const Mesh & ) = delete;
	Mesh( Mesh &&other ) noexcept
	    : triangles( std::move( other.triangles ) ), bvh( std::move( other.bvh ) ),
	      position( other.position ), scale( other.scale ), bbox( other.bbox )
	{
	}

	Mesh &operator=( Mesh &&other ) noexcept;

	void translate( Vec3 trans );
	void setScale( float s );
	void buildBVH();
};
