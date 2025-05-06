#pragma once

#include <vector>

struct Vec3
{
	float x, y, z;
	Vec3( float a = 0 ) : x( a ), y( a ), z( a ) {}
	Vec3( float x_, float y_, float z_ ) : x( x_ ), y( y_ ), z( z_ ) {}

	Vec3  operator+( const Vec3 &b ) const;
	Vec3  operator-( const Vec3 &b ) const;
	Vec3  operator*( float b ) const;
	Vec3  operator*( const Vec3 &b ) const;
	Vec3 &operator+=( const Vec3 &b );

	Vec3 &operator-=( const Vec3 &b );

	float dot( const Vec3 &b ) const;

	Vec3 cross( const Vec3 &b ) const;

	float length() const;

	Vec3 normalize() const;

	float operator[]( int i ) const;
};

struct Ray
{
	Vec3 origin, dir;
	Ray( Vec3 o, Vec3 d ) : origin( o ), dir( d ) {}
};

struct Hit
{
	float t;
	Vec3  normal;
	Vec3  color;
	bool  reflective = false;
};

struct AABB
{
	Vec3 min, max;

	AABB() : min( std::numeric_limits<float>::max() ), max( -std::numeric_limits<float>::max() ) {}

	AABB( const Vec3 &min_, const Vec3 &max_ ) : min( min_ ), max( max_ ) {}

	bool intersect( const Ray &ray, float &tMin, float &tMax ) const;

	static AABB combine( const AABB &a, const AABB &b )
	{
		return AABB(
		    Vec3( std::min( a.min.x, b.min.x ), std::min( a.min.y, b.min.y ), std::min( a.min.z, b.min.z ) ),
		    Vec3( std::max( a.max.x, b.max.x ),
		          std::max( a.max.y, b.max.y ),
		          std::max( a.max.z, b.max.z ) ) );
	}

	Vec3 getCenter() const;
};

struct Triangle
{
	Vec3 v0, v1, v2;
	Vec3 normal;
	Vec3 color;
	bool reflective;
	AABB bbox;

	Triangle( Vec3 a, Vec3 b, Vec3 c, Vec3 col, bool refl );
};

bool intersectTriangle( const Ray &ray, const Triangle &tri, Hit &hit );

struct BVHNode
{
	AABB                     bbox;
	std::unique_ptr<BVHNode> left;
	std::unique_ptr<BVHNode> right;
	std::vector<Triangle>    triangles;

	BVHNode() = default;
	BVHNode( std::vector<Triangle> &tris, int startIdx, int endIdx, int depth = 0 );

	bool intersect( const Ray &ray, Hit &hit ) const;
};

struct SphereBVH
{
	AABB                                             bbox;
	std::unique_ptr<SphereBVH>                       left;
	std::unique_ptr<SphereBVH>                       right;
	std::vector<std::tuple<Vec3, float, Vec3, bool>> spheres;

	SphereBVH() = default;

	SphereBVH( std::vector<std::tuple<Vec3, float, Vec3, bool>> &sphereList,
	           int                                               startIdx,
	           int                                               endIdx,
	           int                                               depth = 0 );

	bool intersect( const Ray &ray, Hit &hit ) const;
};

bool intersectSphere( const Ray &ray, Vec3 center, float radius, Vec3 color, bool reflective, Hit &hit );

bool intersectTriangle( const Ray &ray, const Triangle &tri, Hit &hit );

bool intersectGround( const Ray &ray, Hit &hit );
