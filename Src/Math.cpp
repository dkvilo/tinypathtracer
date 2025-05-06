#include "Math.hpp"
#include <algorithm>

Vec3 Vec3::operator+( const Vec3 &b ) const
{
	return { x + b.x, y + b.y, z + b.z };
}

Vec3 Vec3::operator-( const Vec3 &b ) const
{
	return { x - b.x, y - b.y, z - b.z };
}

Vec3 Vec3::operator*( float b ) const
{
	return { x * b, y * b, z * b };
}

Vec3 Vec3::operator*( const Vec3 &b ) const
{
	return { x * b.x, y * b.y, z * b.z };
}

Vec3 &Vec3::operator+=( const Vec3 &b )
{
	x += b.x;
	y += b.y;
	z += b.z;
	return *this;
}

Vec3 &Vec3::Vec3::operator-=( const Vec3 &b )
{
	x -= b.x;
	y -= b.y;
	z -= b.z;
	return *this;
}

float Vec3::dot( const Vec3 &b ) const
{
	return x * b.x + y * b.y + z * b.z;
}

Vec3 Vec3::cross( const Vec3 &b ) const
{
	return { y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x };
}

float Vec3::length() const
{
	return std::sqrt( x * x + y * y + z * z );
}

Vec3 Vec3::normalize() const
{
	float len = length();
	return len > 0 ? *this * ( 1.0f / len ) : *this;
}

float Vec3::operator[]( int i ) const
{
	if ( i == 0 )
		return x;
	if ( i == 1 )
		return y;
	return z;
}

bool AABB::intersect( const Ray &ray, float &tMin, float &tMax ) const
{
	float txMin = ( min.x - ray.origin.x ) / ray.dir.x;
	float txMax = ( max.x - ray.origin.x ) / ray.dir.x;
	if ( ray.dir.x < 0 )
		std::swap( txMin, txMax );

	float tyMin = ( min.y - ray.origin.y ) / ray.dir.y;
	float tyMax = ( max.y - ray.origin.y ) / ray.dir.y;
	if ( ray.dir.y < 0 )
		std::swap( tyMin, tyMax );

	if ( txMin > tyMax || tyMin > txMax )
		return false;

	tMin = std::max( txMin, tyMin );
	tMax = std::min( txMax, tyMax );

	float tzMin = ( min.z - ray.origin.z ) / ray.dir.z;
	float tzMax = ( max.z - ray.origin.z ) / ray.dir.z;
	if ( ray.dir.z < 0 )
		std::swap( tzMin, tzMax );

	if ( tMin > tzMax || tzMin > tMax )
		return false;

	tMin = std::max( tMin, tzMin );
	tMax = std::min( tMax, tzMax );

	return tMax > 0;
}

Vec3 AABB::getCenter() const
{
	return ( min + max ) * 0.5f;
}

Triangle::Triangle( Vec3 a, Vec3 b, Vec3 c, Vec3 col, bool refl )
    : v0( a ), v1( b ), v2( c ), color( col ), reflective( refl )
{

	Vec3 edge1 = v1 - v0;
	Vec3 edge2 = v2 - v0;
	normal     = edge1.cross( edge2 ).normalize();

	bbox.min.x = std::min( std::min( a.x, b.x ), c.x );
	bbox.min.y = std::min( std::min( a.y, b.y ), c.y );
	bbox.min.z = std::min( std::min( a.z, b.z ), c.z );

	bbox.max.x = std::max( std::max( a.x, b.x ), c.x );
	bbox.max.y = std::max( std::max( a.y, b.y ), c.y );
	bbox.max.z = std::max( std::max( a.z, b.z ), c.z );
}

BVHNode::BVHNode( std::vector<Triangle> &tris, int startIdx, int endIdx, int depth )
{
	const int MAX_TRIANGLES_PER_LEAF = 4;
	const int MAX_DEPTH              = 20;
	for ( int i = startIdx; i < endIdx; i++ )
	{
		bbox = AABB::combine( bbox, tris[i].bbox );
	}

	int numTriangles = endIdx - startIdx;
	if ( numTriangles <= MAX_TRIANGLES_PER_LEAF || depth >= MAX_DEPTH )
	{
		triangles.reserve( numTriangles );
		for ( int i = startIdx; i < endIdx; i++ )
		{
			triangles.push_back( tris[i] );
		}
		return;
	}

	int axis = depth % 3;
	std::sort( tris.begin() + startIdx,
	           tris.begin() + endIdx,
	           [axis]( const Triangle &a, const Triangle &b )
	           {
		           float centerA = ( a.bbox.min[axis] + a.bbox.max[axis] ) * 0.5f;
		           float centerB = ( b.bbox.min[axis] + b.bbox.max[axis] ) * 0.5f;
		           return centerA < centerB;
	           } );

	int mid = startIdx + numTriangles / 2;

	left  = std::make_unique<BVHNode>( tris, startIdx, mid, depth + 1 );
	right = std::make_unique<BVHNode>( tris, mid, endIdx, depth + 1 );
}

bool BVHNode::intersect( const Ray &ray, Hit &hit ) const
{
	float tMin, tMax;
	if ( !bbox.intersect( ray, tMin, tMax ) || tMax < 0.001f || tMin > hit.t )
	{
		return false;
	}

	bool hitAnything = false;
	if ( !left && !right )
	{
		for ( const auto &tri : triangles )
		{
			Hit tempHit = hit;
			if ( intersectTriangle( ray, tri, tempHit ) && tempHit.t < hit.t )
			{
				hit         = tempHit;
				hitAnything = true;
			}
		}
		return hitAnything;
	}

	if ( left )
	{
		hitAnything |= left->intersect( ray, hit );
	}

	if ( right )
	{
		hitAnything |= right->intersect( ray, hit );
	}

	return hitAnything;
}

bool intersectSphere( const Ray &ray, Vec3 center, float radius, Vec3 color, bool reflective, Hit &hit )
{
	Vec3  oc = ray.origin - center;
	float b  = oc.dot( ray.dir );
	float c  = oc.dot( oc ) - radius * radius;
	float h  = b * b - c;
	if ( h < 0 )
		return false;
	h       = std::sqrt( h );
	float t = -b - h;
	if ( t < 0.001f )
		t = -b + h;
	if ( t < 0.001f )
		return false;
	hit.t          = t;
	Vec3 p         = ray.origin + ray.dir * t;
	hit.normal     = ( p - center ).normalize();
	hit.color      = color;
	hit.reflective = reflective;
	return true;
}

bool intersectTriangle( const Ray &ray, const Triangle &tri, Hit &hit )
{
	const float EPSILON = 0.0000001f;
	Vec3        edge1   = tri.v1 - tri.v0;
	Vec3        edge2   = tri.v2 - tri.v0;
	Vec3        h       = ray.dir.cross( edge2 );
	float       a       = edge1.dot( h );

	if ( a > -EPSILON && a < EPSILON )
		return false; // ray is parallel to triangle

	float f = 1.0f / a;
	Vec3  s = ray.origin - tri.v0;
	float u = f * s.dot( h );

	if ( u < 0.0f || u > 1.0f )
		return false;

	Vec3  q = s.cross( edge1 );
	float v = f * ray.dir.dot( q );

	if ( v < 0.0f || u + v > 1.0f )
		return false;

	float t = f * edge2.dot( q );

	if ( t > EPSILON )
	{
		hit.t          = t;
		hit.normal     = tri.normal;
		hit.color      = tri.color;
		hit.reflective = tri.reflective;
		return true;
	}

	return false;
}

bool intersectGround( const Ray &ray, Hit &hit )
{
	if ( ray.dir.y >= -0.001f )
		return false;
	float t = -ray.origin.y / ray.dir.y;
	if ( t < 0.001f )
		return false;
	hit.t          = t;
	hit.normal     = Vec3( 0, 1, 0 );
	hit.color      = Vec3( 0.7f, 0.7f, 0.7f );
	hit.reflective = false;
	return true;
}

SphereBVH::SphereBVH( std::vector<std::tuple<Vec3, float, Vec3, bool>> &sphereList,
                      int                                               startIdx,
                      int                                               endIdx,
                      int                                               depth )
{
	const int MAX_SPHERES_PER_LEAF = 4;
	const int MAX_DEPTH            = 20;

	for ( int i = startIdx; i < endIdx; i++ )
	{
		const auto &[center, radius, color, reflective] = sphereList[i];
		AABB sphereBox( Vec3( center.x - radius, center.y - radius, center.z - radius ),
		                Vec3( center.x + radius, center.y + radius, center.z + radius ) );
		bbox = AABB::combine( bbox, sphereBox );
	}

	int numSpheres = endIdx - startIdx;
	if ( numSpheres <= MAX_SPHERES_PER_LEAF || depth >= MAX_DEPTH )
	{
		spheres.reserve( numSpheres );
		for ( int i = startIdx; i < endIdx; i++ )
		{
			spheres.push_back( sphereList[i] );
		}
		return;
	}

	int axis = depth % 3;

	std::sort( sphereList.begin() + startIdx,
	           sphereList.begin() + endIdx,
	           [axis]( const auto &a, const auto &b )
	           { return std::get<0>( a )[axis] < std::get<0>( b )[axis]; } );

	int mid = startIdx + numSpheres / 2;

	left  = std::make_unique<SphereBVH>( sphereList, startIdx, mid, depth + 1 );
	right = std::make_unique<SphereBVH>( sphereList, mid, endIdx, depth + 1 );
}

bool SphereBVH::intersect( const Ray &ray, Hit &hit ) const
{
	float tMin, tMax;
	if ( !bbox.intersect( ray, tMin, tMax ) || tMax < 0.001f || tMin > hit.t )
	{
		return false;
	}

	bool hitAnything = false;
	if ( !left && !right )
	{
		for ( const auto &[center, radius, color, reflective] : spheres )
		{
			Hit tempHit = hit;
			if ( intersectSphere( ray, center, radius, color, reflective, tempHit ) && tempHit.t < hit.t )
			{
				hit         = tempHit;
				hitAnything = true;
			}
		}
		return hitAnything;
	}

	if ( left )
	{
		hitAnything |= left->intersect( ray, hit );
	}
	if ( right )
	{
		hitAnything |= right->intersect( ray, hit );
	}

	return hitAnything;
}