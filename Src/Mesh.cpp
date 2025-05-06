#include "Mesh.hpp"

Mesh &Mesh::operator=( Mesh &&other ) noexcept
{
	if ( this != &other )
	{
		triangles = std::move( other.triangles );
		bvh       = std::move( other.bvh );
		position  = other.position;
		scale     = other.scale;
		bbox      = other.bbox;
	}
	return *this;
}

void Mesh::translate( Vec3 trans )
{
	position = position + trans;
}

void Mesh::setScale( float s )
{
	scale = s;
}

void Mesh::buildBVH()
{
	std::vector<Triangle> transformedTriangles;
	transformedTriangles.reserve( triangles.size() );

	for ( const auto &tri : triangles )
	{
		Triangle transformed = tri;
		transformed.v0       = ( tri.v0 * scale ) + position;
		transformed.v1       = ( tri.v1 * scale ) + position;
		transformed.v2       = ( tri.v2 * scale ) + position;

		Vec3 edge1         = transformed.v1 - transformed.v0;
		Vec3 edge2         = transformed.v2 - transformed.v0;
		transformed.normal = edge1.cross( edge2 ).normalize();

		transformed.bbox.min.x = std::min( std::min( transformed.v0.x, transformed.v1.x ), transformed.v2.x );
		transformed.bbox.min.y = std::min( std::min( transformed.v0.y, transformed.v1.y ), transformed.v2.y );
		transformed.bbox.min.z = std::min( std::min( transformed.v0.z, transformed.v1.z ), transformed.v2.z );

		transformed.bbox.max.x = std::max( std::max( transformed.v0.x, transformed.v1.x ), transformed.v2.x );
		transformed.bbox.max.y = std::max( std::max( transformed.v0.y, transformed.v1.y ), transformed.v2.y );
		transformed.bbox.max.z = std::max( std::max( transformed.v0.z, transformed.v1.z ), transformed.v2.z );

		transformedTriangles.push_back( transformed );
	}

	if ( !transformedTriangles.empty() )
	{
		bvh  = std::make_unique<BVHNode>( transformedTriangles, 0, transformedTriangles.size() );
		bbox = bvh->bbox;
	}
}
