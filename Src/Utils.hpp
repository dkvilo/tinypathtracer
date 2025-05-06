#include "Math.hpp"
#include "Mesh.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

bool loadOBJ( const std::string &filename, Mesh &mesh, Vec3 color, bool reflective )
{
	std::ifstream file( filename );
	if ( !file.is_open() )
	{
		std::cerr << "Failed to open file: " << filename << std::endl;
		return false;
	}

	std::cout << "Loading OBJ file: " << filename << std::endl;

	std::vector<Vec3> vertices;
	std::vector<Vec3> normals;
	std::string       line;

	vertices.push_back( Vec3( 0, 0, 0 ) );
	normals.push_back( Vec3( 0, 0, 0 ) );

	int lineCount   = 0;
	int vertexCount = 0;
	int faceCount   = 0;

	while ( std::getline( file, line ) )
	{
		lineCount++;
		std::istringstream iss( line );
		std::string        token;
		iss >> token;

		if ( token == "v" )
		{
			float x, y, z;
			if ( !( iss >> x >> y >> z ) )
			{
				std::cerr << "Error parsing vertex at line " << lineCount << std::endl;
				continue;
			}
			vertices.push_back( Vec3( x, y, z ) );
			vertexCount++;
		}
		else if ( token == "vn" )
		{
			float x, y, z;
			if ( !( iss >> x >> y >> z ) )
			{
				std::cerr << "Error parsing normal at line " << lineCount << std::endl;
				continue;
			}
			normals.push_back( Vec3( x, y, z ).normalize() );
		}
		else if ( token == "f" )
		{
			std::string v1, v2, v3;
			if ( !( iss >> v1 >> v2 >> v3 ) )
			{
				std::cerr << "Error parsing face at line " << lineCount << std::endl;
				continue;
			}

			auto getIndex = []( const std::string &str ) -> int
			{
				std::istringstream ss( str );
				int                v;
				char               slash;
				ss >> v;
				return v;
			};

			try
			{
				int idx1 = getIndex( v1 );
				int idx2 = getIndex( v2 );
				int idx3 = getIndex( v3 );

				if ( idx1 > 0 && idx2 > 0 && idx3 > 0 && idx1 < vertices.size() && idx2 < vertices.size() &&
				     idx3 < vertices.size() )
				{
					mesh.triangles.emplace_back( vertices[idx1],
					                             vertices[idx2],
					                             vertices[idx3],
					                             color,
					                             reflective );
					faceCount++;
				}
				else
				{
					std::cerr << "Invalid vertex indices at line " << lineCount << ": " << idx1 << ", "
					          << idx2 << ", " << idx3 << " (max: " << vertices.size() - 1 << ")" << std::endl;
				}
			}
			catch ( const std::exception &e )
			{
				std::cerr << "Exception while parsing face at line " << lineCount << ": " << e.what()
				          << std::endl;
			}
		}
	}

	file.close();

	std::cout << "OBJ loaded: " << vertexCount << " vertices, " << faceCount << " faces" << std::endl;

	if ( mesh.triangles.empty() )
	{
		std::cerr << "No valid triangles found in OBJ file" << std::endl;
		return false;
	}

	return true;
}

bool intersectMesh( const Ray &ray, const Mesh &mesh, Hit &hit )
{
	if ( !mesh.bvh )
	{
		return false;
	}

	float tMin, tMax;
	if ( !mesh.bbox.intersect( ray, tMin, tMax ) || tMax < 0.001f || tMin > hit.t )
	{
		return false;
	}

	return mesh.bvh->intersect( ray, hit );
}
