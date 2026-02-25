/* earcut.cpp */

#include "earcut.h"
#include "earcut.hpp"
#include "core/object/class_db.h"

Earcut::Earcut() {
}

Earcut::~Earcut() {
	// add your cleanup here
}

void Earcut::_init() {
	// initialize any variables here
}

void Earcut::_process(const double delta) {
}

PackedInt32Array Earcut::earcut(PackedVector2Array p_polygon, PackedInt32Array p_holes) {
	// A hole is a subset of p_polygon
	// For example, if p_polygon contains 20 vertices and p_holes = [15], then vertices [15-20] are a hole
	
	Polygons polygons;
	_Polygon polygon;
	Point point;
	
	size_t vertex_count = p_polygon.size();
	size_t holes_size = p_holes.size();
	int hole_index = 0;
	
	for (int vertex_index = 0; vertex_index < vertex_count; vertex_index++) {
		point = {p_polygon[vertex_index].x, p_polygon[vertex_index].y};
		
		if (holes_size > 0) {
			// Check if build polygon is a hole
			if (hole_index < holes_size) {
			  if (vertex_index == p_holes[hole_index]) {
				polygons.push_back(polygon);
				polygon.clear();
				hole_index++;
			  }
			}
		}
		
		polygon.push_back(point);
	}
	
    // ensure we catch the last poly ring
    polygons.push_back(polygon);
	
	// Fill polygon structure with actual data. Any winding order works.
	// The first polyline defines the main polygon.
	// Following polylines define holes.

	// Run tessellation
	// Returns array of indices that refer to the vertices of the input polygon.
	// Output triangles are clockwise.
	std::vector<int> indices = mapbox::earcut<int>(polygons);
	
	PackedInt32Array triangles;	
	// Slow, use PoolIntArray
	for (size_t i = 0; i < indices.size(); i++) {
		triangles.push_back(indices[i]);
	}
	
	return triangles;
}

void Earcut::_bind_methods() {
	ClassDB::bind_method(D_METHOD("_process"), &Earcut::_process);
	ClassDB::bind_method(D_METHOD("earcut"), &Earcut::earcut);
	//register_property<GDExample, float>("amplitude", &GDExample::amplitude, 10.0);
	//register_property<GDExample, float>("speed", &GDExample::set_speed, &GDExample::get_speed, 1.0);

	//register_signal<Earcut>((char *)"position_changed", "node", GODOT_VARIANT_TYPE_OBJECT, "new_pos", GODOT_VARIANT_TYPE_VECTOR2);
}