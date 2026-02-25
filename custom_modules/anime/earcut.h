/* earcut.h */

#ifndef EARCUT_H
#define EARCUT_H

#include "core/object/ref_counted.h"
#include "core/io/packed_data_container.h"

#include <vector>
#include <array>

typedef double Coord;
typedef std::array<Coord,2> Point;
typedef std::vector<Point> _Polygon;
typedef std::vector<_Polygon> Polygons;

class Earcut : public RefCounted {
    GDCLASS(Earcut, RefCounted)

//private:
//    float time_passed;

protected:
    static void _bind_methods();

public:

    Earcut();
    ~Earcut();

    void _init(); // our initializer called by Godot

    void _process(const double delta);
    //std::vector<int> earcut(const std::vector<Point> &p_polygon, const std::vector<int> &p_holes);
    //godot::Array earcut(godot::Array p_polygon, godot::Array p_holes);// {return godot::Array();}
    PackedInt32Array earcut(PackedVector2Array p_polygon, PackedInt32Array p_holes);
};

#endif // EARCUT_H