#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <locale.h>
#include <assert.h>
#include "tesselator.h"

typedef struct Vector3f { TESSreal v[3]; } Vector3f;
typedef std::vector<Vector3f> Polygon;
typedef std::vector<Polygon> Polygons;

static void export_stl(const Polygons &triangles, std::ostream &output)
{
  setlocale(LC_NUMERIC, "C"); // Ensure radix is . (not ,) in output
  output << "solid OpenSCAD_Model\n";
  for (size_t pi=0;pi<triangles.size();pi++) {
    const Polygon &p = triangles[pi];
    assert(p.size() == 3); // STL only allows triangles
    std::stringstream stream;
    stream << p[0].v[0] << " " << p[0].v[1] << " " << p[0].v[2];
    std::string vs1 = stream.str();
    stream.str("");
    stream << p[1].v[0] << " " << p[1].v[1] << " " << p[1].v[2];
    std::string vs2 = stream.str();
    stream.str("");
    stream << p[2].v[0] << " " << p[2].v[1] << " " << p[2].v[2];
    std::string vs3 = stream.str();
    //    if (vs1 != vs2 && vs1 != vs3 && vs2 != vs3) {
      // The above condition ensures that there are 3 distinct vertices, but
      // they may be collinear. If they are, the unit normal is meaningless
      // so the default value of "1 0 0" can be used. If the vertices are not
      // collinear then the unit normal must be calculated from the
      // components.
    //      Vector3f normal = (p[1] - p[0]).cross(p[2] - p[0]);
    //      normal.normalize();
    //      output << "  facet normal " << normal[0] << " " << normal[1] << " " << normal[2] << "\n";
    output << "  facet normal 0 0 0\n";
      output << "    outer loop\n";
		
      for (size_t vi=0;vi<p.size();vi++) {
        const Vector3f &v = p[vi];
        output << "      vertex " << v.v[0] << " " << v.v[1] << " " << v.v[2] << "\n";
      }
      output << "    endloop\n";
      output << "  endfacet\n";
      //    }
  }
  output << "endsolid OpenSCAD_Model\n";
  setlocale(LC_NUMERIC, "");      // Set default locale
}


/*!
  file format: 
  1. polygon coordinates (x,y,z) are comma separated (+/- spaces) and 
  each coordinate is on a separate line
  2. each polygon is separated by one or more blank lines
*/
bool import_polygon(Polygons &polyhole, const std::string &filename)
{
  std::ifstream ifs(filename.c_str());
  if (!ifs) return false;

  std::string line;
  Polygon polygon;
  while (std::getline(ifs, line)) {
    std::stringstream ss(line);
    double X = 0.0, Y = 0.0, Z = 0.0;
    if (!(ss >> X)) {
      //ie blank lines => flag start of next polygon 
      if (polygon.size() > 0) polyhole.push_back(polygon);
      polygon.clear();
      continue;
    }
    char c = ss.peek();  
    while (c == ' ') {ss.read(&c, 1); c = ss.peek();} //gobble spaces before comma
    if (c == ',') {ss.read(&c, 1); c = ss.peek();} //gobble comma
    while (c == ' ') {ss.read(&c, 1); c = ss.peek();} //gobble spaces after comma
    if (!(ss >> Y)) {
      std::cerr << "Y error\n";
      return false;
    }
    c = ss.peek();
    while (c == ' ') {ss.read(&c, 1); c = ss.peek();} //gobble spaces before comma
    if (c == ',') {ss.read(&c, 1); c = ss.peek();} //gobble comma
    while (c == ' ') {ss.read(&c, 1); c = ss.peek();} //gobble spaces after comma
    if (!(ss >> Z)) {
      std::cerr << "Z error\n";
      return false;
    }
    Vector3f v;
    v.v[0] = X;
    v.v[1] = Y;
    v.v[2] = Z;
    polygon.push_back(v);
  }
  if (polygon.size() > 0) polyhole.push_back(polygon);
  ifs.close();
  return true;
}

static void *stdAlloc(void* userData, unsigned int size) {
	int* allocated = ( int*)userData;
	TESS_NOTUSED(userData);
	*allocated += (int)size;
	return malloc(size);
}

static void stdFree(void* userData, void* ptr) {
	TESS_NOTUSED(userData);
	free(ptr);
}

/* Returns 1 on error */
int tessellatePolygon(const Polygons &polygons,
                      Polygons &triangles,
                      const Vector3f *normal)
{
  // No polygon. FIXME: Will this ever happen or can we assert here?
  if (polygons.empty()) return false;

	// if (polygon.size() == 3) {
	// 	PRINTD("input polygon has 3 points. shortcut tessellation.");
	// 	Polygon t;
	// 	t.push_back(Vector3d(polygon[0].x(), polygon[0].y(), polygon[0].z()));
	// 	t.push_back(Vector3d(polygon[1].x(), polygon[1].y(), polygon[1].z()));
	// 	t.push_back(Vector3d(polygon[2].x(), polygon[2].y(), polygon[2].z()));
	// 	triangles.push_back(t);
	// 	return false;
	// }

  TESSreal *normalvec = NULL;
  TESSreal passednormal[3];
  if (normal) {
    passednormal[0] = normal->v[0];
    passednormal[1] = normal->v[1];
    passednormal[2] = normal->v[2];
    normalvec = passednormal;
  }

  int allocated = 0;
  TESSalloc ma;
  TESStesselator* tess = 0;

  memset(&ma, 0, sizeof(ma));
  ma.memalloc = stdAlloc;
  ma.memfree = stdFree;
  ma.userData = (void*)&allocated;
  ma.extraVertices = 256; // realloc not provided, allow 256 extra vertices.
  
  tess = tessNewTess(&ma);
  if (!tess) return -1;

  std::vector<TESSreal> contour;
  std::vector<Vector3f> outputvertices;
  for (size_t pi=0;pi<polygons.size();pi++) {
    const Polygon &poly = polygons[pi];
    contour.clear();
    for (size_t vi=0;vi<poly.size();vi++) {
      const Vector3f &v = poly[vi];
      outputvertices.push_back(v);
      contour.push_back(v.v[0]);
      contour.push_back(v.v[1]);
      contour.push_back(v.v[2]);
    }
    tessAddContour(tess, 3, &contour.front(), sizeof(TESSreal) * 3, poly.size());
  }

  //  if (!tessTesselate(tess, TESS_WINDING_ODD, TESS_CONSTRAINED_DELAUNAY_TRIANGLES, 3, 3, normalvec)) return -1;
  if (!tessTesselate(tess, TESS_WINDING_ODD, TESS_POLYGONS, 3, 3, normalvec)) return 1;

  const TESSindex *vindices = tessGetVertexIndices(tess);
  const TESSindex *elements = tessGetElements(tess);
  int numelems = tessGetElementCount(tess);
  
  Polygon tri;
  for (int t=0;t<numelems;t++) {
    tri.resize(3);
    bool err = false;
    for (int i=0;i<3;i++) {
      int eidx = elements[t*3 + i];
      int vidx = vindices[eidx];
      printf("%d (%d) ", eidx, vidx);
      if (vidx != TESS_UNDEF) {
        tri[i] = outputvertices[vidx];
      }
      else {
        err = true;
      }
    }
    if (!err) triangles.push_back(tri);
    else printf("Error: Triangle contains UNDEF vertex due to intersection");
    printf("\n");
  }

  tessDeleteTess(tess);

  return 0;
}

int main(int argc, char *argv[])
{
  Polygons polyhole;
  Vector3f *normal = NULL;
  if (argc >= 2) {
    if (!import_polygon(polyhole, argv[1])) {
      std::cerr << "Error importing polygon" << std::endl;
      exit(1);
    }
    std::cerr << "Imported " << polyhole.size() << " polygons" << std::endl;

    if (argc == 3) {
      normal = new Vector3f;
      int idx = 0;
      char *buffer = strtok(argv[2], ",");
      while (buffer) {
        normal->v[idx++] = atof(buffer);
        if (idx == 3) break;
        buffer = strtok(NULL, ",");
      }
      assert(idx == 3);

   }
  }
  else {
    //construct two non-intersecting nested polygons  
    Polygon polygon1;
    Vector3f v;
    v.v[0]=0;v.v[1]=0;v.v[2]=0;
    polygon1.push_back(v);
    v.v[0]=2;v.v[1]=0;v.v[2]=0;
    polygon1.push_back(v);
    v.v[0]=2;v.v[1]=2;v.v[2]=0;
    polygon1.push_back(v);
    v.v[0]=0;v.v[1]=2;v.v[2]=0;
    polygon1.push_back(v);
    Polygon polygon2;
    v.v[0]=0.5;v.v[1]=0.5;v.v[2]=0;
    polygon2.push_back(v);
    v.v[0]=1.5;v.v[1]=0.5;v.v[2]=0;
    polygon2.push_back(v);
    v.v[0]=1.5;v.v[1]=1.5;v.v[2]=0;
    polygon2.push_back(v);
    v.v[0]=0.5;v.v[1]=1.5;v.v[2]=0;
    polygon2.push_back(v);
    polyhole.push_back(polygon1);
    polyhole.push_back(polygon2);
  }

  Polygons triangles;
  bool ok = tessellatePolygon(polyhole, triangles, normal);
  std::cerr << "Tessellated into " << triangles.size() << " triangles" << std::endl;

  export_stl(triangles, std::cout);
}
