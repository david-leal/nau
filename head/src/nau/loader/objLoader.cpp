#include <nau/loader/OBJLoader.h>

// Include files
// Class definition
#include <nau/config.h>
#include <nau/slogger.h>

#include <nau.h>
#include <nau/debug/profile.h>
#include <nau/scene/sceneobject.h>
#include <nau/scene/sceneobjectfactory.h>
#include <nau/geometry/iboundingvolume.h>
#include <nau/geometry/boundingvolumefactory.h>
#include <nau/math/vec3.h>
#include <nau/math/mat4.h>
#include <nau/math/transformfactory.h>
#include <nau/render/vertexdata.h>
#include <nau/render/irenderable.h>
#include <nau/material/materialgroup.h>
#include <nau/clogger.h>
#include <nau/material/material.h>

// Assert and other basics
#include <assert.h>
#include <fstream>
#include <map>
#include <cstring>

#ifdef WIN32
#define PATH_SEPARATOR "\\"
#define PATH_SEPARATOR_C '\\'
#define strdup _strdup
#else
#define PATH_SEPARATOR "/"
#define PATH_SEPARATOR_C '/'
#endif

 //#define OBJLOADER_DEBUG

// Namespaces from Nau too.
using namespace nau::loader;
using namespace nau::scene;
using namespace nau::math;
using namespace nau::geometry;
using namespace nau::render;
using namespace nau::material;

//Include GLM
//#include "GLM/glm.h"

// GLM Stuff start

#include <GL/glew.h>


#ifndef structFile2
#define structFile2

#ifndef M_PI
#define M_PI 3.14159265
#endif

#define GLM_NONE     (0)		/* render with only vertices */
#define GLM_FLAT     (1 << 0)		/* render with facet normals */
#define GLM_SMOOTH   (1 << 1)		/* render with vertex normals */
#define GLM_TEXTURE  (1 << 2)		/* render with texture coords */
#define GLM_COLOR    (1 << 3)		/* render with colors */
#define GLM_MATERIAL (1 << 4)		/* render with materials */


/* GLMmaterial: Structure that defines a material in a model. 
 */
typedef struct _GLMmaterial
{
  char* name;				/* name of material */
  GLfloat diffuse[4];			/* diffuse component */
  GLfloat ambient[4];			/* ambient component */
  GLfloat specular[4];			/* specular component */
  GLfloat emmissive[4];			/* emmissive component */
  GLfloat shininess;			/* specular exponent */
  char*   texture;
} GLMmaterial;

/* GLMtriangle: Structure that defines a triangle in a model.
 */
typedef struct _GLMtriangle {
  GLuint vindices[3];			/* array of triangle vertex indices */
  GLuint nindices[3];			/* array of triangle normal indices */
  GLuint tindices[3];			/* array of triangle texcoord indices*/
  GLuint findex;			/* index of triangle facet normal */
} GLMtriangle;

/* GLMgroup: Structure that defines a group in a model.
 */
typedef struct _GLMgroup {
  char*             name;		/* name of this group */
  GLuint            numtriangles;	/* number of triangles in this group */
  GLuint*           triangles;		/* array of triangle indices */
  GLuint            material;           /* index to material for group */
  struct _GLMgroup* next;		/* pointer to next group in model */
} GLMgroup;

/* GLMmodel: Structure that defines a model.
 */
typedef struct _GLMmodel {
  char*    pathname;			/* path to this model */
  char*    mtllibname;			/* name of the material library */

  GLuint   numvertices;			/* number of vertices in model */
  GLfloat* vertices;			/* array of vertices  */

  GLuint   numnormals;			/* number of normals in model */
  GLfloat* normals;			/* array of normals */

  GLuint   numtexcoords;		/* number of texcoords in model */
  GLfloat* texcoords;			/* array of texture coordinates */

  GLuint   numfacetnorms;		/* number of facetnorms in model */
  GLfloat* facetnorms;			/* array of facetnorms */

  GLuint       numtriangles;		/* number of triangles in model */
  GLMtriangle* triangles;		/* array of triangles */

  GLuint       nummaterials;		/* number of materials in model */
  GLMmaterial* materials;		/* array of materials */

  GLuint       numgroups;		/* number of groups in model */
  GLMgroup*    groups;			/* linked list of groups */

  GLfloat position[3];			/* position of the model */

} GLMmodel;


#endif


/* glmUnitize: "unitize" a model by translating it to the origin and
 * scaling it to fit in a unit cube around the origin.  Returns the
 * scalefactor used.
 *
 * model - properly initialized GLMmodel structure 
 */
GLfloat
glmUnitize(GLMmodel* model);

/* glmDimensions: Calculates the dimensions (width, height, depth) of
 * a model.
 *
 * model      - initialized GLMmodel structure
 * dimensions - array of 3 GLfloats (GLfloat dimensions[3])
 */
GLvoid
glmDimensions(GLMmodel* model, GLfloat* dimensions);

/* glmScale: Scales a model by a given amount.
 * 
 * model - properly initialized GLMmodel structure
 * scale - scalefactor (0.5 = half as large, 2.0 = twice as large)
 */
GLvoid
glmScale(GLMmodel* model, GLfloat scale);

/* glmReverseWinding: Reverse the polygon winding for all polygons in
 * this model.  Default winding is counter-clockwise.  Also changes
 * the direction of the normals.
 * 
 * model - properly initialized GLMmodel structure 
 */
GLvoid
glmReverseWinding(GLMmodel* model);

/* glmFacetNormals: Generates facet normals for a model (by taking the
 * cross product of the two vectors derived from the sides of each
 * triangle).  Assumes a counter-clockwise winding.
 *
 * model - initialized GLMmodel structure
 */
GLvoid
glmFacetNormals(GLMmodel* model);

/* glmVertexNormals: Generates smooth vertex normals for a model.
 * First builds a list of all the triangles each vertex is in.  Then
 * loops through each vertex in the the list averaging all the facet
 * normals of the triangles each vertex is in.  Finally, sets the
 * normal index in the triangle for the vertex to the generated smooth
 * normal.  If the dot product of a facet normal and the facet normal
 * associated with the first triangle in the list of triangles the
 * current vertex is in is greater than the cosine of the angle
 * parameter to the function, that facet normal is not added into the
 * average normal calculation and the corresponding vertex is given
 * the facet normal.  This tends to preserve hard edges.  The angle to
 * use depends on the model, but 90 degrees is usually a good start.
 *
 * model - initialized GLMmodel structure
 * angle - maximum angle (in degrees) to smooth across
 */
GLvoid
glmVertexNormals(GLMmodel* model, GLfloat angle);

/* glmLinearTexture: Generates texture coordinates according to a
 * linear projection of the texture map.  It generates these by
 * linearly mapping the vertices onto a square.
 *
 * model - pointer to initialized GLMmodel structure
 */
GLvoid
glmLinearTexture(GLMmodel* model);

/* glmSpheremapTexture: Generates texture coordinates according to a
 * spherical projection of the texture map.  Sometimes referred to as
 * spheremap, or reflection map texture coordinates.  It generates
 * these by using the normal to calculate where that vertex would map
 * onto a sphere.  Since it is impossible to map something flat
 * perfectly onto something spherical, there is distortion at the
 * poles.  This particular implementation causes the poles along the X
 * axis to be distorted.
 *
 * model - pointer to initialized GLMmodel structure
 */
GLvoid
glmSpheremapTexture(GLMmodel* model);

/* glmDelete: Deletes a GLMmodel structure.
 *
 * model - initialized GLMmodel structure
 */
GLvoid
glmDelete(GLMmodel* model);

/* glmReadOBJ: Reads a model description from a Wavefront .OBJ file.
 * Returns a pointer to the created object which should be free'd with
 * glmDelete().
 *
 * filename - name of the file containing the Wavefront .OBJ format data.  
 */
GLMmodel* 
glmReadOBJ(const char* filename);

/* glmWriteOBJ: Writes a model description in Wavefront .OBJ format to
 * a file.
 *
 * model    - initialized GLMmodel structure
 * filename - name of the file to write the Wavefront .OBJ format data to
 * mode     - a bitwise or of values describing what is written to the file
 *            GLM_NONE    -  write only vertices
 *            GLM_FLAT    -  write facet normals
 *            GLM_SMOOTH  -  write vertex normals
 *            GLM_TEXTURE -  write texture coords
 *            GLM_FLAT and GLM_SMOOTH should not both be specified.
 */
GLvoid
glmWriteOBJ(GLMmodel* model, char* filename, GLuint mode);

/* glmDraw: Renders the model to the current OpenGL context using the
 * mode specified.
 *
 * model    - initialized GLMmodel structure
 * mode     - a bitwise OR of values describing what is to be rendered.
 *            GLM_NONE    -  render with only vertices
 *            GLM_FLAT    -  render with facet normals
 *            GLM_SMOOTH  -  render with vertex normals
 *            GLM_TEXTURE -  render with texture coords
 *            GLM_FLAT and GLM_SMOOTH should not both be specified.
 */
GLvoid
glmDraw(GLMmodel* model, GLuint mode);

/* glmList: Generates and returns a display list for the model using
 * the mode specified.
 *
 * model    - initialized GLMmodel structure
 * mode     - a bitwise OR of values describing what is to be rendered.
 *            GLM_NONE    -  render with only vertices
 *            GLM_FLAT    -  render with facet normals
 *            GLM_SMOOTH  -  render with vertex normals
 *            GLM_TEXTURE -  render with texture coords
 *            GLM_FLAT and GLM_SMOOTH should not both be specified.  
 */
GLuint
glmList(GLMmodel* model, GLuint mode);

/* glmWeld: eliminate (weld) vectors that are within an epsilon of
 * each other.
 *
 * model      - initialized GLMmodel structure
 * epsilon    - maximum difference between vertices
 *              ( 0.00001 is a good start for a unitized model)
 *
 */
GLuint
glmWeld(GLMmodel* model, GLfloat epsilon);



// ------------------------------------------------------------
// Declarations over, actual code... -_-
// ------------------------------------------------------------



#define T(x) (model->triangles[(x)])


/* _GLMnode: general purpose node  extern
 */
typedef struct _GLMnode {
  GLuint           index;
  GLboolean        averaged;
  struct _GLMnode* next;
} GLMnode;


/* glmMax: returns the maximum of two floats */
static GLfloat
glmMax(GLfloat a, GLfloat b) 
{
  if (b > a)
    return b;
  return a;
}

/* glmAbs: returns the absolute value of a float */
static GLfloat
glmAbs(GLfloat f)
{
  if (f < 0)
    return -f;
  return f;
}

/* glmDot: compute the dot product of two vectors
 *
 * u - array of 3 GLfloats (GLfloat u[3])
 * v - array of 3 GLfloats (GLfloat v[3])
 */
static GLfloat
glmDot(GLfloat* u, GLfloat* v)
{
  assert(u); assert(v);

  return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

/* glmCross: compute the cross product of two vectors
 *
 * u - array of 3 GLfloats (GLfloat u[3])
 * v - array of 3 GLfloats (GLfloat v[3])
 * n - array of 3 GLfloats (GLfloat n[3]) to return the cross product in
 */
static GLvoid
glmCross(GLfloat* u, GLfloat* v, GLfloat* n)
{
  assert(u); assert(v); assert(n);

  n[0] = u[1]*v[2] - u[2]*v[1];
  n[1] = u[2]*v[0] - u[0]*v[2];
  n[2] = u[0]*v[1] - u[1]*v[0];
}

/* glmNormalize: normalize a vector
 *
 * v - array of 3 GLfloats (GLfloat v[3]) to be normalized
 */
static GLvoid
glmNormalize(GLfloat* v)
{
  GLfloat l;

  assert(v);

  l = (GLfloat)sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  v[0] /= l;
  v[1] /= l;
  v[2] /= l;
}

/* glmEqual: compares two vectors and returns GL_TRUE if they are
 * equal (within a certain threshold) or GL_FALSE if not. An epsilon
 * that works fairly well is 0.000001.
 *
 * u - array of 3 GLfloats (GLfloat u[3])
 * v - array of 3 GLfloats (GLfloat v[3]) 
 */
static GLboolean
glmEqual(GLfloat* u, GLfloat* v, GLfloat epsilon)
{
  if (glmAbs(u[0] - v[0]) < epsilon &&
      glmAbs(u[1] - v[1]) < epsilon &&
      glmAbs(u[2] - v[2]) < epsilon) 
  {
    return GL_TRUE;
  }
  return GL_FALSE;
}

/* glmWeldVectors: eliminate (weld) vectors that are within an
 * epsilon of each other.
 *
 * vectors    - array of GLfloat[3]'s to be welded
 * numvectors - number of GLfloat[3]'s in vectors
 * epsilon    - maximum difference between vectors 
 *
 */
GLfloat*
glmWeldVectors(GLfloat* vectors, GLuint* numvectors, GLfloat epsilon)
{
  GLfloat* copies;
  GLuint   copied;
  GLuint   i, j;

  copies = (GLfloat*)malloc(sizeof(GLfloat) * 3 * (*numvectors + 1));
  memcpy(copies, vectors, (sizeof(GLfloat) * 3 * (*numvectors + 1)));

  copied = 1;
  for (i = 1; i <= *numvectors; i++) {
    for (j = 1; j <= copied; j++) {
      if (glmEqual(&vectors[3 * i], &copies[3 * j], epsilon)) {
	goto duplicate;
      }
    }

    /* must not be any duplicates -- add to the copies array */
    copies[3 * copied + 0] = vectors[3 * i + 0];
    copies[3 * copied + 1] = vectors[3 * i + 1];
    copies[3 * copied + 2] = vectors[3 * i + 2];
    j = copied;				/* pass this along for below */
    copied++;

  duplicate:
    /* set the first component of this vector to point at the correct
       index into the new copies array */
    vectors[3 * i + 0] = (GLfloat)j;
  }

  *numvectors = copied-1;
  return copies;
}

/* glmFindGroup: Find a group in the model
 */
GLMgroup*
glmFindGroup(GLMmodel* model, char* name)
{
  GLMgroup* group;

  assert(model);

  group = model->groups;
  while(group) {
    if (!strcmp(name, group->name))
      break;
    group = group->next;
  }

  return group;
}

/* glmAddGroup: Add a group to the model
 */
GLMgroup*
glmAddGroup(GLMmodel* model, char* name)
{
  GLMgroup* group;

  group = glmFindGroup(model, name);
  if (!group) {
    group = (GLMgroup*)malloc(sizeof(GLMgroup));
    group->name = strdup(name);
    group->material = 0;
    group->numtriangles = 0;
    group->triangles = NULL;
    group->next = model->groups;
    model->groups = group;
    model->numgroups++;
  }

  return group;
}

/* glmFindGroup: Find a material in the model
 */
GLuint
glmFindMaterial(GLMmodel* model, char* name)
{
  GLuint i;

  /* XXX doing a linear search on a string key'd list is pretty lame,
     but it works and is fast enough for now. */
  for (i = 0; i < model->nummaterials; i++) {
    if (!strcmp(model->materials[i].name, name))
      goto found;
  }

  /* didn't find the name, so print a warning and return the default
     material (0). */
  printf("glmFindMaterial():  can't find material \"%s\".\n", name);
  i = 0;

found:
  return i;
}


/* glmDirName: return the directory given a path
 *
 * path - filesystem path
 *
 * NOTE: the return value should be free'd.
 */
static char*
glmDirName(char* path)
{
  char* dir;
  char* s;

  dir = strdup(path);

  s = strrchr(dir, PATH_SEPARATOR_C);
  if (s)
    s[1] = '\0';
  else
    dir[0] = '\0';

  return dir;
}


/* glmReadMTL: read a wavefront material library file
 *
 * model - properly initialized GLMmodel structure
 * name  - name of the material library
 */
static GLvoid
glmReadMTL(GLMmodel* model, char* name)
{
  FILE* file;
  char* dir;
  char* filename;
  char  buf[128];
  GLuint nummaterials, i;

  dir = glmDirName(model->pathname);
  filename = (char*)malloc(sizeof(char) * (strlen(dir) + strlen(name) + 1));
  strcpy(filename, dir);
  strcat(filename, name);
  free(dir);

  file = fopen(filename, "r");
  if (!file) {
	  NAU_THROW("Failed to open OBJ material file: %s", filename);
//    fprintf(stderr, "glmReadMTL() failed: can't open material file \"%s\".\n",
//	    filename);
 //   exit(1);
  }
  free(filename);

  /* count the number of materials in the file */
  nummaterials = 1;
  while(fscanf(file, "%s", buf) != EOF) {
    switch(buf[0]) {
    case '#':				/* comment */
      /* eat up rest of line */
      fgets(buf, sizeof(buf), file);
      break;
    case 'n':				/* newmtl */
      fgets(buf, sizeof(buf), file);
      nummaterials++;
      sscanf(buf, "%s %s", buf, buf);
      break;
    default:
      /* eat up rest of line */
      fgets(buf, sizeof(buf), file);
      break;
    }
  }

  rewind(file);

  model->materials = (GLMmaterial*)malloc(sizeof(GLMmaterial) * nummaterials);
  model->nummaterials = nummaterials;

  /* set the default material */
  for (i = 0; i < nummaterials; i++) {
    model->materials[i].name = NULL;
    model->materials[i].shininess = 65.0f;
    model->materials[i].diffuse[0] = 0.8f;
    model->materials[i].diffuse[1] = 0.8f;
    model->materials[i].diffuse[2] = 0.8f;
    model->materials[i].diffuse[3] = 1.0f;
    model->materials[i].ambient[0] = 0.2f;
    model->materials[i].ambient[1] = 0.2f;
    model->materials[i].ambient[2] = 0.2f;
    model->materials[i].ambient[3] = 1.0f;
    model->materials[i].specular[0] = 0.0f;
    model->materials[i].specular[1] = 0.0f;
    model->materials[i].specular[2] = 0.0f;
    model->materials[i].specular[3] = 1.0f;
    model->materials[i].emmissive[0] = 0.0f;
    model->materials[i].emmissive[1] = 0.0f;
    model->materials[i].emmissive[2] = 0.0f;
    model->materials[i].emmissive[3] = 1.0f;
	model->materials[i].texture = NULL; //new std::string("");
  }
  model->materials[0].name = strdup("Default3DSMat");
  

  /* now, read in the data */
  nummaterials = 0;
  while(fscanf(file, "%s", buf) != EOF) {
    switch(buf[0]) {
    case '#':				/* comment */
      /* eat up rest of line */
      fgets(buf, sizeof(buf), file);
      break;
    case 'n':				/* newmtl */
      fgets(buf, sizeof(buf), file);
      sscanf(buf, "%s %s", buf, buf);
      nummaterials++;
      model->materials[nummaterials].name = strdup(buf);
      break;
	case 'm' :				/* map_Kd */
		dir = glmDirName(model->pathname);
		char  buf2[1024];
		fgetc(file);
		fgets(buf2, sizeof(buf2), file);
		model->materials[nummaterials].texture = (char*)malloc(sizeof(char) * (strlen(dir) + strlen(buf2) + 1));
		model->materials[nummaterials].texture[0] = '\0';
		strcat(model->materials[nummaterials].texture, dir);
		strcat(model->materials[nummaterials].texture, buf2);
		model->materials[nummaterials].texture[strlen(model->materials[nummaterials].texture)-1] = '\0';
		//model->materials[nummaterials].texture = strdup(buf2);
		free(dir);
		break;
    case 'N':
      fscanf(file, "%f", &model->materials[nummaterials].shininess);
      /* wavefront shininess is from [0, 1000], so scale for OpenGL */
      model->materials[nummaterials].shininess /= 1000.0;
      model->materials[nummaterials].shininess *= 128.0;
      break;
    case 'K':
      switch(buf[1]) {
      case 'd':
	fscanf(file, "%f %f %f",
	       &model->materials[nummaterials].diffuse[0],
	       &model->materials[nummaterials].diffuse[1],
	       &model->materials[nummaterials].diffuse[2]);
	break;
      case 's':
	fscanf(file, "%f %f %f",
	       &model->materials[nummaterials].specular[0],
	       &model->materials[nummaterials].specular[1],
	       &model->materials[nummaterials].specular[2]);
	break;
      case 'a':
	fscanf(file, "%f %f %f",
	       &model->materials[nummaterials].ambient[0],
	       &model->materials[nummaterials].ambient[1],
	       &model->materials[nummaterials].ambient[2]);
	break;
      default:
	/* eat up rest of line */
	fgets(buf, sizeof(buf), file);
	break;
      }
      break;
    default:
      /* eat up rest of line */
      fgets(buf, sizeof(buf), file);
      break;
    }
  }
}

/* glmWriteMTL: write a wavefront material library file
 *
 * model      - properly initialized GLMmodel structure
 * modelpath  - pathname of the model being written
 * mtllibname - name of the material library to be written
 */
static GLvoid
glmWriteMTL(GLMmodel* model, char* modelpath, char* mtllibname)
{
  FILE* file;
  char* dir;
  char* filename;
  GLMmaterial* material;
  GLuint i;

  dir = glmDirName(modelpath);
  filename = (char*)malloc(sizeof(char) * (strlen(dir)+strlen(mtllibname)));
  strcpy(filename, dir);
  strcat(filename, mtllibname);
  free(dir);

  /* open the file */
  file = fopen(filename, "w");
  if (!file) {
    fprintf(stderr, "glmWriteMTL() failed: can't open file \"%s\".\n",
	    filename);
    exit(1);
  }
  free(filename);

  /* spit out a header */
  fprintf(file, "#  \n");
  fprintf(file, "#  Wavefront MTL generated by GLM library\n");
  fprintf(file, "#  \n");
  fprintf(file, "#  GLM library\n");
  fprintf(file, "#  Nate Robins\n");
  fprintf(file, "#  ndr@pobox.com\n");
  fprintf(file, "#  http://www.pobox.com/~ndr\n");
  fprintf(file, "#  \n\n");

  for (i = 0; i < model->nummaterials; i++) {
    material = &model->materials[i];
    fprintf(file, "newmtl %s\n", material->name);
    fprintf(file, "Ka %f %f %f\n", 
	    material->ambient[0], material->ambient[1], material->ambient[2]);
    fprintf(file, "Kd %f %f %f\n", 
	    material->diffuse[0], material->diffuse[1], material->diffuse[2]);
    fprintf(file, "Ks %f %f %f\n", 
	    material->specular[0],material->specular[1],material->specular[2]);
    fprintf(file, "Ns %f\n", material->shininess / 128.0 * 1000.0);
    fprintf(file, "\n");
  }
}


/* glmFirstPass: first pass at a Wavefront OBJ file that gets all the
 * statistics of the model (such as #vertices, #normals, etc)
 *
 * model - properly initialized GLMmodel structure
 * file  - (fopen'd) file descriptor 
 */
static GLvoid
glmFirstPass(GLMmodel* model, FILE* file) 
{
  GLuint    numvertices;		/* number of vertices in model */
  GLuint    numnormals;			/* number of normals in model */
  GLuint    numtexcoords;		/* number of texcoords in model */
  GLuint    numtriangles;		/* number of triangles in model */
  GLMgroup* group;			/* current group */
  unsigned  v, n, t;
  char      buf[128];

  /* make a default group */
  group = glmAddGroup(model, "default");

  numvertices = numnormals = numtexcoords = numtriangles = 0;
  while(fscanf(file, "%s", buf) != EOF) {
    switch(buf[0]) {
    case '#':				/* comment */
      /* eat up rest of line */
      fgets(buf, sizeof(buf), file);
      break;
    case 'v':				/* v, vn, vt */
      switch(buf[1]) {
      case '\0':			/* vertex */
	/* eat up rest of line */
	fgets(buf, sizeof(buf), file);
	numvertices++;
	break;
      case 'n':				/* normal */
	/* eat up rest of line */
	fgets(buf, sizeof(buf), file);
	numnormals++;
	break;
      case 't':				/* texcoord */
	/* eat up rest of line */
	fgets(buf, sizeof(buf), file);
	numtexcoords++;
	break;
      default:
	printf("glmFirstPass(): Unknown token \"%s\".\n", buf);
	exit(1);
	break;
      }
      break;
    case 'm':
      fgets(buf, sizeof(buf), file);
      sscanf(buf, "%s %s", buf, buf);
      model->mtllibname = strdup(buf);
      glmReadMTL(model, buf);
      break;
    case 'u':
      /* eat up rest of line */
      fgets(buf, sizeof(buf), file);
      break;
    case 'g':				/* group */
      /* eat up rest of line */
      fgets(buf, sizeof(buf), file);
#if SINGLE_STRING_GROUP_NAMES
      sscanf(buf, "%s", buf);
#else
      buf[strlen(buf)-1] = '\0';	/* nuke '\n' */
#endif
      group = glmAddGroup(model, buf);
      break;
    case 'f':				/* face */
      v = n = t = 0;
      fscanf(file, "%s", buf);
      /* can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d */
      if (strstr(buf, "//")) {
	/* v//n */
	sscanf(buf, "%d//%d", &v, &n);
	fscanf(file, "%d//%d", &v, &n);
	fscanf(file, "%d//%d", &v, &n);
	numtriangles++;
	group->numtriangles++;
	while(fscanf(file, "%d//%d", &v, &n) > 0) {
	  numtriangles++;
	  group->numtriangles++;
	}
      } else if (sscanf(buf, "%d/%d/%d", &v, &t, &n) == 3) {
	/* v/t/n */
	fscanf(file, "%d/%d/%d", &v, &t, &n);
	fscanf(file, "%d/%d/%d", &v, &t, &n);
	numtriangles++;
	group->numtriangles++;
	while(fscanf(file, "%d/%d/%d", &v, &t, &n) > 0) {
	  numtriangles++;
	  group->numtriangles++;
	}
      } else if (sscanf(buf, "%d/%d", &v, &t) == 2) {
	/* v/t */
	fscanf(file, "%d/%d", &v, &t);
	fscanf(file, "%d/%d", &v, &t);
	numtriangles++;
	group->numtriangles++;
	while(fscanf(file, "%d/%d", &v, &t) > 0) {
	  numtriangles++;
	  group->numtriangles++;
	}
      } else {
	/* v */
	fscanf(file, "%d", &v);
	fscanf(file, "%d", &v);
	numtriangles++;
	group->numtriangles++;
	while(fscanf(file, "%d", &v) > 0) {
	  numtriangles++;
	  group->numtriangles++;
	}
      }
      break;

    default:
      /* eat up rest of line */
      fgets(buf, sizeof(buf), file);
      break;
    }
  }

  /* set the stats in the model structure */
  model->numvertices  = numvertices;
  model->numnormals   = numnormals;
  model->numtexcoords = numtexcoords;
  model->numtriangles = numtriangles;

  /* allocate memory for the triangles in each group */
  group = model->groups;
  while(group) {
    group->triangles = (GLuint*)malloc(sizeof(GLuint) * group->numtriangles);
    group->numtriangles = 0;
    group = group->next;
  }
}

/* glmSecondPass: second pass at a Wavefront OBJ file that gets all
 * the data.
 *
 * model - properly initialized GLMmodel structure
 * file  - (fopen'd) file descriptor 
 */
static GLvoid
glmSecondPass(GLMmodel* model, FILE* file) 
{
  GLuint    numvertices;		/* number of vertices in model */
  GLuint    numnormals;			/* number of normals in model */
  GLuint    numtexcoords;		/* number of texcoords in model */
  GLuint    numtriangles;		/* number of triangles in model */
  GLfloat*  vertices;			/* array of vertices  */
  GLfloat*  normals;			/* array of normals */
  GLfloat*  texcoords;			/* array of texture coordinates */
  GLMgroup* group;			/* current group pointer */
  GLuint    material;			/* current material */
  GLuint    v, n, t;
  char      buf[128];

  /* set the pointer shortcuts */
  vertices     = model->vertices;
  normals      = model->normals;
  texcoords    = model->texcoords;
  group        = model->groups;

  /* on the second pass through the file, read all the data into the
     allocated arrays */
  numvertices = numnormals = numtexcoords = 1;
  numtriangles = 0;
  material = 0;
  while(fscanf(file, "%s", buf) != EOF) {
    switch(buf[0]) {
    case '#':				/* comment */
      /* eat up rest of line */
      fgets(buf, sizeof(buf), file);
      break;
    case 'v':				/* v, vn, vt */
      switch(buf[1]) {
      case '\0':			/* vertex */
	fscanf(file, "%f %f %f", 
	       &vertices[3 * numvertices + 0], 
	       &vertices[3 * numvertices + 1], 
	       &vertices[3 * numvertices + 2]);
	numvertices++;
	break;
      case 'n':				/* normal */
	fscanf(file, "%f %f %f", 
	       &normals[3 * numnormals + 0],
	       &normals[3 * numnormals + 1], 
	       &normals[3 * numnormals + 2]);
	numnormals++;
	break;
      case 't':				/* texcoord */
	fscanf(file, "%f %f", 
	       &texcoords[2 * numtexcoords + 0],
	       &texcoords[2 * numtexcoords + 1]);
	numtexcoords++;
	break;
      }
      break;
    case 'u':
      fgets(buf, sizeof(buf), file);
      sscanf(buf, "%s %s", buf, buf);
      group->material = material = glmFindMaterial(model, buf);
      break;
    case 'g':				/* group */
      /* eat up rest of line */
      fgets(buf, sizeof(buf), file);
#if SINGLE_STRING_GROUP_NAMES
      sscanf(buf, "%s", buf);
#else
      buf[strlen(buf)-1] = '\0';	/* nuke '\n' */
#endif
      group = glmFindGroup(model, buf);
      group->material = material;
      break;
    case 'f':				/* face */
      v = n = t = 0;
      fscanf(file, "%s", buf);
      /* can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d */
      if (strstr(buf, "//")) {
	/* v//n */
	sscanf(buf, "%d//%d", &v, &n);
	T(numtriangles).vindices[0] = v;
	T(numtriangles).nindices[0] = n;
	fscanf(file, "%d//%d", &v, &n);
	T(numtriangles).vindices[1] = v;
	T(numtriangles).nindices[1] = n;
	fscanf(file, "%d//%d", &v, &n);
	T(numtriangles).vindices[2] = v;
	T(numtriangles).nindices[2] = n;
	group->triangles[group->numtriangles++] = numtriangles;
	numtriangles++;
	while(fscanf(file, "%d//%d", &v, &n) > 0) {
	  T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
	  T(numtriangles).nindices[0] = T(numtriangles-1).nindices[0];
	  T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
	  T(numtriangles).nindices[1] = T(numtriangles-1).nindices[2];
	  T(numtriangles).vindices[2] = v;
	  T(numtriangles).nindices[2] = n;
	  group->triangles[group->numtriangles++] = numtriangles;
	  numtriangles++;
	}
      } else if (sscanf(buf, "%d/%d/%d", &v, &t, &n) == 3) {
	/* v/t/n */
	T(numtriangles).vindices[0] = v;
	T(numtriangles).tindices[0] = t;
	T(numtriangles).nindices[0] = n;
	fscanf(file, "%d/%d/%d", &v, &t, &n);
	T(numtriangles).vindices[1] = v;
	T(numtriangles).tindices[1] = t;
	T(numtriangles).nindices[1] = n;
	fscanf(file, "%d/%d/%d", &v, &t, &n);
	T(numtriangles).vindices[2] = v;
	T(numtriangles).tindices[2] = t;
	T(numtriangles).nindices[2] = n;
	group->triangles[group->numtriangles++] = numtriangles;
	numtriangles++;
	while(fscanf(file, "%d/%d/%d", &v, &t, &n) > 0) {
	  T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
	  T(numtriangles).tindices[0] = T(numtriangles-1).tindices[0];
	  T(numtriangles).nindices[0] = T(numtriangles-1).nindices[0];
	  T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
	  T(numtriangles).tindices[1] = T(numtriangles-1).tindices[2];
	  T(numtriangles).nindices[1] = T(numtriangles-1).nindices[2];
	  T(numtriangles).vindices[2] = v;
	  T(numtriangles).tindices[2] = t;
	  T(numtriangles).nindices[2] = n;
	  group->triangles[group->numtriangles++] = numtriangles;
	  numtriangles++;
	}
      } else if (sscanf(buf, "%d/%d", &v, &t) == 2) {
	/* v/t */
	T(numtriangles).vindices[0] = v;
	T(numtriangles).tindices[0] = t;
	fscanf(file, "%d/%d", &v, &t);
	T(numtriangles).vindices[1] = v;
	T(numtriangles).tindices[1] = t;
	fscanf(file, "%d/%d", &v, &t);
	T(numtriangles).vindices[2] = v;
	T(numtriangles).tindices[2] = t;
	group->triangles[group->numtriangles++] = numtriangles;
	numtriangles++;
	while(fscanf(file, "%d/%d", &v, &t) > 0) {
	  T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
	  T(numtriangles).tindices[0] = T(numtriangles-1).tindices[0];
	  T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
	  T(numtriangles).tindices[1] = T(numtriangles-1).tindices[2];
	  T(numtriangles).vindices[2] = v;
	  T(numtriangles).tindices[2] = t;
	  group->triangles[group->numtriangles++] = numtriangles;
	  numtriangles++;
	}
      } else {
	/* v */
	sscanf(buf, "%d", &v);
	T(numtriangles).vindices[0] = v;
	fscanf(file, "%d", &v);
	T(numtriangles).vindices[1] = v;
	fscanf(file, "%d", &v);
	T(numtriangles).vindices[2] = v;
	group->triangles[group->numtriangles++] = numtriangles;
	numtriangles++;
	while(fscanf(file, "%d", &v) > 0) {
	  T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
	  T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
	  T(numtriangles).vindices[2] = v;
	  group->triangles[group->numtriangles++] = numtriangles;
	  numtriangles++;
	}
      }
      break;

    default:
      /* eat up rest of line */
      fgets(buf, sizeof(buf), file);
      break;
    }
  }

#if 0
  /* announce the memory requirements */
  printf(" Memory: %d bytes\n",
	 numvertices  * 3*sizeof(GLfloat) +
	 numnormals   * 3*sizeof(GLfloat) * (numnormals ? 1 : 0) +
	 numtexcoords * 3*sizeof(GLfloat) * (numtexcoords ? 1 : 0) +
	 numtriangles * sizeof(GLMtriangle));
#endif
}




/* glmUnitize: "unitize" a model by translating it to the origin and
 * scaling it to fit in a unit cube around the origin (-1 to 1 in all
 * dimensions).  
 * Returns the scalefactor used.
 *
 * model - properly initialized GLMmodel structure 
 */
GLfloat
glmUnitize(GLMmodel* model)
{
  GLuint  i;
  GLfloat maxx, minx, maxy, miny, maxz, minz;
  GLfloat cx, cy, cz, w, h, d;
  GLfloat scale;

  assert(model);
  assert(model->vertices);

  /* get the max/mins */
  maxx = minx = model->vertices[3 + 0];
  maxy = miny = model->vertices[3 + 1];
  maxz = minz = model->vertices[3 + 2];
  for (i = 1; i <= model->numvertices; i++) {
    if (maxx < model->vertices[3 * i + 0])
      maxx = model->vertices[3 * i + 0];
    if (minx > model->vertices[3 * i + 0])
      minx = model->vertices[3 * i + 0];

    if (maxy < model->vertices[3 * i + 1])
      maxy = model->vertices[3 * i + 1];
    if (miny > model->vertices[3 * i + 1])
      miny = model->vertices[3 * i + 1];

    if (maxz < model->vertices[3 * i + 2])
      maxz = model->vertices[3 * i + 2];
    if (minz > model->vertices[3 * i + 2])
      minz = model->vertices[3 * i + 2];
  }

  /* calculate model width, height, and depth */
  w = glmAbs(maxx) + glmAbs(minx);
  h = glmAbs(maxy) + glmAbs(miny);
  d = glmAbs(maxz) + glmAbs(minz);

  /* calculate center of the model */
  cx = (maxx + minx) / 2.0;
  cy = (maxy + miny) / 2.0;
  cz = (maxz + minz) / 2.0;

  /* calculate unitizing scale factor */
  scale = 2.0 / glmMax(glmMax(w, h), d);

  /* translate around center then scale */
  for (i = 1; i <= model->numvertices; i++) {
    model->vertices[3 * i + 0] -= cx;
    model->vertices[3 * i + 1] -= cy;
    model->vertices[3 * i + 2] -= cz;
    model->vertices[3 * i + 0] *= scale;
    model->vertices[3 * i + 1] *= scale;
    model->vertices[3 * i + 2] *= scale;
  }

  return scale;
}

/* glmDimensions: Calculates the dimensions (width, height, depth) of
 * a model.
 *
 * model      - initialized GLMmodel structure
 * dimensions - array of 3 GLfloats (GLfloat dimensions[3])
 */
GLvoid
glmDimensions(GLMmodel* model, GLfloat* dimensions)
{
  GLuint i;
  GLfloat maxx, minx, maxy, miny, maxz, minz;

  assert(model);
  assert(model->vertices);
  assert(dimensions);

  /* get the max/mins */
  maxx = minx = model->vertices[3 + 0];
  maxy = miny = model->vertices[3 + 1];
  maxz = minz = model->vertices[3 + 2];
  for (i = 1; i <= model->numvertices; i++) {
    if (maxx < model->vertices[3 * i + 0])
      maxx = model->vertices[3 * i + 0];
    if (minx > model->vertices[3 * i + 0])
      minx = model->vertices[3 * i + 0];

    if (maxy < model->vertices[3 * i + 1])
      maxy = model->vertices[3 * i + 1];
    if (miny > model->vertices[3 * i + 1])
      miny = model->vertices[3 * i + 1];

    if (maxz < model->vertices[3 * i + 2])
      maxz = model->vertices[3 * i + 2];
    if (minz > model->vertices[3 * i + 2])
      minz = model->vertices[3 * i + 2];
  }

  /* calculate model width, height, and depth */
  dimensions[0] = glmAbs(maxx) + glmAbs(minx);
  dimensions[1] = glmAbs(maxy) + glmAbs(miny);
  dimensions[2] = glmAbs(maxz) + glmAbs(minz);
}

/* glmScale: Scales a model by a given amount.
 * 
 * model - properly initialized GLMmodel structure
 * scale - scalefactor (0.5 = half as large, 2.0 = twice as large)
 */
GLvoid
glmScale(GLMmodel* model, GLfloat scale)
{
  GLuint i;

  for (i = 1; i <= model->numvertices; i++) {
    model->vertices[3 * i + 0] *= scale;
    model->vertices[3 * i + 1] *= scale;
    model->vertices[3 * i + 2] *= scale;
  }
}

/* glmReverseWinding: Reverse the polygon winding for all polygons in
 * this model.  Default winding is counter-clockwise.  Also changes
 * the direction of the normals.
 * 
 * model - properly initialized GLMmodel structure 
 */
GLvoid
glmReverseWinding(GLMmodel* model)
{
  GLuint i, swap;

  assert(model);

  for (i = 0; i < model->numtriangles; i++) {
    swap = T(i).vindices[0];
    T(i).vindices[0] = T(i).vindices[2];
    T(i).vindices[2] = swap;

    if (model->numnormals) {
      swap = T(i).nindices[0];
      T(i).nindices[0] = T(i).nindices[2];
      T(i).nindices[2] = swap;
    }

    if (model->numtexcoords) {
      swap = T(i).tindices[0];
      T(i).tindices[0] = T(i).tindices[2];
      T(i).tindices[2] = swap;
    }
  }

  /* reverse facet normals */
  for (i = 1; i <= model->numfacetnorms; i++) {
    model->facetnorms[3 * i + 0] = -model->facetnorms[3 * i + 0];
    model->facetnorms[3 * i + 1] = -model->facetnorms[3 * i + 1];
    model->facetnorms[3 * i + 2] = -model->facetnorms[3 * i + 2];
  }

  /* reverse vertex normals */
  for (i = 1; i <= model->numnormals; i++) {
    model->normals[3 * i + 0] = -model->normals[3 * i + 0];
    model->normals[3 * i + 1] = -model->normals[3 * i + 1];
    model->normals[3 * i + 2] = -model->normals[3 * i + 2];
  }
}

/* glmFacetNormals: Generates facet normals for a model (by taking the
 * cross product of the two vectors derived from the sides of each
 * triangle).  Assumes a counter-clockwise winding.
 *
 * model - initialized GLMmodel structure
 */
GLvoid
glmFacetNormals(GLMmodel* model)
{
  GLuint  i;
  GLfloat u[3];
  GLfloat v[3];
  
  assert(model);
  assert(model->vertices);

  /* clobber any old facetnormals */
  if (model->facetnorms)
    free(model->facetnorms);

  /* allocate memory for the new facet normals */
  model->numfacetnorms = model->numtriangles;
  model->facetnorms = (GLfloat*)malloc(sizeof(GLfloat) *
				       3 * (model->numfacetnorms + 1));

  for (i = 0; i < model->numtriangles; i++) {
    model->triangles[i].findex = i+1;

    u[0] = model->vertices[3 * T(i).vindices[1] + 0] -
           model->vertices[3 * T(i).vindices[0] + 0];
    u[1] = model->vertices[3 * T(i).vindices[1] + 1] -
           model->vertices[3 * T(i).vindices[0] + 1];
    u[2] = model->vertices[3 * T(i).vindices[1] + 2] -
           model->vertices[3 * T(i).vindices[0] + 2];

    v[0] = model->vertices[3 * T(i).vindices[2] + 0] -
           model->vertices[3 * T(i).vindices[0] + 0];
    v[1] = model->vertices[3 * T(i).vindices[2] + 1] -
           model->vertices[3 * T(i).vindices[0] + 1];
    v[2] = model->vertices[3 * T(i).vindices[2] + 2] -
           model->vertices[3 * T(i).vindices[0] + 2];

    glmCross(u, v, &model->facetnorms[3 * (i+1)]);
    glmNormalize(&model->facetnorms[3 * (i+1)]);
  }
}

/* glmVertexNormals: Generates smooth vertex normals for a model.
 * First builds a list of all the triangles each vertex is in.  Then
 * loops through each vertex in the the list averaging all the facet
 * normals of the triangles each vertex is in.  Finally, sets the
 * normal index in the triangle for the vertex to the generated smooth
 * normal.  If the dot product of a facet normal and the facet normal
 * associated with the first triangle in the list of triangles the
 * current vertex is in is greater than the cosine of the angle
 * parameter to the function, that facet normal is not added into the
 * average normal calculation and the corresponding vertex is given
 * the facet normal.  This tends to preserve hard edges.  The angle to
 * use depends on the model, but 90 degrees is usually a good start.
 *
 * model - initialized GLMmodel structure
 * angle - maximum angle (in degrees) to smooth across
 */

GLvoid
glmVertexNormals(GLMmodel* model, GLfloat angle)
{
  GLMnode*  node;
  GLMnode*  tail;
  GLMnode** members;
  GLfloat*  normals;
  GLuint    numnormals;
  GLfloat   average[3];
  GLfloat   dot, cos_angle;
  GLuint    i, avg;

  assert(model);
  assert(model->facetnorms);

  /* calculate the cosine of the angle (in degrees) */
  cos_angle = cos(angle * M_PI / 180.0);

  /* nuke any previous normals */
  if (model->normals)
    free(model->normals);

  /* allocate space for new normals */
  model->numnormals = model->numtriangles * 3; /* 3 normals per triangle */
  model->normals = (GLfloat*)malloc(sizeof(GLfloat)* 3* (model->numnormals+1));

  /* allocate a structure that will hold a linked list of triangle
     indices for each vertex */
  members = (GLMnode**)malloc(sizeof(GLMnode*) * (model->numvertices + 1));
  for (i = 1; i <= model->numvertices; i++)
    members[i] = NULL;
  
  /* for every triangle, create a node for each vertex in it */
  for (i = 0; i < model->numtriangles; i++) {
    node = (GLMnode*)malloc(sizeof(GLMnode));
    node->index = i;
    node->next  = members[T(i).vindices[0]];
    members[T(i).vindices[0]] = node;

    node = (GLMnode*)malloc(sizeof(GLMnode));
    node->index = i;
    node->next  = members[T(i).vindices[1]];
    members[T(i).vindices[1]] = node;

    node = (GLMnode*)malloc(sizeof(GLMnode));
    node->index = i;
    node->next  = members[T(i).vindices[2]];
    members[T(i).vindices[2]] = node;
  }

  /* calculate the average normal for each vertex */
  numnormals = 1;
  for (i = 1; i <= model->numvertices; i++) {
    /* calculate an average normal for this vertex by averaging the
       facet normal of every triangle this vertex is in */
    node = members[i];


	//Change made in this file (I put this two lines in commentary)
    //if (!node)
    //  fprintf(stderr, "glmVertexNormals(): vertex w/o a triangle\n");
    
	
	average[0] = 0.0; average[1] = 0.0; average[2] = 0.0;
    avg = 0;
    while (node) {
      /* only average if the dot product of the angle between the two
         facet normals is greater than the cosine of the threshold
         angle -- or, said another way, the angle between the two
         facet normals is less than (or equal to) the threshold angle */
      dot = glmDot(&model->facetnorms[3 * T(node->index).findex],
 		    &model->facetnorms[3 * T(members[i]->index).findex]);
      if (dot > cos_angle) {
	node->averaged = GL_TRUE;
	average[0] += model->facetnorms[3 * T(node->index).findex + 0];
	average[1] += model->facetnorms[3 * T(node->index).findex + 1];
	average[2] += model->facetnorms[3 * T(node->index).findex + 2];
	avg = 1;			/* we averaged at least one normal! */
      } else {
	node->averaged = GL_FALSE;
      }
      node = node->next;
    }

    if (avg) {
      /* normalize the averaged normal */
      glmNormalize(average);

      /* add the normal to the vertex normals list */
      model->normals[3 * numnormals + 0] = average[0];
      model->normals[3 * numnormals + 1] = average[1];
      model->normals[3 * numnormals + 2] = average[2];
      avg = numnormals;
      numnormals++;
    }

    /* set the normal of this vertex in each triangle it is in */
    node = members[i];
    while (node) {
      if (node->averaged) {
	/* if this node was averaged, use the average normal */
	if (T(node->index).vindices[0] == i)
	  T(node->index).nindices[0] = avg;
	else if (T(node->index).vindices[1] == i)
	  T(node->index).nindices[1] = avg;
	else if (T(node->index).vindices[2] == i)
	  T(node->index).nindices[2] = avg;
      } else {
	/* if this node wasn't averaged, use the facet normal */
	model->normals[3 * numnormals + 0] = 
	  model->facetnorms[3 * T(node->index).findex + 0];
	model->normals[3 * numnormals + 1] = 
	  model->facetnorms[3 * T(node->index).findex + 1];
	model->normals[3 * numnormals + 2] = 
	  model->facetnorms[3 * T(node->index).findex + 2];
	if (T(node->index).vindices[0] == i)
	  T(node->index).nindices[0] = numnormals;
	else if (T(node->index).vindices[1] == i)
	  T(node->index).nindices[1] = numnormals;
	else if (T(node->index).vindices[2] == i)
	  T(node->index).nindices[2] = numnormals;
	numnormals++;
      }
      node = node->next;
    }
  }
  
  model->numnormals = numnormals - 1;

  /* free the member information */
  for (i = 1; i <= model->numvertices; i++) {
    node = members[i];
    while (node) {
      tail = node;
      node = node->next;
      free(tail);
    }
  }
//  free(members);

  /* pack the normals array (we previously allocated the maximum
     number of normals that could possibly be created (numtriangles *
     3), so get rid of some of them (usually alot unless none of the
     facet normals were averaged)) */
  normals = model->normals;
  model->normals = (GLfloat*)malloc(sizeof(GLfloat)* 3* (model->numnormals+1));
  for (i = 1; i <= model->numnormals; i++) {
    model->normals[3 * i + 0] = normals[3 * i + 0];
    model->normals[3 * i + 1] = normals[3 * i + 1];
    model->normals[3 * i + 2] = normals[3 * i + 2];
  }
  free(normals);
}


/* glmLinearTexture: Generates texture coordinates according to a
 * linear projection of the texture map.  It generates these by
 * linearly mapping the vertices onto a square.
 *
 * model - pointer to initialized GLMmodel structure
 */
GLvoid
glmLinearTexture(GLMmodel* model)
{
  GLMgroup *group;
  GLfloat dimensions[3];
  GLfloat x, y, scalefactor;
  GLuint i;
  
  assert(model);

  if (model->texcoords)
    free(model->texcoords);
  model->numtexcoords = model->numvertices;
  model->texcoords=(GLfloat*)malloc(sizeof(GLfloat)*2*(model->numtexcoords+1));
  
  glmDimensions(model, dimensions);
  scalefactor = 2.0 / 
    glmAbs(glmMax(glmMax(dimensions[0], dimensions[1]), dimensions[2]));

  /* do the calculations */
  for(i = 1; i <= model->numvertices; i++) {
    x = model->vertices[3 * i + 0] * scalefactor;
    y = model->vertices[3 * i + 2] * scalefactor;
    model->texcoords[2 * i + 0] = (x + 1.0) / 2.0;
    model->texcoords[2 * i + 1] = (y + 1.0) / 2.0;
  }
  
  /* go through and put texture coordinate indices in all the triangles */
  group = model->groups;
  while(group) {
    for(i = 0; i < group->numtriangles; i++) {
      T(group->triangles[i]).tindices[0] = T(group->triangles[i]).vindices[0];
      T(group->triangles[i]).tindices[1] = T(group->triangles[i]).vindices[1];
      T(group->triangles[i]).tindices[2] = T(group->triangles[i]).vindices[2];
    }    
    group = group->next;
  }

#if 0
  printf("glmLinearTexture(): generated %d linear texture coordinates\n",
	  model->numtexcoords);
#endif
}

/* glmSpheremapTexture: Generates texture coordinates according to a
 * spherical projection of the texture map.  Sometimes referred to as
 * spheremap, or reflection map texture coordinates.  It generates
 * these by using the normal to calculate where that vertex would map
 * onto a sphere.  Since it is impossible to map something flat
 * perfectly onto something spherical, there is distortion at the
 * poles.  This particular implementation causes the poles along the X
 * axis to be distorted.
 *
 * model - pointer to initialized GLMmodel structure
 */
GLvoid
glmSpheremapTexture(GLMmodel* model)
{
  GLMgroup* group;
  GLfloat theta, phi, rho, x, y, z, r;
  GLuint i;
  
  assert(model);
  assert(model->normals);

  if (model->texcoords)
    free(model->texcoords);
  model->numtexcoords = model->numnormals;
  model->texcoords=(GLfloat*)malloc(sizeof(GLfloat)*2*(model->numtexcoords+1));
     
  for (i = 1; i <= model->numnormals; i++) {
    z = model->normals[3 * i + 0];	/* re-arrange for pole distortion */
    y = model->normals[3 * i + 1];
    x = model->normals[3 * i + 2];
    r = sqrt((x * x) + (y * y));
    rho = sqrt((r * r) + (z * z));
      
    if(r == 0.0f) {
	theta = 0.0f;
	phi = 0.0f;
    } else {
      if(z == 0.0f)
	phi = 3.14159265f / 2.0f;
      else
	phi = acos(z / rho);

      if(y == 0.0f)
	theta = 3.141592365f / 2.0f;
      else
	theta = asin(y / r) + (3.14159265f / 2.0f);
    }
    
    model->texcoords[2 * i + 0] = theta / 3.14159265f;
    model->texcoords[2 * i + 1] = phi / 3.14159265f;
  }
  
  /* go through and put texcoord indices in all the triangles */
  group = model->groups;
  while(group) {
    for (i = 0; i < group->numtriangles; i++) {
      T(group->triangles[i]).tindices[0] = T(group->triangles[i]).nindices[0];
      T(group->triangles[i]).tindices[1] = T(group->triangles[i]).nindices[1];
      T(group->triangles[i]).tindices[2] = T(group->triangles[i]).nindices[2];
    }
    group = group->next;
  }
}

/* glmDelete: Deletes a GLMmodel structure.
 *
 * model - initialized GLMmodel structure
 */
GLvoid
glmDelete(GLMmodel* model)
{
  GLMgroup* group;
  GLuint i;

  assert(model);

  if (model->pathname)   free(model->pathname);
  if (model->mtllibname) free(model->mtllibname);
  if (model->vertices)   free(model->vertices);
  if (model->normals)    free(model->normals);
  if (model->texcoords)  free(model->texcoords);
  if (model->facetnorms) free(model->facetnorms);
  if (model->triangles)  free(model->triangles);
  if (model->materials) {
    for (i = 0; i < model->nummaterials; i++)
      free(model->materials[i].name);
  }
  free(model->materials);
  while(model->groups) {
    group = model->groups;
    model->groups = model->groups->next;
    free(group->name);
    free(group->triangles);
    free(group);
  }

  free(model);
}

/* glmReadOBJ: Reads a model description from a Wavefront .OBJ file.
 * Returns a pointer to the created object which should be free'd with
 * glmDelete().
 *
 * filename - name of the file containing the Wavefront .OBJ format data.  
 */
GLMmodel* 
glmReadOBJ(const char* filename)
{
  GLMmodel* model;
  FILE*     file;

  /* open the file */
  file = fopen(filename, "r");
  if (!file) {
    fprintf(stderr, "glmReadOBJ() failed: can't open data file \"%s\".\n",
	    filename);
    exit(1);
  }

  /* allocate a new model */
  model = (GLMmodel*)malloc(sizeof(GLMmodel));
  model->pathname      = strdup(filename);
  model->mtllibname    = NULL;
  model->numvertices   = 0;
  model->vertices      = NULL;
  model->numnormals    = 0;
  model->normals       = NULL;
  model->numtexcoords  = 0;
  model->texcoords     = NULL;
  model->numfacetnorms = 0;
  model->facetnorms    = NULL;
  model->numtriangles  = 0;
  model->triangles     = NULL;
  model->nummaterials  = 0;
  model->materials     = NULL;
  model->numgroups     = 0;
  model->groups        = NULL;
  model->position[0]   = 0.0;
  model->position[1]   = 0.0;
  model->position[2]   = 0.0;

  /* make a first pass through the file to get a count of the number
     of vertices, normals, texcoords & triangles */
  glmFirstPass(model, file);

  /* allocate memory */
  model->vertices = (GLfloat*)malloc(sizeof(GLfloat) *
				     3 * (model->numvertices + 1));
  model->triangles = (GLMtriangle*)malloc(sizeof(GLMtriangle) *
					  model->numtriangles);
  if (model->numnormals) {
    model->normals = (GLfloat*)malloc(sizeof(GLfloat) *
				      3 * (model->numnormals + 1));
  }
  if (model->numtexcoords) {
    model->texcoords = (GLfloat*)malloc(sizeof(GLfloat) *
					2 * (model->numtexcoords + 1));
  }

  /* rewind to beginning of file and read in the data this pass */
  rewind(file);

  glmSecondPass(model, file);

  /* close the file */
  fclose(file);

  return model;
}

/* glmWriteOBJ: Writes a model description in Wavefront .OBJ format to
 * a file.
 *
 * model    - initialized GLMmodel structure
 * filename - name of the file to write the Wavefront .OBJ format data to
 * mode     - a bitwise or of values describing what is written to the file
 *            GLM_NONE     -  render with only vertices
 *            GLM_FLAT     -  render with facet normals
 *            GLM_SMOOTH   -  render with vertex normals
 *            GLM_TEXTURE  -  render with texture coords
 *            GLM_COLOR    -  render with colors (color material)
 *            GLM_MATERIAL -  render with materials
 *            GLM_COLOR and GLM_MATERIAL should not both be specified.  
 *            GLM_FLAT and GLM_SMOOTH should not both be specified.  
 */
GLvoid
glmWriteOBJ(GLMmodel* model, char* filename, GLuint mode)
{
  GLuint    i;
  FILE*     file;
  GLMgroup* group;

  assert(model);

  /* do a bit of warning */
  if (mode & GLM_FLAT && !model->facetnorms) {
    printf("glmWriteOBJ() warning: flat normal output requested "
	   "with no facet normals defined.\n");
    mode &= ~GLM_FLAT;
  }
  if (mode & GLM_SMOOTH && !model->normals) {
    printf("glmWriteOBJ() warning: smooth normal output requested "
	   "with no normals defined.\n");
    mode &= ~GLM_SMOOTH;
  }
  if (mode & GLM_TEXTURE && !model->texcoords) {
    printf("glmWriteOBJ() warning: texture coordinate output requested "
	   "with no texture coordinates defined.\n");
    mode &= ~GLM_TEXTURE;
  }
  if (mode & GLM_FLAT && mode & GLM_SMOOTH) {
    printf("glmWriteOBJ() warning: flat normal output requested "
	   "and smooth normal output requested (using smooth).\n");
    mode &= ~GLM_FLAT;
  }
  if (mode & GLM_COLOR && !model->materials) {
    printf("glmWriteOBJ() warning: color output requested "
	   "with no colors (materials) defined.\n");
    mode &= ~GLM_COLOR;
  }
  if (mode & GLM_MATERIAL && !model->materials) {
    printf("glmWriteOBJ() warning: material output requested "
	   "with no materials defined.\n");
    mode &= ~GLM_MATERIAL;
  }
  if (mode & GLM_COLOR && mode & GLM_MATERIAL) {
    printf("glmDraw() warning: color and material output requested "
	   "outputting only materials.\n");
    mode &= ~GLM_COLOR;
  }


  /* open the file */
  file = fopen(filename, "w");
  if (!file) {
    fprintf(stderr, "glmWriteOBJ() failed: can't open file \"%s\" to write.\n",
	    filename);
    exit(1);
  }

  /* spit out a header */
  fprintf(file, "#  \n");
  fprintf(file, "#  Wavefront OBJ generated by GLM library\n");
  fprintf(file, "#  \n");
  fprintf(file, "#  GLM library\n");
  fprintf(file, "#  Nate Robins\n");
  fprintf(file, "#  ndr@pobox.com\n");
  fprintf(file, "#  http://www.pobox.com/~ndr\n");
  fprintf(file, "#  \n");

  if (mode & GLM_MATERIAL && model->mtllibname) {
    fprintf(file, "\nmtllib %s\n\n", model->mtllibname);
    glmWriteMTL(model, filename, model->mtllibname);
  }

  /* spit out the vertices */
  fprintf(file, "\n");
  fprintf(file, "# %d vertices\n", model->numvertices);
  for (i = 1; i <= model->numvertices; i++) {
    fprintf(file, "v %f %f %f\n", 
	    model->vertices[3 * i + 0],
	    model->vertices[3 * i + 1],
	    model->vertices[3 * i + 2]);
  }

  /* spit out the smooth/flat normals */
  if (mode & GLM_SMOOTH) {
    fprintf(file, "\n");
    fprintf(file, "# %d normals\n", model->numnormals);
    for (i = 1; i <= model->numnormals; i++) {
      fprintf(file, "vn %f %f %f\n", 
	      model->normals[3 * i + 0],
	      model->normals[3 * i + 1],
	      model->normals[3 * i + 2]);
    }
  } else if (mode & GLM_FLAT) {
    fprintf(file, "\n");
    fprintf(file, "# %d normals\n", model->numfacetnorms);
    for (i = 1; i <= model->numnormals; i++) {
      fprintf(file, "vn %f %f %f\n", 
	      model->facetnorms[3 * i + 0],
	      model->facetnorms[3 * i + 1],
	      model->facetnorms[3 * i + 2]);
    }
  }

  /* spit out the texture coordinates */
  if (mode & GLM_TEXTURE) {
    fprintf(file, "\n");
    fprintf(file, "# %d texcoords\n", model->texcoords);
    for (i = 1; i <= model->numtexcoords; i++) {
      fprintf(file, "vt %f %f\n", 
	      model->texcoords[2 * i + 0],
	      model->texcoords[2 * i + 1]);
    }
  }

  fprintf(file, "\n");
  fprintf(file, "# %d groups\n", model->numgroups);
  fprintf(file, "# %d faces (triangles)\n", model->numtriangles);
  fprintf(file, "\n");

  group = model->groups;
  while(group) {
    fprintf(file, "g %s\n", group->name);
    if (mode & GLM_MATERIAL)
      fprintf(file, "usemtl %s\n", model->materials[group->material].name);
    for (i = 0; i < group->numtriangles; i++) {
      if (mode & GLM_SMOOTH && mode & GLM_TEXTURE) {
	fprintf(file, "f %d/%d/%d %d/%d/%d %d/%d/%d\n",
		T(group->triangles[i]).vindices[0], 
		T(group->triangles[i]).nindices[0], 
		T(group->triangles[i]).tindices[0],
		T(group->triangles[i]).vindices[1],
		T(group->triangles[i]).nindices[1],
		T(group->triangles[i]).tindices[1],
		T(group->triangles[i]).vindices[2],
		T(group->triangles[i]).nindices[2],
		T(group->triangles[i]).tindices[2]);
      } else if (mode & GLM_FLAT && mode & GLM_TEXTURE) {
	fprintf(file, "f %d/%d %d/%d %d/%d\n",
		T(group->triangles[i]).vindices[0],
		T(group->triangles[i]).findex,
		T(group->triangles[i]).vindices[1],
		T(group->triangles[i]).findex,
		T(group->triangles[i]).vindices[2],
		T(group->triangles[i]).findex);
      } else if (mode & GLM_TEXTURE) {
	fprintf(file, "f %d/%d %d/%d %d/%d\n",
		T(group->triangles[i]).vindices[0],
		T(group->triangles[i]).tindices[0],
		T(group->triangles[i]).vindices[1],
		T(group->triangles[i]).tindices[1],
		T(group->triangles[i]).vindices[2],
		T(group->triangles[i]).tindices[2]);
      } else if (mode & GLM_SMOOTH) {
	fprintf(file, "f %d//%d %d//%d %d//%d\n",
		T(group->triangles[i]).vindices[0],
		T(group->triangles[i]).nindices[0],
		T(group->triangles[i]).vindices[1],
		T(group->triangles[i]).nindices[1],
		T(group->triangles[i]).vindices[2], 
		T(group->triangles[i]).nindices[2]);
      } else if (mode & GLM_FLAT) {
	fprintf(file, "f %d//%d %d//%d %d//%d\n",
		T(group->triangles[i]).vindices[0], 
		T(group->triangles[i]).findex,
		T(group->triangles[i]).vindices[1],
		T(group->triangles[i]).findex,
		T(group->triangles[i]).vindices[2],
		T(group->triangles[i]).findex);
      } else {
	fprintf(file, "f %d %d %d\n",
		T(group->triangles[i]).vindices[0],
		T(group->triangles[i]).vindices[1],
		T(group->triangles[i]).vindices[2]);
      }
    }
    fprintf(file, "\n");
    group = group->next;
  }

  fclose(file);
}

/* glmDraw: Renders the model to the current OpenGL context using the
 * mode specified.
 *
 * model    - initialized GLMmodel structure
 * mode     - a bitwise OR of values describing what is to be rendered.
 *            GLM_NONE     -  render with only vertices
 *            GLM_FLAT     -  render with facet normals
 *            GLM_SMOOTH   -  render with vertex normals
 *            GLM_TEXTURE  -  render with texture coords
 *            GLM_COLOR    -  render with colors (color material)
 *            GLM_MATERIAL -  render with materials
 *            GLM_COLOR and GLM_MATERIAL should not both be specified.  
 *            GLM_FLAT and GLM_SMOOTH should not both be specified.  
 */
GLvoid
glmDraw(GLMmodel* model, GLuint mode)
{
  static GLuint i;
  static GLMgroup* group;
  static GLMtriangle* triangle;
  static GLMmaterial* material;

  assert(model);
  assert(model->vertices);

  /* do a bit of warning */
  if (mode & GLM_FLAT && !model->facetnorms) {
    printf("glmDraw() warning: flat render mode requested "
	   "with no facet normals defined.\n");
    mode &= ~GLM_FLAT;
  }
  if (mode & GLM_SMOOTH && !model->normals) {
    printf("glmDraw() warning: smooth render mode requested "
	   "with no normals defined.\n");
    mode &= ~GLM_SMOOTH;
  }
  if (mode & GLM_TEXTURE && !model->texcoords) {
    printf("glmDraw() warning: texture render mode requested "
	   "with no texture coordinates defined.\n");
    mode &= ~GLM_TEXTURE;
  }
  if (mode & GLM_FLAT && mode & GLM_SMOOTH) {
    printf("glmDraw() warning: flat render mode requested "
	   "and smooth render mode requested (using smooth).\n");
    mode &= ~GLM_FLAT;
  }
  if (mode & GLM_COLOR && !model->materials) {
    printf("glmDraw() warning: color render mode requested "
	   "with no materials defined.\n");
    mode &= ~GLM_COLOR;
  }
  if (mode & GLM_MATERIAL && !model->materials) {
    printf("glmDraw() warning: material render mode requested "
	   "with no materials defined.\n");
    mode &= ~GLM_MATERIAL;
  }
  if (mode & GLM_COLOR && mode & GLM_MATERIAL) {
    printf("glmDraw() warning: color and material render mode requested "
	   "using only material mode.\n");
    mode &= ~GLM_COLOR;
  }
  if (mode & GLM_COLOR)
    glEnable(GL_COLOR_MATERIAL);
  else if (mode & GLM_MATERIAL)
    glDisable(GL_COLOR_MATERIAL);

  /* perhaps this loop should be unrolled into material, color, flat,
     smooth, etc. loops?  since most cpu's have good branch prediction
     schemes (and these branches will always go one way), probably
     wouldn't gain too much?  */

  group = model->groups;
  while (group) {
    if (mode & GLM_MATERIAL) {
      material = &model->materials[group->material];
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, material->ambient);
      glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material->diffuse);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, material->specular);
      glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, material->shininess);
    }

    if (mode & GLM_COLOR) {
      material = &model->materials[group->material];
      glColor3fv(material->diffuse);
    }

    glBegin(GL_TRIANGLES);
    for (i = 0; i < group->numtriangles; i++) {
      triangle = &T(group->triangles[i]);

      if (mode & GLM_FLAT)
	glNormal3fv(&model->facetnorms[3 * triangle->findex]);
      
      if (mode & GLM_SMOOTH)
	glNormal3fv(&model->normals[3 * triangle->nindices[0]]);
      if (mode & GLM_TEXTURE)
	glTexCoord2fv(&model->texcoords[2 * triangle->tindices[0]]);
      glVertex3fv(&model->vertices[3 * triangle->vindices[0]]);
      
      if (mode & GLM_SMOOTH)
	glNormal3fv(&model->normals[3 * triangle->nindices[1]]);
      if (mode & GLM_TEXTURE)
	glTexCoord2fv(&model->texcoords[2 * triangle->tindices[1]]);
      glVertex3fv(&model->vertices[3 * triangle->vindices[1]]);
      
      if (mode & GLM_SMOOTH)
	glNormal3fv(&model->normals[3 * triangle->nindices[2]]);
      if (mode & GLM_TEXTURE)
	glTexCoord2fv(&model->texcoords[2 * triangle->tindices[2]]);
      glVertex3fv(&model->vertices[3 * triangle->vindices[2]]);
      
    }
    glEnd();

    group = group->next;
  }
}

/* glmList: Generates and returns a display list for the model using
 * the mode specified.
 *
 * model    - initialized GLMmodel structure
 * mode     - a bitwise OR of values describing what is to be rendered.
 *            GLM_NONE     -  render with only vertices
 *            GLM_FLAT     -  render with facet normals
 *            GLM_SMOOTH   -  render with vertex normals
 *            GLM_TEXTURE  -  render with texture coords
 *            GLM_COLOR    -  render with colors (color material)
 *            GLM_MATERIAL -  render with materials
 *            GLM_COLOR and GLM_MATERIAL should not both be specified.  
 * GLM_FLAT and GLM_SMOOTH should not both be specified.  */
GLuint
glmList(GLMmodel* model, GLuint mode)
{
  GLuint list;

  list = glGenLists(1);
  glNewList(list, GL_COMPILE);
  glmDraw(model, mode);
  glEndList();

  return list;
}

/* glmWeld: eliminate (weld) vectors that are within an epsilon of
 * each other.
 *
 * model      - initialized GLMmodel structure
 * epsilon    - maximum difference between vertices
 *              ( 0.00001 is a good start for a unitized model)
 *
 */
GLuint
glmWeld(GLMmodel* model, GLfloat epsilon)
{
  GLfloat* vectors;
  GLfloat* copies;
  GLuint   numvectors;
  GLuint   i, welded;

  /* vertices */
  numvectors = model->numvertices;
  vectors    = model->vertices;
  copies = glmWeldVectors(vectors, &numvectors, epsilon);
  welded = model->numvertices - numvectors - 1;

  for (i = 0; i < model->numtriangles; i++) {
    T(i).vindices[0] = (GLuint)vectors[3 * T(i).vindices[0] + 0];
    T(i).vindices[1] = (GLuint)vectors[3 * T(i).vindices[1] + 0];
    T(i).vindices[2] = (GLuint)vectors[3 * T(i).vindices[2] + 0];
  }

  /* free space for old vertices */
  free(vectors);

  /* allocate space for the new vertices */
  model->numvertices = numvectors;
  model->vertices = (GLfloat*)malloc(sizeof(GLfloat) * 
				     3 * (model->numvertices + 1));

  /* copy the optimized vertices into the actual vertex list */
  for (i = 1; i <= model->numvertices; i++) {
    model->vertices[3 * i + 0] = copies[3 * i + 0];
    model->vertices[3 * i + 1] = copies[3 * i + 1];
    model->vertices[3 * i + 2] = copies[3 * i + 2];
  }

  free(copies);

  return welded;
}


#if 0
  /* normals */
  if (model->numnormals) {
  numvectors = model->numnormals;
  vectors    = model->normals;
  copies = glmOptimizeVectors(vectors, &numvectors);

  printf("glmOptimize(): %d redundant normals.\n", 
	 model->numnormals - numvectors);

  for (i = 0; i < model->numtriangles; i++) {
    T(i).nindices[0] = (GLuint)vectors[3 * T(i).nindices[0] + 0];
    T(i).nindices[1] = (GLuint)vectors[3 * T(i).nindices[1] + 0];
    T(i).nindices[2] = (GLuint)vectors[3 * T(i).nindices[2] + 0];
  }

  /* free space for old normals */
  free(vectors);

  /* allocate space for the new normals */
  model->numnormals = numvectors;
  model->normals = (GLfloat*)malloc(sizeof(GLfloat) * 
				    3 * (model->numnormals + 1));

  /* copy the optimized vertices into the actual vertex list */
  for (i = 1; i <= model->numnormals; i++) {
    model->normals[3 * i + 0] = copies[3 * i + 0];
    model->normals[3 * i + 1] = copies[3 * i + 1];
    model->normals[3 * i + 2] = copies[3 * i + 2];
  }

  free(copies);
  }

  /* texcoords */
  if (model->numtexcoords) {
  numvectors = model->numtexcoords;
  vectors    = model->texcoords;
  copies = glmOptimizeVectors(vectors, &numvectors);

  printf("glmOptimize(): %d redundant texcoords.\n", 
	 model->numtexcoords - numvectors);

  for (i = 0; i < model->numtriangles; i++) {
    for (j = 0; j < 3; j++) {
      T(i).tindices[j] = (GLuint)vectors[3 * T(i).tindices[j] + 0];
    }
  }

  /* free space for old texcoords */
  free(vectors);

  /* allocate space for the new texcoords */
  model->numtexcoords = numvectors;
  model->texcoords = (GLfloat*)malloc(sizeof(GLfloat) * 
				      2 * (model->numtexcoords + 1));

  /* copy the optimized vertices into the actual vertex list */
  for (i = 1; i <= model->numtexcoords; i++) {
    model->texcoords[2 * i + 0] = copies[2 * i + 0];
    model->texcoords[2 * i + 1] = copies[2 * i + 1];
  }

  free(copies);
  }
#endif

#if 0
  /* look for unused vertices */
  /* look for unused normals */
  /* look for unused texcoords */
  for (i = 1; i <= model->numvertices; i++) {
    for (j = 0; j < model->numtriangles; i++) {
      if (T(j).vindices[0] == i || 
	  T(j).vindices[1] == i || 
	  T(j).vindices[1] == i)
	break;
    }
  }
#endif


// GLM Stuff end


// ------------------------
// OBJLoader start
// ------------------------
//int MAXFLOAT = 0xffffff;

std::vector<nau::math::vec4>* readGL3FArray(GLfloat* a, unsigned int arraysize)
{
	std::vector<nau::math::vec4> *v = new std::vector<nau::math::vec4>;

	v->resize(arraysize);

	unsigned int i=0;

	//nau::math::vec3 v2;
	//v2..set

	for(i=0;i<arraysize;i++)
	{
		(*v)[i] = nau::math::vec4((float)a[(i+1)*3], (float)a[((i+1)*3)+1], (float)a[((i+1)*3)+2], 1.0f);
	}

	return v;
}

std::vector<nau::math::vec4>* readGL2FArray(GLfloat* a, unsigned int arraysize)
{
	std::vector<vec4> *v = new std::vector<nau::math::vec4>;

	v->resize(arraysize);

	unsigned int i=0;

	//nau::math::vec3 v2;
	//v2..set

	for(i=0;i<arraysize;i++)
	{
		(*v)[i] = nau::math::vec4((float)a[(i+1)*2], (float)a[((i+1)*2)+1], 0.0f, 1.0f);
	}

	return v;
}

void OBJLoader::loadScene (nau::scene::IScene *aScene, std::string &aFilename)
{
	// TODO
	// 

#ifdef OBJLOADER_DEBUG
	FILE *out = fopen("objdebug.txt","w");

	fprintf(out,"Reading obj\n");
	fflush(out);
#endif

	// Read OBJ file
	GLMmodel *obj = glmReadOBJ(aFilename.c_str());

#ifdef OBJLOADER_DEBUG
	fprintf(out,"Finished reading obj\n");
	fflush(out);

	// Convert stuff

	fprintf(out,"Starting intermediate reading\n");
	fflush(out);
#endif

	// Intermediate Vertices
	// Number of Vertices
	unsigned int nVertices;
	nVertices = obj->numvertices;
	// Vertices to vec3
	std::vector<nau::math::vec4>* verticesSource = readGL3FArray(obj->vertices,obj->numvertices);

	// SANITY CHECK - Facet Normals
	// These should exist on any real model.
	if (obj->numfacetnorms==0) { glmFacetNormals(obj); }

	// SANITY CHECK - NORMALS  (this should never happen on a real model!!)
	// Only here because some UNreal models lack Normals and tend to kind of crash Nau... -_-
	if (obj->numnormals==0)
	{ glmVertexNormals(obj,90);	}

	// Intermediate Normals
	// Number of normals
	unsigned int nNormals;
	nNormals = obj->numnormals;
	// Normals to vec3
	std::vector<nau::math::vec4>* normalsSource = readGL3FArray(obj->normals,obj->numnormals);


	// SANITY CHECK - If there are no Texture Coordinates, have to make some up!
	if (obj->numtexcoords == 0)
	{
		// Drat! Let's create a fake texture coordinate
		obj->texcoords = (GLfloat*)malloc(sizeof(GLfloat) * 2 * (1 + 1));
		obj->texcoords[2] = 0.0f;
		obj->texcoords[3] = 0.0f;
		// And now let's assign to every triangle the fake coords.
		for (unsigned int i = 0; i < obj->numtriangles; i++)
		{
			obj->triangles[i].tindices[0] = 1;
			obj->triangles[i].tindices[1] = 1;
			obj->triangles[i].tindices[2] = 1;
		}
		obj->numtexcoords = 1;
	}

	// Intermediate Texture Coordinates
	// Number of Tex Coords
	unsigned int nTexCoords;
	nTexCoords = obj->numtexcoords;
	// Tex Coords to vec3
	std::vector<nau::math::vec4>* texcoordsSource = readGL2FArray(obj->texcoords,obj->numtexcoords);

	// Intermediate Facet Normals
	// Number of Facet Normals
	unsigned int nFacetNormals;
	nFacetNormals = obj->numfacetnorms;
	// Facet Normals to vec3
	std::vector<nau::math::vec4>* facetnormsSource = readGL3FArray(obj->facetnorms,obj->numfacetnorms);

#ifdef OBJLOADER_DEBUG
	fprintf(out,"Finished intermediate reading\n");
	fflush(out);

	fprintf(out,"Creating Object\n");
	fflush(out);
#endif

	// Create an object for the Model. Make it a SimpleObject as there is no
	// OctreeNode information to be gleaned from OBJ files. (is there?)
	SceneObject *aObject = SceneObjectFactory::create ("SimpleObject");

	// Get and set name as the model path (whiny C++ about const string -_- )
	aObject->setName((new std::string(obj->pathname))->c_str());

	// Bounding Box? Hmm, GLModel does not include Bounding Box information.
	// That means reading and generating it off the data included within.
	// Though it LOOKS LIKE it's actually generated on the fly during execution
	// So I'll take advantage of the same methods

#ifdef OBJLOADER_DEBUG
	fprintf(out,"Creating Bounding Volume\n");
	fflush(out);
#endif

	// Create standard Bounding Box
	IBoundingVolume *aBoundingVolume = BoundingVolumeFactory::create ("BoundingBox");
	// Bind it to the object
	aObject->setBoundingVolume (aBoundingVolume);
	// Retrieve the BoundingVolume internal bounds for updating (actually NOT needed)
	//std::vector<vec3>& points = aBoundingVolume->getPoints(); // Thus commented out
	// Use the vertices list to calculate bounds and center.
	aBoundingVolume->calculate(*verticesSource);
	// -------------------------------------------------------------------------

#ifdef OBJLOADER_DEBUG
	fprintf(out,"Creating Transform\n");
	fflush(out);
#endif

	// Transform
	// Create Transform
	ITransform *aTransform = TransformFactory::create ("SimpleTransform");

	//// I have NO idea what matrix to generate. Identity, I choose you!
	//mat4 *mat = new mat4;
	//aTransform->setMat44(mat);

	// Set Transform
	aObject->setTransform(aTransform);
	// -------------------------------

#ifdef OBJLOADER_DEBUG
	fprintf(out,"Creating Renderable\n");
	fflush(out);
#endif

	// Renderable
	// Set Renderable Factory 
	IRenderable *aRenderable = 0;
	aRenderable = RESOURCEMANAGER->createRenderable ("Mesh","unnamed", aObject->getName());
	//aRenderable->setName("unnamed");
	//RESOURCEMANAGER->addRenderable(aRenderable, aObject->getName());
	// --------------------------------

#ifdef OBJLOADER_DEBUG
	fprintf(out,"Importing Vertex Data into Renderable\n");
	fflush(out);
#endif

	// Import VERTEX/NORMAL/TEXTURE data into Renderable
	VertexData *vdata = &(aRenderable->getVertexData()); //VertexData::create();

	std::vector<nau::math::vec4>* vertices = new std::vector<nau::math::vec4>;
	std::vector<nau::math::vec4>* normals = new std::vector<nau::math::vec4>;
	std::vector<nau::math::vec4>* texcoords = new std::vector<nau::math::vec4>;


	GLMgroup *currG = obj->groups;

	while (currG != NULL)
	{

		for (unsigned int i=0; i < currG->numtriangles ; i++)
		{
			// Obtain data
			std::vector<nau::math::vec4*> v; 
			v.resize(3);
			v[0] = new nau::math::vec4( (*verticesSource)[(obj->triangles[currG->triangles[i]].vindices[0])-1]);
			v[1] = new nau::math::vec4( (*verticesSource)[(obj->triangles[currG->triangles[i]].vindices[1])-1]);
			v[2] = new nau::math::vec4( (*verticesSource)[(obj->triangles[currG->triangles[i]].vindices[2])-1]);
			
			std::vector<nau::math::vec4*> n;
			n.resize(3);
			n[0] = new nau::math::vec4( (*normalsSource)[(obj->triangles[currG->triangles[i]].nindices[0])-1]);
			n[1] = new nau::math::vec4( (*normalsSource)[(obj->triangles[currG->triangles[i]].nindices[1])-1]);
			n[2] = new nau::math::vec4( (*normalsSource)[(obj->triangles[currG->triangles[i]].nindices[2])-1]);
		
			std::vector<nau::math::vec4*> t;
			t.resize(3);
			if ((obj->triangles[currG->triangles[i]].tindices[0] <= obj->numtexcoords) && obj->triangles[currG->triangles[i]].tindices[0] > 0) {
				t[0] = new nau::math::vec4( (*texcoordsSource)[(obj->triangles[currG->triangles[i]].tindices[0])-1]);
				t[1] = new nau::math::vec4( (*texcoordsSource)[(obj->triangles[currG->triangles[i]].tindices[1])-1]);
				t[2] = new nau::math::vec4( (*texcoordsSource)[(obj->triangles[currG->triangles[i]].tindices[2])-1]);
			}
			else {
				t[0] = new nau::math::vec4(0.0f, 0.0f, 0.0f, 0.0f);
				t[1] = new nau::math::vec4(0.0f, 0.0f, 0.0f, 0.0f);
				t[2] = new nau::math::vec4(0.0f, 0.0f, 0.0f, 0.0f);
			}
			// Loop through the three generated vertices, fetch the real values and
			// figure out whether they already exist.
			for (unsigned int i=0; i < 3; i++)
			{
				// Check the final tables to see whether it already exists
				unsigned int j=0;
				
				/*while (j < vertices->size())
				{	
					// Check position
					if (*(v[i]) == (*vertices)[j] && *(n[i]) == (*normals)[j] && *(t[i]) == (*texcoords)[j])
						break; // break out of the while cycle
					j++;
				}*/
				j = vertices->size();
				if (j == vertices->size())
				{
					// Vertex does not exist, add it
					vertices->push_back(*(v[i]));
					normals->push_back(*(n[i]));
					texcoords->push_back(*(t[i]));
					v[i] = NULL;
					n[i] = NULL;
					t[i] = NULL;
				}
				else
				{
					// It does exist, clean up
					delete v[i];
					delete n[i];
					delete t[i];
					v[i] = NULL;
					n[i] = NULL;
					t[i] = NULL;
				}
			}
	
			// Clean up
			//delete v;
			//delete n;
			//delete t;
		}

		currG = currG->next;
	}
	vdata->setDataFor(VertexData::getAttribIndex("position"), vertices);
	vdata->setDataFor(VertexData::getAttribIndex("normal"), normals);
	vdata->setDataFor(VertexData::getAttribIndex("texCoord0"), texcoords);

	//vdata->setIndexData(triangleIndexes);

	// Import MATERIAL data
#ifdef OBJLOADER_DEBUG
	fprintf(out,"Importing Material data\n");
	fflush(out);
#endif

	// Add all materials to the library first
	GLMmaterial *currM;
	for (unsigned int i=0; i < obj->nummaterials;i++)
	{
		currM = &(obj->materials[i]);

		// Create material
		Material* aMaterial;
		if (!MATERIALLIBMANAGER->hasMaterial (DEFAULTMATERIALLIBNAME, currM->name))
			aMaterial = MATERIALLIBMANAGER->createMaterial(currM->name);
		else
			aMaterial = MATERIALLIBMANAGER->getDefaultMaterial(currM->name);
		// Set Name
		//aMaterial->setName (currM->name);
		// Pass on data
		aMaterial->getColor().setProp(ColorMaterial::AMBIENT,currM->ambient);
		aMaterial->getColor().setProp(ColorMaterial::DIFFUSE,currM->diffuse);
		aMaterial->getColor().setProp(ColorMaterial::EMISSION,currM->emmissive);
		aMaterial->getColor().setProp(ColorMaterial::SHININESS,currM->shininess);
		aMaterial->getColor().setProp(ColorMaterial::SPECULAR,currM->specular);

		// Grab the texture
		if (currM->texture!=NULL && currM->texture!="")
		{
			std::string stmp = currM->texture;
			aMaterial->createTexture(0,stmp);
			
		}


		// Add to Material Lib Manager
		//if (!MATERIALLIBMANAGER->hasMaterial (DEFAULTMATERIALLIBNAME, aMaterial->getName()))
		//{ MATERIALLIBMANAGER->addMaterial (DEFAULTMATERIALLIBNAME, aMaterial); }
		//else
		//{ delete aMaterial; }
	}

	// SANITY CHECK - ARE THERE ANY ACTUAL MATERIALS DEFINED?
	if (obj->nummaterials==0)
	{
		// There HAS to be a material!! Otherwise, code goes boom
		// in a messy way ahead! Workaround by adding a default material...

		// Create material
		if (!MATERIALLIBMANAGER->hasMaterial(DEFAULTMATERIALLIBNAME,"Default3DSMat"))
			Material* aMaterial = MATERIALLIBMANAGER->createMaterial("Default3DSMat");
		// Set Name
		//aMaterial->setName ("OBJDEFAULT_NONEDEFINED");
		// Set Defaults
		//float d = 0.5f;
		//float da = 0.8f;
		//aMaterial->getColor().setDiffuse(d,d,d,da);
		//aMaterial->getColor().setAmbient(d,d,d,da);
		//aMaterial->getColor().setEmission(0,0,0,da);
		//aMaterial->getColor().setShininess(0.0f);
		//aMaterial->getColor().setSpecular(d,d,d,da);
		// Add to Material Lib Manager (if it doesn't already exist!)
		/*if (!MATERIALLIBMANAGER->hasMaterial (DEFAULTMATERIALLIBNAME, aMaterial->getName()))
		{ MATERIALLIBMANAGER->addMaterial (DEFAULTMATERIALLIBNAME, aMaterial); }
		else
		{ delete aMaterial; }*/
	}

#ifdef OBJLOADER_DEBUG
	fprintf(out,"Importing Material Group data, and there are %d groups\n", obj->numgroups);
	fflush(out);
#endif


	currG = obj->groups;
	int offset = 0;

	for (unsigned int i=0; i < obj->numgroups; i++)
	{
		#ifdef OBJLOADER_DEBUG
			fprintf(out,"Starting Group #%d\n", i);
			fflush(out);
		#endif

		// Then import MaterialGroups

		#ifdef OBJLOADER_DEBUG
			fprintf(out,"Setting Parent\n");
			fflush(out);
		#endif


		#ifdef OBJLOADER_DEBUG
			fprintf(out,"Finished setting Parent\n");
			fflush(out);

			fprintf(out,"Setting MatGroup Name, Mat: %d , numMats: %d\n", currG->material, obj->nummaterials);
			fflush(out);
		#endif

		// SANITY CHECK - ARE THERE ANY ACTUAL MATERIALS DEFINED?
			
		std::string s;
		if (obj->nummaterials==0)
			// NONE! Use default
			s = "Default3DSMat";
		else
			// Set material group name
			s = obj->materials[currG->material].name;

		MaterialGroup *aMatGroup = new MaterialGroup(aRenderable, s);
		//aMatGroup->setParent (aRenderable);
		// Set up the index array

		#ifdef OBJLOADER_DEBUG
			fprintf(out,"Creating Array\n");
			fflush(out);
		#endif

		// Create array
		std::vector<unsigned int> *iArr = new std::vector<unsigned int>;
		iArr->resize(currG->numtriangles*3);

		// Now the tricky part, we have to match the indices in the OBJ file with the corresponding vertex data
		// then look up in the tables to figure out their actual index
		// So...
		// For each triangle...
		unsigned int triangleCounter,verticeCounter;
		for (triangleCounter = 0; triangleCounter < currG->numtriangles; triangleCounter++)
		{
			// ... and for each vertice...
			for (verticeCounter = 0; verticeCounter < 3; verticeCounter++)
			{
				//unsigned int vp = (obj->triangles[currG->triangles[triangleCounter]].vindices[verticeCounter])-1;
				//unsigned int np = (obj->triangles[currG->triangles[triangleCounter]].nindices[verticeCounter])-1;
				//unsigned int tp = (obj->triangles[currG->triangles[triangleCounter]].tindices[verticeCounter])-1;
				
				// Get the vertex information
			/*	nau::math::vec4 *v = new nau::math::vec4((*verticesSource)[vp]);
				nau::math::vec4 *n = new nau::math::vec4((*normalsSource)[np]);
				nau::math::vec4 *t = new nau::math::vec4((*texcoordsSource)[tp]);*/

				unsigned int lookupCounter = 0;
				bool found = false;
				// Then lookup which index matters
				/*for (lookupCounter = 0; lookupCounter < vertices->size() && found == false; lookupCounter++)
				{
					if ((*v) == (*vertices)[lookupCounter] && (*n) == (*normals)[lookupCounter] && (*t) == (*texcoords)[lookupCounter])
					{
					found = true;
					}
				}*/
				// Store index
				//(*iArr)[(triangleCounter*3)+verticeCounter] = lookupCounter-1;
				(*iArr)[(triangleCounter*3)+verticeCounter] = (triangleCounter*3)+verticeCounter + offset;
				// Free up memory
				/*delete v;
				delete n;
				delete t;*/

				// And done with vertice
			}
			// And done with triangle
		}

		#ifdef OBJLOADER_DEBUG
			fprintf(out,"Assigning Index List to Material Group\n");
			fflush(out);
		#endif

		// Assign it to Material Group
		aMatGroup->setIndexList(iArr);

		#ifdef OBJLOADER_DEBUG
			fprintf(out,"Adding MatGroup to Renderable\n");
			fflush(out);
		#endif

		// Add it to the Renderable
		if (currG->numtriangles > 0)
			aRenderable->addMaterialGroup(aMatGroup);
		// Well, using UGLY HAXX to avoid memory copy, too.
		//aRenderable->getMaterialGroups().push_back (aMatGroup);

		// Get next group
		currG = currG->next;
		offset += (triangleCounter*3);
	}

#ifdef OBJLOADER_DEBUG
	fprintf(out,"Setting renderable\n");
	fflush(out);
#endif

	// Set the Object's Renderable
	aObject->setRenderable (aRenderable);

#ifdef OBJLOADER_DEBUG
	fprintf(out,"Adding object to scene\n");
	fflush(out);
#endif

	// Add Object to the scene  (uncomment when done)
	aScene->add (aObject);
	
#ifdef OBJLOADER_DEBUG
	fprintf(out,"Deleting temporary GLMmodel\n");
	fflush(out);
#endif

	// Delete temporary GLMmodel
	glmDelete(obj);

	// Done!
#ifdef OBJLOADER_DEBUG
	fprintf(out,"Done importing!\n");
	fflush(out);

	fclose(out);
#endif
}

void OBJLoader::writeScene (nau::scene::IScene *aScene, std::string &aFilename)
{

}