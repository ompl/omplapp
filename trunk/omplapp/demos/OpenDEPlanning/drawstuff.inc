// this file is essentially copied form the drawstuff lib (part of ODE)
// http://sourceforge.net/projects/opende/

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <ode/ode.h>

#include <cstdio>

#define dsError printf

//***************************************************************************
// PPM image object

typedef unsigned char byte;

class Image {
  int image_width,image_height;
  byte *image_data;
public:
  Image (char *filename);
  // load from PPM file
  ~Image();
  int width() { return image_width; }
  int height() { return image_height; }
  byte *data() { return image_data; }
};


// skip over whitespace and comments in a stream.

static void skipWhiteSpace (char *filename, FILE *f)
{
  int c,d;
  for(;;) {
    c = fgetc(f);
    if (c==EOF) dsError ("unexpected end of file in \"%s\"",filename);

    // skip comments
    if (c == '#') {
      do {
	d = fgetc(f);
	if (d==EOF) dsError ("unexpected end of file in \"%s\"",filename);
      } while (d != '\n');
      continue;
    }

    if (c > ' ') {
      ungetc (c,f);
      return;
    }
  }
}


// read a number from a stream, this return 0 if there is none (that's okay
// because 0 is a bad value for all PPM numbers anyway).

static int readNumber (char *filename, FILE *f)
{
  int c,n=0;
  for(;;) {
    c = fgetc(f);
    if (c==EOF) dsError ("unexpected end of file in \"%s\"",filename);
    if (c >= '0' && c <= '9') n = n*10 + (c - '0');
    else {
      ungetc (c,f);
      return n;
    }
  }
}


Image::Image (char *filename)
{
  FILE *f = fopen (filename,"rb");
  if (!f) dsError ("Can't open image file `%s'",filename);

  // read in header
  if (fgetc(f) != 'P' || fgetc(f) != '6')
    dsError ("image file \"%s\" is not a binary PPM (no P6 header)",filename);
  skipWhiteSpace (filename,f);

  // read in image parameters
  image_width = readNumber (filename,f);
  skipWhiteSpace (filename,f);
  image_height = readNumber (filename,f);
  skipWhiteSpace (filename,f);
  int max_value = readNumber (filename,f);

  // check values
  if (image_width < 1 || image_height < 1)
    dsError ("bad image file \"%s\"",filename);
  if (max_value != 255)
    dsError ("image file \"%s\" must have color range of 255",filename);

  // read either nothing, LF (10), or CR,LF (13,10)
  int c = fgetc(f);
  if (c == 10) {
    // LF
  }
  else if (c == 13) {
    // CR
    c = fgetc(f);
    if (c != 10) ungetc (c,f);
  }
  else ungetc (c,f);

  // read in rest of data
  image_data = new byte [image_width*image_height*3];
  if (fread (image_data,image_width*image_height*3,1,f) != 1)
    dsError ("Can not read data from image file `%s'",filename);
  fclose (f);
}


Image::~Image()
{
  delete[] image_data;
}

//***************************************************************************
// Texture object.

class Texture {
  Image *image;
  GLuint name;
public:
  Texture (char *filename);
  ~Texture();
  void bind (int modulate);
};


Texture::Texture (char *filename)
{
  image = new Image (filename);
  glGenTextures (1,&name);
  glBindTexture (GL_TEXTURE_2D,name);

  // set pixel unpacking mode
  glPixelStorei (GL_UNPACK_SWAP_BYTES, 0);
  glPixelStorei (GL_UNPACK_ROW_LENGTH, 0);
  glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei (GL_UNPACK_SKIP_ROWS, 0);
  glPixelStorei (GL_UNPACK_SKIP_PIXELS, 0);

  // glTexImage2D (GL_TEXTURE_2D, 0, 3, image->width(), image->height(), 0,
  //		   GL_RGB, GL_UNSIGNED_BYTE, image->data());
  gluBuild2DMipmaps (GL_TEXTURE_2D, 3, image->width(), image->height(),
		     GL_RGB, GL_UNSIGNED_BYTE, image->data());

  // set texture parameters - will these also be bound to the texture???
  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
		   GL_LINEAR_MIPMAP_LINEAR);

  glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
}


Texture::~Texture()
{
  delete image;
  glDeleteTextures (1,&name);
}


void Texture::bind (int modulate)
{
  glBindTexture (GL_TEXTURE_2D,name);
  glTexEnvi (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,
	     modulate ? GL_MODULATE : GL_DECAL);
}

static int use_textures=1;		// 1 if textures to be drawn
static int use_shadows=1;		// 1 if shadows to be drawn
static int current_state = 0;
static Texture *sky_texture = 0;
static Texture *ground_texture = 0;
static Texture *wood_texture = 0;
static Texture *checkered_texture = 0;

static Texture *texture[4+1]; // +1 since index 0 is not used

static float color[4] = {0,0,0,0};	// current r,g,b,alpha color
static int tnum = 0;			// current texture number

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

// constants to convert degrees to radians and the reverse
#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD (M_PI/180.0)

// light vector. LIGHTZ is implicitly 1
#define LIGHTX (1.0f)
#define LIGHTY (0.4f)

// ground and sky
#define SHADOW_INTENSITY (0.65f)
#define GROUND_R (0.5f) 	// ground color for when there's no texture
#define GROUND_G (0.5f)
#define GROUND_B (0.3f)

const float ground_scale = 1.0f/1.0f;	// ground texture scale (1/size)
const float ground_ofsx = 0.5;		// offset of ground texture
const float ground_ofsy = 0.5;
const float sky_scale = 1.0f/4.0f;	// sky texture scale (1/size)
const float sky_height = 1.0f;		// sky height above viewpoint

static void setColor (float r, float g, float b, float alpha)
{
  GLfloat light_ambient[4],light_diffuse[4],light_specular[4];
  light_ambient[0] = r*0.3f;
  light_ambient[1] = g*0.3f;
  light_ambient[2] = b*0.3f;
  light_ambient[3] = alpha;
  light_diffuse[0] = r*0.7f;
  light_diffuse[1] = g*0.7f;
  light_diffuse[2] = b*0.7f;
  light_diffuse[3] = alpha;
  light_specular[0] = r*0.2f;
  light_specular[1] = g*0.2f;
  light_specular[2] = b*0.2f;
  light_specular[3] = alpha;
  glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, light_ambient);
  glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, light_diffuse);
  glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, light_specular);
  glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 5.0f);
}

void dsSetColor (float red, float green, float blue)
{
  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  color[0] = red;
  color[1] = green;
  color[2] = blue;
  color[3] = 1;
}

void dsSetColorAlpha (float red, float green, float blue,
				 float alpha)
{
  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  color[0] = red;
  color[1] = green;
  color[2] = blue;
  color[3] = alpha;
}

static void setTransform (const float pos[3], const float R[12])
{
  GLfloat matrix[16];
  matrix[0]=R[0];
  matrix[1]=R[4];
  matrix[2]=R[8];
  matrix[3]=0;
  matrix[4]=R[1];
  matrix[5]=R[5];
  matrix[6]=R[9];
  matrix[7]=0;
  matrix[8]=R[2];
  matrix[9]=R[6];
  matrix[10]=R[10];
  matrix[11]=0;
  matrix[12]=pos[0];
  matrix[13]=pos[1];
  matrix[14]=pos[2];
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf (matrix);
}

static void setTransformD (const double pos[3], const double R[12])
{
  GLdouble matrix[16];
  matrix[0]=R[0];
  matrix[1]=R[4];
  matrix[2]=R[8];
  matrix[3]=0;
  matrix[4]=R[1];
  matrix[5]=R[5];
  matrix[6]=R[9];
  matrix[7]=0;
  matrix[8]=R[2];
  matrix[9]=R[6];
  matrix[10]=R[10];
  matrix[11]=0;
  matrix[12]=pos[0];
  matrix[13]=pos[1];
  matrix[14]=pos[2];
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixd (matrix);
}

static void setupDrawingMode()
{
  glEnable (GL_LIGHTING);
  if (tnum) {
    if (use_textures) {
      glEnable (GL_TEXTURE_2D);
      texture[tnum]->bind (1);
      glEnable (GL_TEXTURE_GEN_S);
      glEnable (GL_TEXTURE_GEN_T);
      glTexGeni (GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
      glTexGeni (GL_T,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
      static GLfloat s_params[4] = {1.0f,1.0f,0.0f,1};
      static GLfloat t_params[4] = {0.817f,-0.817f,0.817f,1};
      glTexGenfv (GL_S,GL_OBJECT_PLANE,s_params);
      glTexGenfv (GL_T,GL_OBJECT_PLANE,t_params);
    }
    else {
      glDisable (GL_TEXTURE_2D);
    }
  }
  else {
    glDisable (GL_TEXTURE_2D);
  }
  setColor (color[0],color[1],color[2],color[3]);

  if (color[3] < 1) {
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  }
  else {
    glDisable (GL_BLEND);
  }
}

static void setShadowDrawingMode()
{
  glDisable (GL_LIGHTING);
  if (use_textures) {
    glEnable (GL_TEXTURE_2D);
    ground_texture->bind (1);
    glColor3f (SHADOW_INTENSITY,SHADOW_INTENSITY,SHADOW_INTENSITY);
    glEnable (GL_TEXTURE_2D);
    glEnable (GL_TEXTURE_GEN_S);
    glEnable (GL_TEXTURE_GEN_T);
    glTexGeni (GL_S,GL_TEXTURE_GEN_MODE,GL_EYE_LINEAR);
    glTexGeni (GL_T,GL_TEXTURE_GEN_MODE,GL_EYE_LINEAR);
    static GLfloat s_params[4] = {ground_scale,0,0,ground_ofsx};
    static GLfloat t_params[4] = {0,ground_scale,0,ground_ofsy};
    glTexGenfv (GL_S,GL_EYE_PLANE,s_params);
    glTexGenfv (GL_T,GL_EYE_PLANE,t_params);
  }
  else {
    glDisable (GL_TEXTURE_2D);
    glColor3f (GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY,
	       GROUND_B*SHADOW_INTENSITY);
  }
  glDepthRange (0,0.9999);
}

static void setShadowTransform()
{
  GLfloat matrix[16];
  for (int i=0; i<16; i++) matrix[i] = 0;
  matrix[0]=1;
  matrix[5]=1;
  matrix[8]=-LIGHTX;
  matrix[9]=-LIGHTY;
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf (matrix);
}

static void drawBox (const float sides[3])
{
  float lx = sides[0]*0.5f;
  float ly = sides[1]*0.5f;
  float lz = sides[2]*0.5f;

  // sides
  glBegin (GL_TRIANGLE_STRIP);
  glNormal3f (-1,0,0);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,-ly,lz);
  glVertex3f (-lx,ly,-lz);
  glVertex3f (-lx,ly,lz);
  glNormal3f (0,1,0);
  glVertex3f (lx,ly,-lz);
  glVertex3f (lx,ly,lz);
  glNormal3f (1,0,0);
  glVertex3f (lx,-ly,-lz);
  glVertex3f (lx,-ly,lz);
  glNormal3f (0,-1,0);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,-ly,lz);
  glEnd();

  // top face
  glBegin (GL_TRIANGLE_FAN);
  glNormal3f (0,0,1);
  glVertex3f (-lx,-ly,lz);
  glVertex3f (lx,-ly,lz);
  glVertex3f (lx,ly,lz);
  glVertex3f (-lx,ly,lz);
  glEnd();

  // bottom face
  glBegin (GL_TRIANGLE_FAN);
  glNormal3f (0,0,-1);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,ly,-lz);
  glVertex3f (lx,ly,-lz);
  glVertex3f (lx,-ly,-lz);
  glEnd();
}

void dsDrawBox (const float pos[3], const float R[12],
		const float sides[3])
{
  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glShadeModel (GL_FLAT);
  setTransform (pos,R);
  drawBox (sides);
  glPopMatrix();

  if (use_shadows) {
    setShadowDrawingMode();
    setShadowTransform();
    setTransform (pos,R);
    drawBox (sides);
    glPopMatrix();
    glPopMatrix();
    glDepthRange (0,1);
  }
}

void dsDrawBoxD (const double pos[3], const double R[12],
		 const double sides[3])
{
  int i;
  float pos2[3],R2[12],fsides[3];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];
  for (i=0; i<3; i++) fsides[i]=(float)sides[i];
  dsDrawBox (pos2,R2,fsides);
}


static void drawPatch (float p1[3], float p2[3], float p3[3], int level)
{
  int i;
  if (level > 0) {
    float q1[3],q2[3],q3[3];		 // sub-vertices
    for (i=0; i<3; i++) {
      q1[i] = 0.5f*(p1[i]+p2[i]);
      q2[i] = 0.5f*(p2[i]+p3[i]);
      q3[i] = 0.5f*(p3[i]+p1[i]);
    }
    float length1 = (float)(1.0/sqrt(q1[0]*q1[0]+q1[1]*q1[1]+q1[2]*q1[2]));
    float length2 = (float)(1.0/sqrt(q2[0]*q2[0]+q2[1]*q2[1]+q2[2]*q2[2]));
    float length3 = (float)(1.0/sqrt(q3[0]*q3[0]+q3[1]*q3[1]+q3[2]*q3[2]));
    for (i=0; i<3; i++) {
      q1[i] *= length1;
      q2[i] *= length2;
      q3[i] *= length3;
    }
    drawPatch (p1,q1,q3,level-1);
    drawPatch (q1,p2,q2,level-1);
    drawPatch (q1,q2,q3,level-1);
    drawPatch (q3,q2,p3,level-1);
  }
  else {
    glNormal3f (p1[0],p1[1],p1[2]);
    glVertex3f (p1[0],p1[1],p1[2]);
    glNormal3f (p2[0],p2[1],p2[2]);
    glVertex3f (p2[0],p2[1],p2[2]);
    glNormal3f (p3[0],p3[1],p3[2]);
    glVertex3f (p3[0],p3[1],p3[2]);
  }
}


// draw a sphere of radius 1

static int sphere_quality = 1;

static void drawSphere()
{
  // icosahedron data for an icosahedron of radius 1.0
# define ICX 0.525731112119133606f
# define ICZ 0.850650808352039932f
  static GLfloat idata[12][3] = {
    {-ICX, 0, ICZ},
    {ICX, 0, ICZ},
    {-ICX, 0, -ICZ},
    {ICX, 0, -ICZ},
    {0, ICZ, ICX},
    {0, ICZ, -ICX},
    {0, -ICZ, ICX},
    {0, -ICZ, -ICX},
    {ICZ, ICX, 0},
    {-ICZ, ICX, 0},
    {ICZ, -ICX, 0},
    {-ICZ, -ICX, 0}
  };

  static int index[20][3] = {
    {0, 4, 1},	  {0, 9, 4},
    {9, 5, 4},	  {4, 5, 8},
    {4, 8, 1},	  {8, 10, 1},
    {8, 3, 10},   {5, 3, 8},
    {5, 2, 3},	  {2, 7, 3},
    {7, 10, 3},   {7, 6, 10},
    {7, 11, 6},   {11, 0, 6},
    {0, 1, 6},	  {6, 1, 10},
    {9, 0, 11},   {9, 11, 2},
    {9, 2, 5},	  {7, 2, 11},
  };

  static GLuint listnum = 0;
  if (listnum==0) {
    listnum = glGenLists (1);
    glNewList (listnum,GL_COMPILE);
    glBegin (GL_TRIANGLES);
    for (int i=0; i<20; i++) {
      drawPatch (&idata[index[i][2]][0],&idata[index[i][1]][0],
		 &idata[index[i][0]][0],sphere_quality);
    }
    glEnd();
    glEndList();
  }
  glCallList (listnum);
}


static void drawSphereShadow (float px, float py, float pz, float radius)
{
  // calculate shadow constants based on light vector
  static int init=0;
  static float len2,len1,scale;
  if (!init) {
    len2 = LIGHTX*LIGHTX + LIGHTY*LIGHTY;
    len1 = 1.0f/(float)sqrt(len2);
    scale = (float) sqrt(len2 + 1);
    init = 1;
  }

  // map sphere center to ground plane based on light vector
  px -= LIGHTX*pz;
  py -= LIGHTY*pz;

  const float kx = 0.96592582628907f;
  const float ky = 0.25881904510252f;
  float x=radius, y=0;

  glBegin (GL_TRIANGLE_FAN);
  for (int i=0; i<24; i++) {
    // for all points on circle, scale to elongated rotated shadow and draw
    float x2 = (LIGHTX*x*scale - LIGHTY*y)*len1 + px;
    float y2 = (LIGHTY*x*scale + LIGHTX*y)*len1 + py;
    glTexCoord2f (x2*ground_scale+ground_ofsx,y2*ground_scale+ground_ofsy);
    glVertex3f (x2,y2,0);

    // rotate [x,y] vector
    float xtmp = kx*x - ky*y;
    y = ky*x + kx*y;
    x = xtmp;
  }
  glEnd();
}

void dsDrawSphere (const float pos[3], const float R[12],
		   float radius)
{
  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glEnable (GL_NORMALIZE);
  glShadeModel (GL_SMOOTH);
  setTransform (pos,R);
  glScaled (radius,radius,radius);
  drawSphere();
  glPopMatrix();
  glDisable (GL_NORMALIZE);

  // draw shadows
  if (use_shadows) {
    glDisable (GL_LIGHTING);
    if (use_textures) {
      ground_texture->bind (1);
      glEnable (GL_TEXTURE_2D);
      glDisable (GL_TEXTURE_GEN_S);
      glDisable (GL_TEXTURE_GEN_T);
      glColor3f (SHADOW_INTENSITY,SHADOW_INTENSITY,SHADOW_INTENSITY);
    }
    else {
      glDisable (GL_TEXTURE_2D);
      glColor3f (GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY,
		 GROUND_B*SHADOW_INTENSITY);
    }
    glShadeModel (GL_FLAT);
    glDepthRange (0,0.9999);
    drawSphereShadow (pos[0],pos[1],pos[2],radius);
    glDepthRange (0,1);
  }
}


void dsDrawSphereD (const double pos[3], const double R[12], float radius)
{
  int i;
  float pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];
  dsDrawSphere (pos2,R2,radius);
}

// draw a capped cylinder of length l and radius r, aligned along the x axis

static int capped_cylinder_quality = 3;

static void drawCapsule (float l, float r)
{
  int i,j;
  float tmp,nx,ny,nz,start_nx,start_ny,a,ca,sa;
  // number of sides to the cylinder (divisible by 4):
  const int n = capped_cylinder_quality*4;

  l *= 0.5;
  a = float(M_PI*2.0)/float(n);
  sa = (float) sin(a);
  ca = (float) cos(a);

  // draw cylinder body
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_STRIP);
  for (i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,l);
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,-l);
    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw first cylinder cap
  start_nx = 0;
  start_ny = 1;
  for (j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    float start_nx2 =  ca*start_nx + sa*start_ny;
    float start_ny2 = -sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    nx = start_nx; ny = start_ny; nz = 0;
    float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (i=0; i<=n; i++) {
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*r,nz2*r,l+nx2*r);
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*r,nz*r,l+nx*r);
      // rotate n,n2
      tmp = ca*ny - sa*nz;
      nz = sa*ny + ca*nz;
      ny = tmp;
      tmp = ca*ny2- sa*nz2;
      nz2 = sa*ny2 + ca*nz2;
      ny2 = tmp;
    }
    glEnd();
    start_nx = start_nx2;
    start_ny = start_ny2;
  }

  // draw second cylinder cap
  start_nx = 0;
  start_ny = 1;
  for (j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    float start_nx2 = ca*start_nx - sa*start_ny;
    float start_ny2 = sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    nx = start_nx; ny = start_ny; nz = 0;
    float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (i=0; i<=n; i++) {
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*r,nz*r,-l+nx*r);
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*r,nz2*r,-l+nx2*r);
      // rotate n,n2
      tmp = ca*ny - sa*nz;
      nz = sa*ny + ca*nz;
      ny = tmp;
      tmp = ca*ny2- sa*nz2;
      nz2 = sa*ny2 + ca*nz2;
      ny2 = tmp;
    }
    glEnd();
    start_nx = start_nx2;
    start_ny = start_ny2;
  }
}


// draw a cylinder of length l and radius r, aligned along the z axis

static void drawCylinder (float l, float r, float zoffset)
{
  int i;
  float tmp,ny,nz,a,ca,sa;
  const int n = 24;	// number of sides to the cylinder (divisible by 4)

  l *= 0.5;
  a = float(M_PI*2.0)/float(n);
  sa = (float) sin(a);
  ca = (float) cos(a);

  // draw cylinder body
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_STRIP);
  for (i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,l+zoffset);
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,-l+zoffset);
    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw top cap
  glShadeModel (GL_FLAT);
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,1);
  glVertex3d (0,0,l+zoffset);
  for (i=0; i<=n; i++) {
    if (i==1 || i==n/2+1)
      setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
    glNormal3d (0,0,1);
    glVertex3d (ny*r,nz*r,l+zoffset);
    if (i==1 || i==n/2+1)
      setColor (color[0],color[1],color[2],color[3]);

    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw bottom cap
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,-1);
  glVertex3d (0,0,-l+zoffset);
  for (i=0; i<=n; i++) {
    if (i==1 || i==n/2+1)
      setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
    glNormal3d (0,0,-1);
    glVertex3d (ny*r,nz*r,-l+zoffset);
    if (i==1 || i==n/2+1)
      setColor (color[0],color[1],color[2],color[3]);

    // rotate ny,nz
    tmp = ca*ny + sa*nz;
    nz = -sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();
}

void dsDrawCapsule (const float pos[3], const float R[12],
				      float length, float radius)
{
  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glShadeModel (GL_SMOOTH);
  setTransform (pos,R);
  drawCapsule (length,radius);
  glPopMatrix();

  if (use_shadows) {
    setShadowDrawingMode();
    setShadowTransform();
    setTransform (pos,R);
    drawCapsule (length,radius);
    glPopMatrix();
    glPopMatrix();
    glDepthRange (0,1);
  }
}

void dsDrawCapsuleD (const double pos[3], const double R[12],
			    float length, float radius)
{
  int i;
  float pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];
  dsDrawCapsule (pos2,R2,length,radius);
}

void dsDrawCylinder (const float pos[3], const float R[12],
		      float length, float radius)
{
  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glShadeModel (GL_SMOOTH);
  setTransform (pos,R);
  drawCylinder (length,radius,0);
  glPopMatrix();

  if (use_shadows) {
    setShadowDrawingMode();
    setShadowTransform();
    setTransform (pos,R);
    drawCylinder (length,radius,0);
    glPopMatrix();
    glPopMatrix();
    glDepthRange (0,1);
  }
}

void dsDrawCylinderD (const double pos[3], const double R[12],
		      float length, float radius)
{
  int i;
  float pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];
  dsDrawCylinder (pos2,R2,length,radius);
}
