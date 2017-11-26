
#ifndef image_H_
#define image_H_

#include "GL/glut.h"
#include "IL/il.h"
#include "IL/ilu.h"

class AImage
{
public:
   AImage();
   ~AImage();

   void flipVertical();
   void flipHorizontal();
   void rotate();

   bool load(const char* name);
   GLubyte* data();
   int width();
   int height();
   void clear();

protected:

   GLubyte* myData;
   ILuint myHandle;
   unsigned int myWidth;
   unsigned int myHeight;
   static bool theInitialized;
};

#endif
