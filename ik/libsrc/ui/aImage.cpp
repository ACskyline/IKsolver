#include <cstring>
#include "aImage.h"

bool AImage::theInitialized = false;

AImage::AImage() : myData(NULL), myWidth(0), myHeight(0)
{
   if (!theInitialized)
   {
      ilInit();
      theInitialized = true;
   }
}

AImage::~AImage()
{
   clear();
}

bool AImage::load(const char* name)
{
   clear();

   ILenum error;
   ilGenImages(1, &myHandle);

   ilBindImage(myHandle);
   if ((error = ilGetError()) != IL_NO_ERROR)
   {
      printf("IL bind image failure: %s\n", 
         iluErrorString(error));
      return false;
   }

   ilLoadImage((char* const) name);
   if ((error = ilGetError()) != IL_NO_ERROR)
   {
      printf("IL load image failure: %s\n", 
         iluErrorString(error));
      return false;
   }

   ILinfo info;
   iluGetImageInfo(&info);
   printf("Loading image %s (%dx%d)\n", 
      name, info.Width, info.Height);

   iluFlipImage();

   myWidth = info.Width;
   myHeight = info.Height;
   myData = new GLubyte[myWidth*myHeight*3];
   ilCopyPixels( 0, 0, 0, // offset
       myWidth, myHeight, 1, // width, height, depth
       IL_RGB, IL_UNSIGNED_BYTE, myData);

   if ((error = ilGetError()) != IL_NO_ERROR)
   {
      printf("IL copy pixels failure: %s\n", 
         iluErrorString(error));

      return false;
   }

   return true;
}

int AImage::width()
{
   return myWidth;
}

int AImage::height()
{
   return myHeight;
}

void AImage::clear()
{
   if (myData)
   {
      ilDeleteImages(1, &myHandle);
      delete[] myData;
   }
}

GLubyte* AImage::data()
{
  return myData;
}

void AImage::rotate()
{
    GLubyte* temp = new GLubyte[myWidth*myHeight*3];

    for (unsigned int i = 0; i < myHeight; i++)
    {
       for (unsigned int j = 0; j < myWidth; j++)
       {
          int tindex = 3*(i*myWidth + j);
          int dindex = 3*(j*myWidth + (myHeight-1-i));

          temp[tindex] = myData[dindex];
          temp[tindex+1] = myData[dindex+1];
          temp[tindex+2] = myData[dindex+2];
       }
    }

    memcpy(myData, temp, myWidth*myHeight*sizeof(GLubyte)*3);

    delete[] temp;
}

void AImage::flipVertical()
{
    GLubyte* temp = new GLubyte[myWidth*myHeight*3];

    for (unsigned int i = 0; i < myHeight; i++)
    {
       GLubyte* temprow = &temp[i*myWidth*3];
       GLubyte* row = &myData[i*myWidth*3];

       int k = myWidth*3-3;
       for (unsigned int j = 0; j < myWidth*3; j+=3)
       {
          temprow[j] = row[k];
          temprow[j+1] = row[k+1];
          temprow[j+2] = row[k+2];
          k-=3;
       }
    }

    memcpy(myData, temp, myWidth*myHeight*sizeof(GLubyte)*3);

    delete[] temp;
}

void AImage::flipHorizontal()
{
    GLubyte* temp = new GLubyte[myWidth*myHeight*3];

    int j = 0;
    for (int i = myHeight-1; i > -1; i--)
    {
      memcpy(&temp[j++*myWidth*3], &myData[i*myWidth*3], myWidth*3);
    }

    memcpy(myData, temp, myWidth*myWidth*sizeof(GLubyte)*3);

    delete[] temp;
}
