/* Copyright (c) 2017-2018, CNRS-LAAS
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include "Raster_Reader.h"


Raster_Reader::Raster_Reader(std::string path)
{

  GDALAllRegister();

/*Once the drivers are registered, the application should call the free standing GDALOpen() function to open a dataset,
passing the name of the dataset and the access desired (GA_ReadOnly or GA_Update).*/


  /// get a DEM file

  gDataSet = (GDALDataset*) GDALOpen(path.c_str(), GA_ReadOnly);

  ///check if data is present

  if (gDataSet == NULL)
  {
    throw std::invalid_argument("Unable to open "+path);
  }

  nCols = gDataSet->GetRasterXSize();
  nRows = gDataSet->GetRasterYSize();
  nBands = gDataSet->GetRasterCount();


}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//this function fetches the values of :originX, originY, pWidth, pHeightusing GDAL geotransform ;



void Raster_Reader::geoTransform()
{


  if (gDataSet->GetGeoTransform(gTransform) == CE_None)
  {

    //position of the top left x
    originX = gTransform[0];
    //position of top left y
    originY = gTransform[3];
    //the pixel width //west-east pixel resolution
    pWidth = gTransform[1];
    //the pixel height//north-south pixel resolution (negative value)
    pHeight = gTransform[5];

  } else
  {

    cerr << "DEM error : no transform can be fetched";
  }


}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

string Raster_Reader::get_projection()
{

  return gDataSet->GetProjectionRef();

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Raster_Reader::Mat_height()
{

  GDALRasterBand* poBand;
  poBand = gDataSet->GetRasterBand(1);
  float maxH = 0;
  float minH = 10000;


  float* Buffer;

  int nXSize = poBand->GetXSize();
  //buffer to read row:creates a free space in the memory with the size we order
  // Buffer = (float *) CPLMalloc(sizeof(float)*nXSize);

  //read row data and store in vector

  for (int i = 0; i < nRows; i++)
  {

    Buffer = (float*) CPLMalloc(sizeof(float) * nXSize);
    // read a row
    poBand->RasterIO(GF_Read, 0, i, nXSize, 1, Buffer, nXSize, 1, GDT_Float32, 0, 0);

    for (int j = 0; j < nCols; ++j)
    {

      RasterData.push_back(Buffer[j]);
      maxH = max(maxH, Buffer[j]);
      if (Buffer[j] != poBand->GetNoDataValue())
      {
        minH = min(minH, Buffer[j]);
      }

    }
    CPLFree(Buffer);

  }
  maxheight = maxH;
  minheight = minH;


}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Raster_Reader::Put_in_Raster(cv::Mat& FP, string gdal_result_path)
{
  cv::Mat firemap = FP.clone();
  cv::imwrite("/home/welarfao/results/Map begore mapping.jpg", firemap);


  GDALDriver* poDriver;
  GDALRasterBand* pBand;
  const char* pszFormat = "GTiff";
  poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);

  char** papszOptions = NULL;

  GDALDataset* poDstDS;
  poDstDS = poDriver->Create(gdal_result_path.c_str(), nCols, nRows, 1, GDT_Float32, papszOptions);

  poDstDS->SetGeoTransform(gTransform);
  poDstDS->SetProjection(gDataSet->GetProjectionRef());

  pBand = poDstDS->GetRasterBand(1);
  //poDstDS->GetRasterBand(1)->SetNoDataValue(0); // with this option only the perimeter of the fire is shown

  float* Buffer;
  Buffer = (float*) CPLMalloc(sizeof(float) * nCols);

  for (int i = 0; i < nRows; i++)
  {
    for (int j = 0; j < nCols; ++j)
    {

      Buffer[j] = firemap.at<uchar>(i, j);
    }

    pBand->RasterIO(GF_Write, 0, i, nCols, 1, Buffer, nCols, 1, GDT_Float32, 0, 0);
  }

  GDALClose((GDALDatasetH) poDstDS);


  CPLFree(Buffer);

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


Point2D Raster_Reader::get_World_coord(float col, float row)
{

  Point2D point;

  point.x = originX + col * gTransform[1];//+ row*gTransform[2];
  point.y = originY /* + col*gTransform[4]*/ + row * gTransform[5];

  return point;

}


///////////////////////////////////////////////////////////////////////////////////////////



Pixel Raster_Reader::get_pixel(double x, double y)
{
/* X = originX + col*geoTransform[1] + row*geoTransform[2];
   Y = originY + col*geoTransform[4] + row*geoTransform[5];

   we consider that we won t have any rotation ,so the equations become :
   X = originX + col*geoTransform[1] ;
   Y = originY + row*geoTransform[5];
*/

  Pixel pix;

  pix.col = (int) ((x - originX) / gTransform[1]);

  pix.row = (int) ((y - originY) / gTransform[5]);

  return pix;

}


///////////////////////////////////////////////////////////////////////////////////////////

double Raster_Reader::get_max_east()
{

  return originX + nCols * gTransform[1];


}

double Raster_Reader::get_max_west()
{

  return originX;

}

double Raster_Reader::get_max_south()
{

  return originY + nRows * gTransform[5];

}

double Raster_Reader::get_max_north()
{

  return originY;

}


double Raster_Reader::get_noData()
{

  return gDataSet->GetRasterBand(1)->GetNoDataValue();

}


////////////////////////////////////////////////////////

double Raster_Reader::get_height(uint64_t col, uint64_t row)
{
/*
RasterData=[ (# # # # ... # # # nCols values)first row /  (# # # # ... # # # nCols values )second row / ..............(# # # # ... # # # nCols values)last row which is the row number nRows]

so to get to the row number y we multiply it with number of cols ,and then we add the position of the the colonne x we need to be exqctly at the position (x,y) of the matrix
*/
  uint64_t cpt = 0;

  cpt = nCols * row + col;


  return RasterData[cpt];
}




////////////////////////////////////////////////////////////////////////////////////////////





Raster_Reader::~Raster_Reader()
{
  if (gDataSet != NULL)
  {
    GDALClose(gDataSet);

  }
}

double Raster_Reader::get_pixel_width()
{
  return pWidth;
}
