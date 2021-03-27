/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2014, Biagio Montesano, all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#include "precomp_custom.hpp"

namespace slam
{
namespace line_descriptor
{
/* draw matches between two images */
void drawLineMatches( const cv::Mat& img1, const std::vector<KeyLine>& keylines1, const cv::Mat& img2, const std::vector<KeyLine>& keylines2,
                      const std::vector<cv::DMatch>& matches1to2, cv::Mat& outImg, const cv::Scalar& matchColor, const cv::Scalar& singleLineColor,
                      const std::vector<char>& matchesMask, int flags )
{

  if(img1.type() != img2.type())
  {
    std::cout << "Input images have different types" << std::endl;
    CV_Assert(img1.type() == img2.type());
  }

  /* initialize output matrix (if necessary) */
  if( flags == DrawLinesMatchesFlags::DEFAULT )
  {
    /* check how many rows are necessary for output matrix */
    int totalRows = img1.rows >= img2.rows ? img1.rows : img2.rows;

    /* initialize output matrix */
    outImg = cv::Mat::zeros( totalRows, img1.cols + img2.cols, img1.type() );

  }

  /* initialize random seed: */
  srand( (unsigned int) time( NULL ) );

  cv::Scalar singleLineColorRGB;
  if( singleLineColor == cv::Scalar::all( -1 ) )
  {
    int R = ( rand() % (int) ( 255 + 1 ) );
    int G = ( rand() % (int) ( 255 + 1 ) );
    int B = ( rand() % (int) ( 255 + 1 ) );

    singleLineColorRGB = cv::Scalar( R, G, B );
  }

  else
    singleLineColorRGB = singleLineColor;

  /* copy input images to output images */
  cv::Mat roi_left( outImg, cv::Rect( 0, 0, img1.cols, img1.rows ) );
  cv::Mat roi_right( outImg, cv::Rect( img1.cols, 0, img2.cols, img2.rows ) );
  img1.copyTo( roi_left );
  img2.copyTo( roi_right );

  /* get columns offset */
  int offset = img1.cols;

  /* if requested, draw lines from both images */
  if( flags != DrawLinesMatchesFlags::NOT_DRAW_SINGLE_LINES )
  {
    for ( size_t i = 0; i < keylines1.size(); i++ )
    {
      KeyLine k1 = keylines1[i];
      //line( outImg, Point2f( k1.startPointX, k1.startPointY ), Point2f( k1.endPointX, k1.endPointY ), singleLineColorRGB, 2 );
      line( outImg, cv::Point2f( k1.sPointInOctaveX, k1.sPointInOctaveY ), cv::Point2f( k1.ePointInOctaveX, k1.ePointInOctaveY ), singleLineColorRGB, 2 );

    }

    for ( size_t j = 0; j < keylines2.size(); j++ )
    {
      KeyLine k2 = keylines2[j];
      line( outImg, cv::Point2f( k2.sPointInOctaveX + offset, k2.sPointInOctaveY ), cv::Point2f( k2.ePointInOctaveX + offset, k2.ePointInOctaveY ), singleLineColorRGB, 2 );
    }
  }

  /* draw matches */
  for ( size_t counter = 0; counter < matches1to2.size(); counter++ )
  {
    if( counter < matchesMask.size() && matchesMask[counter] != 0 )
    {
      cv::DMatch dm = matches1to2[counter];
      KeyLine left = keylines1[dm.queryIdx];
      KeyLine right = keylines2[dm.trainIdx];

      cv::Scalar matchColorRGB;
      if( matchColor == cv::Scalar::all( -1 ) )
      {
        int R = ( rand() % (int) ( 255 + 1 ) );
        int G = ( rand() % (int) ( 255 + 1 ) );
        int B = ( rand() % (int) ( 255 + 1 ) );

        matchColorRGB = cv::Scalar( R, G, B );

        if( singleLineColor == cv::Scalar::all( -1 ) )
          singleLineColorRGB = matchColorRGB;
      }

      else
        matchColorRGB = matchColor;

      /* draw lines if necessary */
//      line( outImg, Point2f( left.startPointX, left.startPointY ), Point2f( left.endPointX, left.endPointY ), singleLineColorRGB, 2 );
//
//      line( outImg, Point2f( right.startPointX + offset, right.startPointY ), Point2f( right.endPointX + offset, right.endPointY ), singleLineColorRGB,
//            2 );
//
//      /* link correspondent lines */
//      line( outImg, Point2f( left.startPointX, left.startPointY ), Point2f( right.startPointX + offset, right.startPointY ), matchColorRGB, 1 );

      line( outImg, cv::Point2f( left.sPointInOctaveX, left.sPointInOctaveY ), cv::Point2f( left.ePointInOctaveX, left.ePointInOctaveY ), singleLineColorRGB, 2 );

        line( outImg, cv::Point2f( right.sPointInOctaveX + offset, right.sPointInOctaveY ), cv::Point2f( right.ePointInOctaveX + offset, right.ePointInOctaveY ), singleLineColorRGB,
              2 );

        /* link correspondent lines */
        line( outImg, cv::Point2f( left.sPointInOctaveX, left.sPointInOctaveY ), cv::Point2f( right.sPointInOctaveX + offset, right.sPointInOctaveY ), matchColorRGB, 1 );
    }
  }
}

/* draw extracted lines on original image */
void drawKeylines( const cv::Mat& image, const std::vector<KeyLine>& keylines, cv::Mat& outImage, const cv::Scalar& color, int flags )
{
  if( flags == DrawLinesMatchesFlags::DEFAULT )
    outImage = image.clone();

  for ( size_t i = 0; i < keylines.size(); i++ )
  {
    /* decide lines' color  */
    cv::Scalar lineColor;
    if( color == cv::Scalar::all( -1 ) )
    {
      int R = ( rand() % (int) ( 255 + 1 ) );
      int G = ( rand() % (int) ( 255 + 1 ) );
      int B = ( rand() % (int) ( 255 + 1 ) );

      lineColor = cv::Scalar( R, G, B );
    }

    else
      lineColor = color;

    /* get line */
    KeyLine k = keylines[i];

    /* draw line */
    line( outImage, cv::Point2f( k.startPointX, k.startPointY ), cv::Point2f( k.endPointX, k.endPointY ), lineColor, 1 );
  }
}

}
}
