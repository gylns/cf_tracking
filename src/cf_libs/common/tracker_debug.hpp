/*
// License Agreement (3-clause BSD License)
// Copyright (c) 2015, Klaus Haag, all rights reserved.
// Third party copyrights and patents are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the names of the copyright holders nor the names of the contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall copyright holders or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
*/

#ifndef TRACKER_DEBUG_HPP_
#define TRACKER_DEBUG_HPP_

#include <opencv2/videoio.hpp>

namespace cf_tracking
{
    class TrackerDebug
    {
    public:
        virtual ~TrackerDebug(){}

        virtual void init(std::string outputFilePath) = 0;
        virtual void printOnImage(cv::Mat& image) = 0;
        virtual void printConsoleOutput() = 0;
        virtual void printToFile() = 0;
        static void recordVideo(cv::Mat image, std::string videoPath)
        {
            static cv::VideoWriter recorder;
            static std::string path;
            //const int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
            const int fourcc = cv::VideoWriter::fourcc('D', 'I', 'V', 'X');
            //const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
            if (!recorder.isOpened() && !videoPath.empty() && !image.empty())
            {
                path = videoPath;
                recorder.open(path, fourcc, 25., image.size());
            }
            if (recorder.isOpened())
            {
                if (path == videoPath)
                {
                    recorder << image;
                }
                else
                {
                    recorder.release();
                    path = "";
                    recordVideo(image, videoPath);
                }
            }
        }
    };
}

#endif
