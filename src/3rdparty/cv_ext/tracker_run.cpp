/*M///////////////////////////////////////////////////////////////////////////////////////
//
// IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
// By downloading, copying, installing or using the software you agree to this license.
// If you do not agree to this license, do not download, install,
// copy or use the software.
//
// License Agreement
// For Open Source Computer Vision Library
// (3-clause BSD License)
//
// Copyright (C) 2000-2015, Intel Corporation, all rights reserved.
// Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
// Copyright (C) 2009-2015, NVIDIA Corporation, all rights reserved.
// Copyright (C) 2010-2013, Advanced Micro Devices, Inc., all rights reserved.
// Copyright (C) 2015, OpenCV Foundation, all rights reserved.
// Copyright (C) 2015, Itseez Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
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
//M*/

/*
// Original file: https://github.com/Itseez/opencv_contrib/blob/292b8fa6aa403fb7ad6d2afadf4484e39d8ca2f1/modules/tracking/samples/tracker.cpp
// + Author: Klaus Haag
// * Refactor file: Split into files/classes
// * Replace command line arguments
// * Add a variety of additional features
*/

#include "tracker_run.hpp"

#include <iostream>
#include <ctype.h>

#include "init_box_selector.hpp"
#include "cf_tracker.hpp"
#include <windows.h>

using namespace cv;
using namespace std;
using namespace TCLAP;
using namespace cf_tracking;


TrackerRun::TrackerRun(ImageAcquisition& cap, size_t n, std::string windowTitle):
_windowTitle(windowTitle),
_cmd(_windowTitle.c_str(), ' ', "0.1"),
_debug(0),
_cap(cap),
_cnt1(0),
_cnt2(0),
_trackers(n)
{
	startTrackers();
}


TrackerRun::TrackerRun(ImageAcquisition& cap, std::vector<cv::Rect>& boxes, std::vector<string>&labels, string windowTitle) :
TrackerRun(cap, boxes.size(), windowTitle)
{
	_paras.initBbs = boxes;
    _paras.initLabels = labels;
    _paras.initLabels.resize(_paras.initBbs.size());
}

TrackerRun::~TrackerRun()
{
    if (_resultsFile.is_open())
        _resultsFile.close();
	stopTrackers();
	cv::destroyAllWindows();
}

Parameters TrackerRun::parseCmdArgs(int argc, const char** argv)
{
    Parameters& paras = _paras;

    try{
        ValueArg<int> deviceIdArg("c", "cam", "Camera device id", false, 0, "integer", _cmd);
        ValueArg<string> seqPathArg("s", "seq", "Path to sequence", false, "", "path", _cmd);
        ValueArg<string> expansionArg("i", "image_name_expansion", "image name expansion (only necessary for image sequences) ie. /%.05d.jpg", false, "", "string", _cmd);
        MultiArg<string> initBbArg("b", "box", "Init Bounding Box", false, "x,y,w,h", _cmd);
        SwitchArg noShowOutputSw("n", "no-show", "Don't show video", _cmd, false);
        SwitchArg noSaveVideoSw("", "no_video", "Don't save video", _cmd, false);
        ValueArg<double> videoScaleArg("", "video_scale", "the scale of saved video", false, 0.5, "double", _cmd);
        ValueArg<string> outputPathArg("o", "out", "Path to output file", false, "", "file path", _cmd);
        ValueArg<string> imgExportPathArg("e", "export", "Path to output folder where the images will be saved with BB", false, "", "folder path", _cmd);
        SwitchArg pausedSw("p", "paused", "Start paused", _cmd, false);
        SwitchArg repeatSw("r", "repeat", "endless loop the same sequence", _cmd, false);
        ValueArg<int> startFrameArg("", "start_frame", "starting frame idx (starting at 1 for the first frame)", false, 1, "integer", _cmd);
        SwitchArg dummySequenceSw("", "mock_sequence", "Instead of processing a regular sequence, a dummy sequence is used to evaluate run time performance.", _cmd, false);
        parseTrackerParas(_cmd, argc, argv);

        paras.device = deviceIdArg.getValue();
        paras.sequencePath = seqPathArg.getValue();
        string expansion = expansionArg.getValue();

        size_t foundExpansion = expansion.find_first_of('.');

        if (foundExpansion != string::npos)
            expansion.erase(foundExpansion, 1);

        paras.expansion = expansion;

        paras.outputFilePath = outputPathArg.getValue();
        paras.imgExportPath = imgExportPathArg.getValue();
        paras.showOutput = !noShowOutputSw.getValue();
        paras.paused = pausedSw.getValue();
        paras.repeat = repeatSw.getValue();
        paras.startFrame = startFrameArg.getValue();
        paras.saveVideo = !noSaveVideoSw.getValue();
        paras.videoScale = videoScaleArg.getValue();

		for (const auto& t : initBbArg.getValue())
		{
			stringstream initBbSs(t);

			double initBb[4];

			for (int i = 0; i < 4; ++i)
			{
				string singleValueStr;
				getline(initBbSs, singleValueStr, ',');
				initBb[i] = static_cast<double>(stod(singleValueStr.c_str()));
			}

			paras.initBbs.push_back(Rect_<double>(initBb[0], initBb[1], initBb[2], initBb[3]));
		}

        if (_debug != 0)
            _debug->init(paras.outputFilePath + "_debug");

        paras.isMockSequence = dummySequenceSw.getValue();
    }
    catch (ArgException &argException)
    {
        cerr << "Command Line Argument Exception: " << argException.what() << endl;
        exit(-1);
    }
    // TODO: properly check every argument and throw exceptions accordingly
    catch (...)
    {
        cerr << "Command Line Argument Exception!" << endl;
        exit(-1);
    }

    return paras;
}

bool TrackerRun::start(int argc, const char** argv)
{
    _paras = parseCmdArgs(argc, argv);
    if (_paras.saveVideo && _paras.videoName.empty())
    {
        SYSTEMTIME st;
        GetLocalTime(&st);
        std::stringstream ss;
        ss << setfill('0') << setw(4) << st.wYear;
        ss << setw(2) << st.wMonth << st.wDay;
        ss << setw(2) << st.wHour << st.wMinute << st.wSecond;
        ss << setw(3) << st.wMilliseconds;// << ".avi";
        _paras.videoName = ss.str() + ".avi";
        _paras.logName = ss.str() + ".csv";
    }

    while (true)
    {
        if (init() == false)
            return false;
        if (run() == false)
            return false;

        if (!_paras.repeat || _exit)
            break;

		for (auto& t : _trackers)
		{
			t._isTrackerInitialzed = false;
			t._hasInitBox = false;
		}
    }

    return true;
}

bool TrackerRun::init()
{
	if (!_cap.isOpened())
	{
		ImgAcqParas imgAcqParas;
		imgAcqParas.device = _paras.device;
		imgAcqParas.expansionStr = _paras.expansion;
		imgAcqParas.isMock = _paras.isMockSequence;
		imgAcqParas.sequencePath = _paras.sequencePath;
		_cap.open(imgAcqParas);
	}

    if (!_cap.isOpened())
    {
        cerr << "Could not open device/sequence/video!" << endl;
        exit(-1);
    }

    int startIdx = _paras.startFrame - 1;

    // HACKFIX:
    //_cap.set(CV_CAP_PROP_POS_FRAMES, startIdx);
    // OpenCV's _cap.set in combination with image sequences is
    // currently bugged on Linux
    // TODO: review when OpenCV 3.0 is stable
    cv::Mat temp;

    for (int i = 0; i < startIdx; ++i)
        _cap >> temp;
    // HACKFIX END

    if (_paras.showOutput)
        namedWindow(_windowTitle.c_str());

    if (!_paras.outputFilePath.empty())
    {
        _resultsFile.open(_paras.outputFilePath.c_str());

        if (!_resultsFile.is_open())
        {
            std::cerr << "Error: Unable to create results file: "
                << _paras.outputFilePath.c_str()
                << std::endl;

            return false;
        }

        _resultsFile.precision(std::numeric_limits<double>::digits10 - 4);
    }

	for (size_t i = 0; i < _paras.initBbs.size() && i < _trackers.size(); i++)
	{
		_trackers[i]._hasInitBox = true;
		_trackers[i]._boundingBox = _paras.initBbs[i];
	}

    _isPaused = _paras.paused;
    _frameIdx = 0;
    return true;
}

bool TrackerRun::run()
{
    bool success = true;

    std::cout << "Switch pause with 'p'" << std::endl;
    std::cout << "Step frame with 'c'" << std::endl;
    std::cout << "Select new target with 'r'" << std::endl;
    std::cout << "Update current tracker model at new location  't'" << std::endl;
    std::cout << "Quit with 'ESC'" << std::endl;

    while (true)
    {
        success = update();

        if (!success)
            break;
    }

    _cap.release();

    return true;
}

bool TrackerRun::update()
{
    int64 tStart = 0;
    int64 tDuration = 0;

    if (!_isPaused || _frameIdx == 0 || _isStep)
    {
        _cap >> _image;

        if (_image.empty())
            return false;

        ++_frameIdx;
        std::cout << "frame: " << _frameIdx << std::endl;
    }

	tStart = getTickCount();

	if (!reinitTrackers())
	{
		return false;
	}
    else if (!_isPaused || _isStep)
    {
        _isStep = false;
		if (_updateAtPos)
		{
			if (!updateAtTrackers())
			{
				return false;
			}
			_updateAtPos = false;
		}
        else
        {
			updateTrackers();
        }
    }
    tDuration = getTickCount() - tStart;

    _trackLog(_paras.logName);

    double fps = static_cast<double>(getTickFrequency() / tDuration);
    if (_paras.showOutput || _paras.saveVideo)
    {
        Mat hudImage;
        _image.copyTo(hudImage);
		drawTrackers(hudImage);

		stringstream ss;
		ss << "FPS: " << fps;
		putText(hudImage, ss.str(), Point(20, 20), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(255, 0, 0));

        ss.str("");
        ss.clear();
        ss << "#" << _frameIdx;
        putText(hudImage, ss.str(), Point(hudImage.cols - 60, 20), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(255, 0, 0));

        if (_debug != 0)
            _debug->printOnImage(hudImage);

        if (!_paras.imgExportPath.empty())
        {
            stringstream ssi;
            ssi << setfill('0') << setw(5) << _frameIdx << ".png";
            std::string imgPath = _paras.imgExportPath + ssi.str();

            try
            {
                imwrite(imgPath, hudImage);
            }
            catch (runtime_error& runtimeError)
            {
                cerr << "Could not write output images: " << runtimeError.what() << endl;
            }
        }

        if (_paras.saveVideo)
        {
            cv::Mat frame;
            cv::resize(hudImage, frame, cv::Size(), _paras.videoScale, _paras.videoScale);
            cf_tracking::TrackerDebug::recordVideo(hudImage, _paras.videoName);
        }

        if (_paras.showOutput)
        {
            imshow(_windowTitle.c_str(), hudImage);
            char c = (char)waitKey(10);

            if (c == 27)
            {
                _exit = true;
                return false;
            }

            switch (c)
            {
            case 'p':
                _isPaused = !_isPaused;
                break;
            case 'c':
                _isStep = true;
                break;
            case 'r':
                for (auto& t : _trackers)
                {
                    t._isTrackerInitialzed = false;
                    t._hasInitBox = false;
                }
                for (size_t i = 0; i < _paras.initBbs.size() && i < _trackers.size(); i++)
                {
                    _trackers[i]._hasInitBox = true;
                    _trackers[i]._boundingBox = _paras.initBbs[i];
                }
                break;
            case 't':
                _updateAtPos = true;
                break;
            default:
                ;
            }
        }
    }

    return true;
}

void TrackerRun::printResults(const cv::Rect_<double>& boundingBox, bool isConfident, double fps)
{
    if (_resultsFile.is_open())
    {
        if (boundingBox.width > 0 && boundingBox.height > 0 && isConfident)
        {
            _resultsFile << boundingBox.x << ","
                << boundingBox.y << ","
                << boundingBox.width << ","
                << boundingBox.height << ","
                << fps << std::endl;
        }
        else
        {
            _resultsFile << "NaN, NaN, NaN, NaN, " << fps << std::endl;
        }

        if (_debug != 0)
            _debug->printToFile();
    }
}


bool TrackerRun::reinitTrackers()
{
	vector<Rect> boxes;
	for (auto& t : _trackers)
	{
		if (!t._isTrackerInitialzed)
		{
			if (!t._hasInitBox)
			{
				Rect box;
				if (!InitBoxSelector::selectBox(_image, box, boxes))
					return false;

				std::cout << "selected box shape: " << box << std::endl;

				t._boundingBox = Rect_<double>(static_cast<double>(box.x),
					static_cast<double>(box.y),
					static_cast<double>(box.width),
					static_cast<double>(box.height));

				t._hasInitBox = true;
			}

			if (t._tracker)
				t._targetOnFrame = t._tracker->reinit(_image, t._boundingBox);
			else
				t._targetOnFrame = false;

			if (t._targetOnFrame)
				t._isTrackerInitialzed = true;
		}

		if (t._isTrackerInitialzed)
		{
			boxes.push_back(t._boundingBox);
		}
		else
		{
			boxes.push_back(cv::Rect());
		}
	}
	return true;
}


bool TrackerRun::updateAtTrackers()
{
	vector<Rect> boxes;
	for (auto& t : _trackers)
	{
		Rect box;

		if (!InitBoxSelector::selectBox(_image, box, boxes))
			return false;

		boxes.push_back(box);
		t._boundingBox = Rect_<double>(static_cast<double>(box.x),
			static_cast<double>(box.y),
			static_cast<double>(box.width),
			static_cast<double>(box.height));

		std::cout << "UpdateAt_: " << t._boundingBox << std::endl;

		if (t._tracker)
			t._targetOnFrame = t._tracker->updateAt(_image, t._boundingBox);
		else
			t._targetOnFrame = false;

		if (!t._targetOnFrame)
			std::cout << "Target not found!" << std::endl;
	}
	return true;
}


void TrackerRun::updateTrackers()
{
	unique_lock<mutex> lck{ _mtx };
    _cnt2 = 0;
	_cnt1 = _trackers.size();
	_cv.notify_all();
	lck.unlock();

    //_cv.wait(std::unique_lock<std::mutex>{std::mutex{}}, [this]() { return _cnt1 == 0; });

    lck.lock();
    std::cout << "wait tracking..." << std::endl;
	_cv.wait(lck, [this]() { return _cnt1 == 0; });
    std::cout << "track done" << std::endl;
    lck.unlock();
}


void TrackerRun::drawTrackers(cv::Mat img)
{
    for (size_t i = 0; i < _trackers.size(); i++)
	{
        auto& t = _trackers[i];
		rectangle(img, t._boundingBox, Scalar(0, 0, 255), 2);
		Point_<double> center;
		center.x = t._boundingBox.x + t._boundingBox.width / 2;
		center.y = t._boundingBox.y + t._boundingBox.height / 2;
		circle(img, center, 3, Scalar(0, 0, 255), 2);

        int cx = (int)center.x;
        int cy = (int)center.y;
        int x = (int)t._boundingBox.x;
        int y = (int)t._boundingBox.y;
        int w = (int)t._boundingBox.width;
        int h = (int)t._boundingBox.height;
        std::stringstream ss;
        ss << "(" << cx << "," << cy << ")";
        putText(img, ss.str(), Point(cx+10, cy), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(255, 0, 0));

        ss.str("");
        ss << "w:" << w << ", h:" << h << " " << _paras.initLabels[i];
        putText(img, ss.str(), Point(x,y-3), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(255, 0, 0));

		if (!t._targetOnFrame)
		{
			cv::Point_<double> tl = t._boundingBox.tl();
			cv::Point_<double> br = t._boundingBox.br();

			line(img, tl, br, Scalar(0, 0, 255));
			line(img, cv::Point_<double>(tl.x, br.y),
				cv::Point_<double>(br.x, tl.y), Scalar(0, 0, 255));
		}
	}
}


void TrackerRun::startTrackers()
{
	for (auto& t : _trackers)
	{
		t._thread = thread{ [this, &t]() {
			while (!_exit)
			{
				unique_lock<mutex> lck{ _mtx };
                std::cout << t._thread.get_id() << " wait _cnt1 " << _cnt1 << std::endl;
				_cv.wait(lck, [this]() { return _cnt1 == _trackers.size(); });
                std::cout << t._thread.get_id() << " wait _cnt1 done " << _cnt1 << std::endl;
				lck.unlock();

				if (_exit) break;

                //std::cout << t._thread.get_id() << "..." << std::endl;
				if (t._tracker)
					t._targetOnFrame = t._tracker->update(_image, t._boundingBox);
				else
					t._targetOnFrame = false;
                //std::cout << t._thread.get_id() << "!!!" << std::endl;

                //lck.lock();
				//_cnt1--;
                //std::cout << "cnt: " << _cnt << std::endl;
				//_cv.notify_all();
                //lck.unlock();

                lck.lock();
                _cv.notify_all();
                _cnt2++;
                if (_cnt2 < _trackers.size())
                {
                    cout << t._thread.get_id() <<  " wait _cnt2 " << _cnt2 << std::endl;
                    _cv.wait(lck, [this]() { return _cnt2 == _trackers.size(); });
                    cout << t._thread.get_id() << " wait _cnt2 done " << _cnt2 << std::endl;
                }
                else
                {
                    _cv.notify_all();
                }
                lck.unlock();

                _cnt1 = 0;
                _cv.notify_all();
			}
		} };
	}
}

void TrackerRun::stopTrackers()
{
	_exit = true;
	for (auto&t : _trackers)
	{
		unique_lock<mutex> lck(_mtx);
		_cnt1 = _trackers.size();
		_cv.notify_all();
		lck.unlock();

		for (auto& t : _trackers)
		{
			if (t._thread.joinable())
				t._thread.join();

			if (t._tracker)
				delete t._tracker;
			
			t._tracker = 0;
		}
	}
}

void TrackerRun::setTrackerDebug(cf_tracking::TrackerDebug* debug)
{
    _debug = debug;
}
