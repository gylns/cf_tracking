#pragma once

#include <opencv2/dnn.hpp>
#include <string>

struct YOLOParam
{
	cv::Scalar mean = 0.;
	float scale = 0.00392f;
	bool swapRB = true;
	int width = 416;
	int height = 416;
	float confThreshold = 0.5f;
	float nmsThreshold = 0.4f;
};

struct DResult
{
	int cls;
	float confidence;
	cv::Rect box;
};

class YOLOModel
{
public:
	YOLOModel(std::string cfg_file, std::string model_file)
	{
		_net = cv::dnn::readNetFromDarknet(cfg_file, model_file);
	}

	void detect(cv::Mat img, std::vector<cv::Rect>& boxes)
	{
		std::vector<DResult> results;
		detect(img, results);
		boxes.clear();
		for (const auto& t : results)
		{
			boxes.push_back(t.box);
		}
	}

    void detect(cv::Mat img, std::vector<DResult>& results)
    {
        results.clear();
        cv::Mat blob = preprocess(img);
        if (!blob.empty())
        {
            _net.setInput(blob);
            std::vector<cv::Mat> outs;
            _net.forward(outs, getOutputsNames());
            postprocess(img, outs, results);
        }
    }

private:
	cv::Mat preprocess(cv::Mat img)
	{
		cv::Mat blob;
		if (!_net.empty() && !img.empty())
		{
			blob = cv::dnn::blobFromImage(img, _param.scale,
				cv::Size(_param.width, _param.height),
				_param.mean, _param.swapRB, false);
		}
		return blob;
	}

	void postprocess(cv::Mat frame, std::vector<cv::Mat> outs, std::vector<DResult>& results, int fid = 0)
	{
		static std::vector<int> outLayers = _net.getUnconnectedOutLayers();
		static std::string outLayerType = _net.getLayer(outLayers[0])->type;

		std::vector<int> classIds;
		std::vector<float> confidences;
		std::vector<cv::Rect> boxes;
		if (outLayerType == "Region")
		{
			for (size_t i = 0; i < outs.size(); ++i)
			{
				// Network produces output blob with a shape NxC where N is a number of
				// detected objects and C is a number of classes + 4 where the first 4
				// numbers are [center_x, center_y, width, height]
				int N = outs[i].rows;
				int C = outs[i].cols;
				int batch = outs[i].size[0];
				if (N < 0 || C < 0)
				{
					N = outs[i].size[1];
					C = outs[i].size[2];
				}
				float* data = (float*)outs[i].data + fid * N * C;
				for (int j = 0; j < N; ++j, data += C)
				{
					cv::Mat scores(1, C - 5, CV_32FC1, &data[5]);
					cv::Point classIdPoint;
					double confidence;
					cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
					if (confidence > _param.confThreshold)
					{
						int centerX = (int)(data[0] * frame.cols);
						int centerY = (int)(data[1] * frame.rows);
						int width = (int)(data[2] * frame.cols);
						int height = (int)(data[3] * frame.rows);
						int left = centerX - width / 2;
						int top = centerY - height / 2;

						classIds.push_back(classIdPoint.x);
						confidences.push_back((float)confidence);
						boxes.push_back(cv::Rect(left, top, width, height));
					}
				}
			}
		}

		std::vector<int> indices;
		cv::dnn::NMSBoxes(boxes, confidences, _param.confThreshold, _param.nmsThreshold, indices);
		for (size_t i = 0; i < indices.size(); ++i)
		{
			int idx = indices[i];
			results.push_back(DResult{ classIds[idx], confidences[idx], boxes[idx] });
		}
	}

	std::vector<cv::String> getOutputsNames() const
	{
		static std::vector<cv::String> names;
		if (names.empty())
		{
			std::vector<int> outLayers = _net.getUnconnectedOutLayers();
			std::vector<cv::String> layersNames = _net.getLayerNames();
			names.resize(outLayers.size());
			for (size_t i = 0; i < outLayers.size(); ++i)
				names[i] = layersNames[outLayers[i] - 1];
		}
		return names;
	}

	YOLOParam _param;
	cv::dnn::Net _net;
};
