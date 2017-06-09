#pragma once
#include <vector>
#include <opencv2/core/mat.hpp>
#include "IO/ImageSave.h"
#include <VimbaC/Include/VmbCommonTypes.h>
#include <VimbaCPP/Include/Camera.h>
#include "CameraData.h"
#include <VimbaCPP/Include/VimbaSystem.h>

#include "../namespaces/tg.h"

using namespace tg;

class FrameObserver : public AVT::VmbAPI::IFrameObserver {


public:
	VmbErrorType res;
	AVT::VmbAPI::FramePtr pFrame;
	AVT::VmbAPI::CameraPtr pCamera;

	// In your contructor call the constructor of the base class
	// and pass a camera object
	explicit FrameObserver(AVT::VmbAPI::CameraPtr pCamera) : IFrameObserver(pCamera) {
		this->pCamera = pCamera;
		res = VmbErrorType::VmbErrorOther;
	}

	void FrameReceived(const AVT::VmbAPI::FramePtr pFrame) override {
		VmbFrameStatusType eReceiveStatus;
		if (VmbErrorSuccess == pFrame->GetReceiveStatus(eReceiveStatus)) {
			if (VmbFrameStatusComplete == eReceiveStatus) {
				log_time << "frame received..\n";
				// Put your code here to react on a successfully received frame
			}
			else {
				log_time << "frame failed..\n";
				// Put your code here to react on an unsuccessfully received frame
			}
		}
		// When you are finished copying the frame , re -queue it
		m_pCamera->QueueFrame(pFrame);
	}
};


class CameraFrame {
public:

	//std::vector<cv::Mat> frames;

	// TODO : Capture frame(s) into opencv image matrix,
	// 1) Convert the captured frame into "old" style lplImage format
	// 2) Based on the information, configure a matrix
	// 3) Copy the data from the captured frame into the matrix

	void captureFrames() {

		// should always be checked for VmbErrorSuccess
		auto& sys = AVT::VmbAPI::VimbaSystem::GetInstance();
		AVT::VmbAPI::CameraPtrVector cameras; // A list of known cameras
		AVT::VmbAPI::FramePtrVector frames(3); // A list of frames for streaming. We chose to queue 3 frames.
		AVT::VmbAPI::FeaturePtr pFeature; // Any camera feature
		VmbInt64_t nPLS; // The payload size of one frame
		VmbErrorType err = sys.Startup();
		err = sys.GetCameras(cameras);
		err = cameras[0]->Open(VmbAccessModeFull);
		err = cameras[0]->GetFeatureByName("PayloadSize", pFeature);
		err = pFeature->GetValue(nPLS);
		AVT::VmbAPI::IFrameObserverPtr pObserver(new FrameObserver(cameras.front())); // Our implementation of a frame observer

		for (AVT::VmbAPI::FramePtrVector::iterator iter = frames.begin(); frames.end() != iter; ++iter) {
			(*iter).reset(new AVT::VmbAPI::Frame(nPLS));
			err = (*iter)->RegisterObserver(pObserver);
			err = cameras[0]->AnnounceFrame(*iter);
		}
		err = cameras[0]->StartCapture();
		for (AVT::VmbAPI::FramePtrVector::iterator iter = frames.begin(); frames.end() != iter; ++iter) {
			err = cameras[0]->QueueFrame(*iter);
		}
		err = sys.GetFeatureByName("AcquisitionStart", pFeature);
		err = pFeature->RunCommand();
		// Program runtime ...
		// When finished , tear down the acquisition chain , close the camera and Vimba
		err = sys.GetFeatureByName("AcquisitionStop", pFeature);
		err = pFeature->RunCommand();
		err = cameras[0]->EndCapture();
		err = cameras[0]->FlushQueue();
		err = cameras[0]->RevokeAllFrames();
		err = cameras[0]->Close();
		err = sys.Shutdown();
		// capture stuff

		// copy into iplImg

		//cv::Mat res(iplImg);

		//frame = iplImg;


	}


};
