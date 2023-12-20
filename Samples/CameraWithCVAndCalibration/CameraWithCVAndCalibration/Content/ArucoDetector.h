//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#pragma once

#include "..\Common\DeviceResources.h"
#include "..\Common\StepTimer.h"
#include "researchmode\ResearchModeApi.h"
#include "ShaderStructures.h"
#include "ModelRenderer.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>  // cv::Canny()
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>

namespace BasicHologram
{
    class ArucoDetector
    {
    public:
        ArucoDetector();
        ~ArucoDetector();

        bool GetFirstCenter(float *px, float *py, ResearchModeSensorTimestamp *pTimeStamp);
        static void FrameReadyCallback(IResearchModeSensorFrame* pSensorFrame, PVOID frameCtx);

    protected:
        void FrameProcessing();

        bool m_fExit = { false };
        std::thread m_pFrameUpdateThread;
        HANDLE m_hFrameEvent;

        std::mutex m_frameMutex;
        std::mutex m_cornerMutex;
        std::mutex m_eventMutex;

        IResearchModeSensorFrame* m_pSensorFrame;
        IResearchModeSensorFrame* m_pSensorFrameIn;

        std::vector<std::vector<cv::Point2f>> m_corners;
        std::vector<cv::Point2f> m_centers;
        std::vector<int> m_ids;
        ResearchModeSensorTimestamp m_timeStamp;

        cv::Mat m_cvResultMat;
    };
}
