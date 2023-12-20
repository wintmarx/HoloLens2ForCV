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

#include "pch.h"
#include "ArucoDetector.h"

#include <opencv2/core.hpp>

using namespace BasicHologram;
using namespace DirectX;
using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::UI::Input::Spatial;

namespace {
void ProcessRmFrameWithAruco(IResearchModeSensorFrame* pSensorFrame, cv::Mat& cvResultMat, std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners)
{
    HRESULT hr = S_OK;
    ResearchModeSensorResolution resolution;
    size_t outBufferCount = 0;
    const BYTE* pImage = nullptr;
    IResearchModeSensorVLCFrame* pVLCFrame = nullptr;
    pSensorFrame->GetResolution(&resolution);
    static cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    hr = pSensorFrame->QueryInterface(IID_PPV_ARGS(&pVLCFrame));

    if (SUCCEEDED(hr))
    {
        pVLCFrame->GetBuffer(&pImage, &outBufferCount);

        cv::Mat processed(resolution.Height, resolution.Width, CV_8U, (void*)pImage);
        cv::aruco::detectMarkers(processed, dictionary, corners, ids);

        cvResultMat = processed;

        // if at least one marker detected
        if (ids.size() > 0)
            cv::aruco::drawDetectedMarkers(cvResultMat, corners, ids);
    }

    if (pVLCFrame)
    {
        pVLCFrame->Release();
    }
}
}

ArucoDetector::ArucoDetector() :
    m_pSensorFrame{ nullptr },
    m_pSensorFrameIn{ nullptr },
    m_hFrameEvent{ CreateEvent(NULL, true, false, NULL) },
    m_fExit{ false },
    m_pFrameUpdateThread{ &ArucoDetector::FrameProcessing, this },
    m_timeStamp{}
{
}

ArucoDetector::~ArucoDetector()
{
    m_fExit = true;
    if (m_pFrameUpdateThread.joinable()) {
        m_pFrameUpdateThread.join();
    }
}

void ArucoDetector::FrameReadyCallback(IResearchModeSensorFrame* pSensorFrame, PVOID frameCtx)
{
    ArucoDetector* arucoDetector = (ArucoDetector*)frameCtx;
    std::lock_guard<std::mutex> guard(arucoDetector->m_frameMutex);

    if (arucoDetector->m_pSensorFrameIn)
    {
        arucoDetector->m_pSensorFrameIn->Release();
    }
    arucoDetector->m_pSensorFrameIn = pSensorFrame;

    if (pSensorFrame)
    {
        pSensorFrame->AddRef();
    }

    SetEvent(arucoDetector->m_hFrameEvent);
}

void ArucoDetector::FrameProcessing()
{
    HRESULT hr = S_OK;

    while (!m_fExit)
    {
        WaitForSingleObject(m_hFrameEvent, INFINITE);

        {
            std::lock_guard<std::mutex> guard(m_eventMutex);

            {
                std::lock_guard<std::mutex> frameguard(m_frameMutex);

                if (m_pSensorFrameIn == nullptr)
                {
                    return;
                }
                if (m_pSensorFrame)
                {
                    m_pSensorFrame->Release();
                }
                m_pSensorFrame = m_pSensorFrameIn;
                m_pSensorFrameIn = nullptr;
            }

            {
                std::lock_guard<std::mutex> guard(m_cornerMutex);

                m_corners.clear();
                m_ids.clear();

                ProcessRmFrameWithAruco(m_pSensorFrame, m_cvResultMat, m_ids, m_corners);

                m_centers.clear();

                for (int i = 0; i < m_corners.size(); i++)
                {
                    float sumx = 0.0f;
                    float sumy = 0.0f;
                    cv::Point2f center;

                    for (int j = 0; j < m_corners[i].size(); j++)
                    {
                        sumx += m_corners[i][j].x;
                        sumy += m_corners[i][j].y;
                    }

                    center.x = sumx / 4.0f;
                    center.y = sumy / 4.0f;

                    m_centers.push_back(center);
                }

                m_pSensorFrame->GetTimeStamp(&m_timeStamp);
            }

            ResetEvent(m_hFrameEvent);
        }
    }
}

bool ArucoDetector::GetFirstCenter(float* px, float* py, ResearchModeSensorTimestamp* pTimeStamp)
{
    std::lock_guard<std::mutex> guard(m_cornerMutex);

    if (m_centers.size() >= 1)
    {
        *px = m_centers[0].x;
        *py = m_centers[0].y;

        return true;
    }

    return false;
}
