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

#include "..\Common\StepTimer.h"
#include "ShaderStructures.h"
#include "researchmode\ResearchModeApi.h"
#include "ModelRenderer.h"

namespace BasicHologram
{
using FrameCallback = std::function<void(IResearchModeSensorFrame*, PVOID frameCtx)>;

class SensorReaderThread
{
public:
    SensorReaderThread(IResearchModeSensor* pLLSensor, HANDLE hasData, ResearchModeSensorConsent* pCamAccessConsent);
    ~SensorReaderThread();

    void SetFrameCallBack(FrameCallback frameCallback, PVOID frameCtx);

protected:
    void CameraUpdateThread(HANDLE hasData, ResearchModeSensorConsent *pCamAccessConsent);
    void UpdateTextureFromCameraFrame(IResearchModeSensorFrame* pSensorFrame, std::shared_ptr<Texture2D> texture2D);

    std::function<void(IResearchModeSensorFrame*, PVOID frameCtx)> m_frameCallback;
    PVOID m_frameCtx;

	IResearchModeSensor *m_pRMCameraSensor = nullptr;
	IResearchModeSensorFrame* m_pSensorFrame;

    std::thread m_pCameraUpdateThread;
    bool m_fExit = { false };
};
}
