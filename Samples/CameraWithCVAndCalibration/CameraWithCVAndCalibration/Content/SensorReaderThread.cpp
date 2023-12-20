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
#include "SensorReaderThread.h"

using namespace BasicHologram;
using namespace DirectX;
using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::UI::Input::Spatial;

SensorReaderThread::SensorReaderThread(IResearchModeSensor* pLLSensor, HANDLE hasData, ResearchModeSensorConsent* pCamAccessConsent):
    m_pRMCameraSensor{ pLLSensor },
    m_pSensorFrame{nullptr},
    m_frameCallback{nullptr},
    m_pCameraUpdateThread{&SensorReaderThread::CameraUpdateThread, this, hasData, pCamAccessConsent },
    m_frameCtx{nullptr}
{
    m_pRMCameraSensor->AddRef();
}

SensorReaderThread::~SensorReaderThread()
{
    m_fExit = true;
    if (m_pCameraUpdateThread.joinable()) {
        m_pCameraUpdateThread.join();
    }

    if (m_pRMCameraSensor)
    {
        m_pRMCameraSensor->CloseStream();
        m_pRMCameraSensor->Release();
    }
}

void SensorReaderThread::SetFrameCallBack(FrameCallback frameCallback, PVOID frameCtx)
{
    m_frameCallback = frameCallback;
    m_frameCtx = frameCtx;
}

void SensorReaderThread::CameraUpdateThread(HANDLE hasData, ResearchModeSensorConsent *pCamAccessConsent)
{
    HRESULT hr = S_OK;

    if (hasData != nullptr)
    {
        DWORD waitResult = WaitForSingleObject(hasData, INFINITE);

        if (waitResult == WAIT_OBJECT_0)
        {
            switch (*pCamAccessConsent)
            {
            case ResearchModeSensorConsent::Allowed:
                OutputDebugString(L"Access is granted");
                break;
            case ResearchModeSensorConsent::DeniedBySystem:
                OutputDebugString(L"Access is denied by the system");
                hr = E_ACCESSDENIED;
                break;
            case ResearchModeSensorConsent::DeniedByUser:
                OutputDebugString(L"Access is denied by the user");
                hr = E_ACCESSDENIED;
                break;
            case ResearchModeSensorConsent::NotDeclaredByApp:
                OutputDebugString(L"Capability is not declared in the app manifest");
                hr = E_ACCESSDENIED;
                break;
            case ResearchModeSensorConsent::UserPromptRequired:
                OutputDebugString(L"Capability user prompt required");
                hr = E_ACCESSDENIED;
                break;
            default:
                OutputDebugString(L"Access is denied by the system");
                hr = E_ACCESSDENIED;
                break;
            }
        }
        else
        {
            hr = E_UNEXPECTED;
        }
    }

    if (FAILED(hr))
    {
        return;
    }

    winrt::check_hresult(m_pRMCameraSensor->OpenStream());

    while (!m_fExit && m_pRMCameraSensor)
    {
        static int gFrameCount = 0;
        HRESULT hr = S_OK;
        IResearchModeSensorFrame* pSensorFrame = nullptr;

        winrt::check_hresult(m_pRMCameraSensor->GetNextBuffer(&pSensorFrame));

        if (m_frameCallback)
        {
            m_frameCallback(pSensorFrame, m_frameCtx);
        }

        if (m_pSensorFrame)
        {
            m_pSensorFrame->Release();
        }

        m_pSensorFrame = pSensorFrame;
    }

    if (m_pRMCameraSensor)
    {
        m_pRMCameraSensor->CloseStream();
    }
}

BYTE ConvertDepthPixel(USHORT v, BYTE bSigma, USHORT mask, USHORT maxshort, const int vmin, const int vmax)
{
    if ((mask != 0) && (bSigma & mask) > 0)
    {
        v = 0;
    }

    if ((maxshort != 0) && (v > maxshort))
    {
        v = 0;
    }
    
    float colorValue = 0.0f;
    if (v <= vmin)
    {
        colorValue = 0.0f;
    }
    else if (v >= vmax)
    {
        colorValue = 1.0f;
    }
    else
    {
        colorValue = (float)(v - vmin) / (float)(vmax - vmin);
    }

    return (BYTE)(colorValue * 255);
}


void SensorReaderThread::UpdateTextureFromCameraFrame(IResearchModeSensorFrame* pSensorFrame, std::shared_ptr<Texture2D> texture2D)
{
	HRESULT hr = S_OK;

    ResearchModeSensorResolution resolution;
	IResearchModeSensorVLCFrame *pVLCFrame = nullptr;
    IResearchModeSensorDepthFrame *pDepthFrame = nullptr;
	size_t outBufferCount = 0;
    const BYTE *pImage = nullptr;

    pSensorFrame->GetResolution(&resolution);

	hr = pSensorFrame->QueryInterface(IID_PPV_ARGS(&pVLCFrame));

    if (FAILED(hr))
    {
        hr = pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));
    }
    
	if (pVLCFrame)
	{
		pVLCFrame->GetBuffer(&pImage, &outBufferCount);

        void* mappedTexture =
            texture2D->MapCPUTexture<void>(
                D3D11_MAP_WRITE /* mapType */);

        for (UINT i = 0; i < resolution.Height; i++)
        {
            for (UINT j = 0; j < resolution.Width; j++)
            {
                UINT32 pixel = 0;
                BYTE inputPixel = pImage[resolution.Width * i + j];
                pixel = inputPixel | (inputPixel << 8) | (inputPixel << 16);

                if (m_pRMCameraSensor->GetSensorType() == LEFT_FRONT)
                {
                    *((UINT32*)(mappedTexture)+((texture2D->GetRowPitch() / 4) * i + (resolution.Width - j - 1))) = pixel;
                }
                else if (m_pRMCameraSensor->GetSensorType() == RIGHT_FRONT)
                {
                    *((UINT32*)(mappedTexture)+((texture2D->GetRowPitch() / 4) * (resolution.Height - i - 1) + j)) = pixel;
                }
                else
                {
                    *((UINT32*)(mappedTexture)+((texture2D->GetRowPitch() / 4) * i + j)) = pixel;
                }
            }
        }
	}

    if (pDepthFrame)
    {
        int maxClampDepth = 0;
        USHORT maxshort = 0;
        USHORT mask = 0;
        const BYTE *pSigma = nullptr;

        if (m_pRMCameraSensor->GetSensorType() == DEPTH_LONG_THROW)
        {
            mask = 0x80;
            maxClampDepth = 4000;
        }
        else if (m_pRMCameraSensor->GetSensorType() == DEPTH_AHAT)
        {
            mask = 0x0;
            maxClampDepth = 1000;
            maxshort = 4090;
        }

        if (m_pRMCameraSensor->GetSensorType() == DEPTH_LONG_THROW)
        {
            hr = pDepthFrame->GetSigmaBuffer(&pSigma, &outBufferCount);
        }

        const UINT16 *pDepth = nullptr;
        pDepthFrame->GetBuffer(&pDepth, &outBufferCount);

        void* mappedTexture =
            texture2D->MapCPUTexture<void>(
                D3D11_MAP_WRITE /* mapType */);

        for (UINT i = 0; i < resolution.Height; i++)
        {
            for (UINT j = 0; j < resolution.Width; j++)
            {
                UINT32 pixel = 0;
                //BYTE inputPixel = pImage[resolution.Width * i + j];
                BYTE inputPixel = ConvertDepthPixel(
                    pDepth[resolution.Width * i + j],
                    pSigma[resolution.Width * i + j],
                    mask,
                    maxshort,
                    0,
                    maxClampDepth);

                pixel = inputPixel | (inputPixel << 8) | (inputPixel << 16);

                *((UINT32*)(mappedTexture)+((texture2D->GetRowPitch() / 4) * i + j)) = pixel;
            }
        }
    }
    
	if (pVLCFrame)
	{
		pVLCFrame->Release();
	}
    
	if (pDepthFrame)
	{
		pDepthFrame->Release();
	}

    texture2D->UnmapCPUTexture();
    texture2D->CopyCPU2GPU();
}
