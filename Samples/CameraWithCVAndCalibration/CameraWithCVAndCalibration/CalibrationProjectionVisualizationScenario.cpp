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
#include "CalibrationProjectionVisualizationScenario.h"
#include "Common\DirectXHelper.h"

extern "C"
HMODULE LoadLibraryA(
    LPCSTR lpLibFileName
);

using namespace BasicHologram;
using namespace concurrency;
using namespace Microsoft::WRL;
using namespace std::placeholders;
using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Gaming::Input;
using namespace winrt::Windows::Graphics::Holographic;
using namespace winrt::Windows::Graphics::DirectX::Direct3D11;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::UI::Input::Spatial;

static ResearchModeSensorConsent camAccessCheck;
static HANDLE camConsentGiven;

CalibrationProjectionVisualizationScenario::CalibrationProjectionVisualizationScenario(std::shared_ptr<DX::DeviceResources> const& deviceResources) :
    Scenario(deviceResources)
{
}

CalibrationProjectionVisualizationScenario::~CalibrationProjectionVisualizationScenario()
{
    if (m_pLFCameraSensor)
    {
        m_pLFCameraSensor->Release();
    }

    if (m_pSensorDevice)
    {
        m_pSensorDevice->EnableEyeSelection();
        m_pSensorDevice->Release();
    }
}

void CalibrationProjectionVisualizationScenario::CamAccessOnComplete(ResearchModeSensorConsent consent)
{
    camAccessCheck = consent;
    SetEvent(camConsentGiven);
}

void CalibrationProjectionVisualizationScenario::IntializeSensors()
{
    HRESULT hr = S_OK;
    size_t sensorCount = 0;
    camConsentGiven = CreateEvent(nullptr, true, false, nullptr);

    HMODULE hrResearchMode = LoadLibraryA("ResearchModeAPI");
    if (hrResearchMode)
    {
        typedef HRESULT(__cdecl* PFN_CREATEPROVIDER) (IResearchModeSensorDevice** ppSensorDevice);
        PFN_CREATEPROVIDER pfnCreate = reinterpret_cast<PFN_CREATEPROVIDER>(GetProcAddress(hrResearchMode, "CreateResearchModeSensorDevice"));
        if (pfnCreate)
        {
            winrt::check_hresult(pfnCreate(&m_pSensorDevice));
        }
        else
        {
            winrt::check_hresult(E_INVALIDARG);
        }
    }

    winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&m_pSensorDeviceConsent)));
    winrt::check_hresult(m_pSensorDeviceConsent->RequestCamAccessAsync(CalibrationProjectionVisualizationScenario::CamAccessOnComplete));

    m_pSensorDevice->DisableEyeSelection();

    winrt::check_hresult(m_pSensorDevice->GetSensorCount(&sensorCount));
    m_sensorDescriptors.resize(sensorCount);

    winrt::check_hresult(m_pSensorDevice->GetSensorDescriptors(m_sensorDescriptors.data(), m_sensorDescriptors.size(), &sensorCount));

    for (auto sensorDescriptor : m_sensorDescriptors)
    {
        IResearchModeSensor *pSensor = nullptr;
        IResearchModeCameraSensor *pCameraSensor = nullptr;

        if (sensorDescriptor.sensorType == LEFT_FRONT)
        {
            winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_pLFCameraSensor));

            winrt::check_hresult(m_pLFCameraSensor->QueryInterface(IID_PPV_ARGS(&pCameraSensor)));

            winrt::check_hresult(pCameraSensor->GetCameraExtrinsicsMatrix(&m_LFCameraPose));

            DirectX::XMFLOAT4 zeros = DirectX::XMFLOAT4(0.0f, 0.0f, 0.0f, 1.0f);

            DirectX::XMMATRIX cameraPose = XMLoadFloat4x4(&m_LFCameraPose);
            DirectX::XMMATRIX cameraRotation = cameraPose;
            cameraRotation.r[3] = DirectX::XMLoadFloat4(&zeros);
            XMStoreFloat4x4(&m_LFCameraRotation, cameraRotation);

            DirectX::XMVECTOR det = XMMatrixDeterminant(cameraRotation);
            XMStoreFloat4(&m_LFRotDeterminant, det);

            m_lf_inv_extr = DirectX::XMMatrixInverse(&det, cameraPose);
        }

        if (sensorDescriptor.sensorType == RIGHT_FRONT)
        {
            winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_pRFCameraSensor));

            winrt::check_hresult(m_pRFCameraSensor->QueryInterface(IID_PPV_ARGS(&pCameraSensor)));

            winrt::check_hresult(pCameraSensor->GetCameraExtrinsicsMatrix(&m_RFCameraPose));

            DirectX::XMFLOAT4 zeros = DirectX::XMFLOAT4(0.0f, 0.0f, 0.0f, 1.0f);

            DirectX::XMMATRIX cameraPose = XMLoadFloat4x4(&m_RFCameraPose);
            DirectX::XMMATRIX cameraRotation = cameraPose;
            cameraRotation.r[3] = DirectX::XMLoadFloat4(&zeros);
            XMStoreFloat4x4(&m_RFCameraRotation, cameraRotation);

            DirectX::XMVECTOR det = XMMatrixDeterminant(cameraRotation);
            XMStoreFloat4(&m_RFRotDeterminant, det);
        }
    }
}

void CalibrationProjectionVisualizationScenario::UpdateState()
{
}

void CalibrationProjectionVisualizationScenario::IntializeSensorFrameModelRendering()
{
    HRESULT hr = S_OK;

    DirectX::XMMATRIX cameraNodeToRigPoseInverted;
    DirectX::XMMATRIX cameraNodeToRigPose;
    DirectX::XMVECTOR det;
    //float xy[2] = {0};
    //float uv[2];

    //DirectX::XMMATRIX groupRotation = DirectX::XMMatrixRotationAxis(DirectX::XMVectorSet(1.f, 0.f, 0.f, 0.f), -DirectX::XM_PIDIV2/2);
    //groupRotation = groupRotation * DirectX::XMMatrixRotationAxis(DirectX::XMVectorSet(0.f, 1.f, 0.f, 0.f), (-DirectX::XM_PIDIV2 / 2 + (DirectX::XM_PIDIV2 / 4 * m_state)));

    //{
    //    IResearchModeCameraSensor *pCameraSensor = nullptr;

    //    // Initialize the sample hologram.
    //    auto axisRFRenderer = std::make_shared<XYZAxisModel>(m_deviceResources, 0.05f, 0.001f);

    //    cameraNodeToRigPose = DirectX::XMLoadFloat4x4(&m_RFCameraPose);
    //    det = XMMatrixDeterminant(cameraNodeToRigPose);
    //    cameraNodeToRigPoseInverted = DirectX::XMMatrixInverse(&det, cameraNodeToRigPose);

    //    axisRFRenderer->SetGroupScaleFactor(1.0);
    //    axisRFRenderer->SetModelTransform(cameraNodeToRigPoseInverted);

    //    axisRFRenderer->SetColors(DirectX::XMFLOAT3(1.0f, 0.0f, 0.0f),
    //        DirectX::XMFLOAT3(0.0f, 1.0f, 0.0f),
    //        DirectX::XMFLOAT3(0.0f, 0.0f, 1.0f));
    //    m_modelRenderers.push_back(axisRFRenderer);

    //    winrt::check_hresult(m_pRFCameraSensor->QueryInterface(IID_PPV_ARGS(&pCameraSensor)));

    //    uv[0] = 0.0f;
    //    uv[1] = 0.0f;

    //    pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);

    //    auto vectorOriginRenderer = std::make_shared<VectorModel>(m_deviceResources, 0.1f, 0.0005f, DirectX::XMFLOAT3(xy[0], xy[1], 1.0f));

    //    vectorOriginRenderer->SetGroupScaleFactor(1.0);
    //    vectorOriginRenderer->SetModelTransform(cameraNodeToRigPoseInverted);
    //    vectorOriginRenderer->SetColor(DirectX::XMFLOAT3(1.0f, 1.0f, 1.0f));
    //    m_modelRenderers.push_back(vectorOriginRenderer);

    //    uv[0] = 640.0f;
    //    uv[1] = 0.0f;

    //    pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);

    //    vectorOriginRenderer = std::make_shared<VectorModel>(m_deviceResources, 0.1f, 0.0005f, DirectX::XMFLOAT3(xy[0], xy[1], 1.0f));

    //    vectorOriginRenderer->SetGroupScaleFactor(1.0);
    //    vectorOriginRenderer->SetModelTransform(cameraNodeToRigPoseInverted);
    //    vectorOriginRenderer->SetColor(DirectX::XMFLOAT3(1.0f, 1.0f, 1.0f));
    //    m_modelRenderers.push_back(vectorOriginRenderer);

    //    uv[0] = 640.0f;
    //    uv[1] = 480.0f;

    //    pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);

    //    vectorOriginRenderer = std::make_shared<VectorModel>(m_deviceResources, 0.1f, 0.0005f, DirectX::XMFLOAT3(xy[0], xy[1], 1.0f));

    //    vectorOriginRenderer->SetGroupScaleFactor(1.0);
    //    vectorOriginRenderer->SetModelTransform(cameraNodeToRigPoseInverted);
    //    vectorOriginRenderer->SetColor(DirectX::XMFLOAT3(1.0f, 1.0f, 1.0f));
    //    m_modelRenderers.push_back(vectorOriginRenderer);

    //    uv[0] = 0.0f;
    //    uv[1] = 480.0f;

    //    pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);

    //    vectorOriginRenderer = std::make_shared<VectorModel>(m_deviceResources, 0.1f, 0.0005f, DirectX::XMFLOAT3(xy[0], xy[1], 1.0f));

    //    vectorOriginRenderer->SetGroupScaleFactor(1.0);
    //    vectorOriginRenderer->SetModelTransform(cameraNodeToRigPoseInverted);
    //    vectorOriginRenderer->SetColor(DirectX::XMFLOAT3(1.0f, 1.0f, 1.0f));
    //    m_modelRenderers.push_back(vectorOriginRenderer);

    //    uv[0] = 640.0f / 2;
    //    uv[1] = 480.0f / 2;

    //    pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);

    //    vectorOriginRenderer = std::make_shared<VectorModel>(m_deviceResources, 0.6f, 0.0005f, DirectX::XMFLOAT3(xy[0], xy[1], 1.0f));

    //    vectorOriginRenderer->SetGroupScaleFactor(1.0);
    //    vectorOriginRenderer->SetModelTransform(cameraNodeToRigPoseInverted);
    //    vectorOriginRenderer->SetGroupTransform(groupRotation);
    //    vectorOriginRenderer->SetColor(DirectX::XMFLOAT3(1.0f, 1.0f, 1.0f));
    //    m_modelRenderers.push_back(vectorOriginRenderer);

    //    m_rayRight = vectorOriginRenderer;

    //    pCameraSensor->Release();
    //}

    //{
    //    IResearchModeCameraSensor *pCameraSensor = nullptr;

    //    // Initialize the sample hologram.
    //    auto axisLFRenderer = std::make_shared<XYZAxisModel>(m_deviceResources, 0.1f, 0.001f);

    //    cameraNodeToRigPose = DirectX::XMLoadFloat4x4(&m_LFCameraPose);
    //    det = XMMatrixDeterminant(cameraNodeToRigPose);
    //    cameraNodeToRigPoseInverted = DirectX::XMMatrixInverse(&det, cameraNodeToRigPose);

    //    //axisLFRenderer->SetOffset(offset);
    //    axisLFRenderer->SetGroupScaleFactor(1.0);
    //    axisLFRenderer->SetModelTransform(cameraNodeToRigPoseInverted);

    //    axisLFRenderer->SetColors(DirectX::XMFLOAT3(1.0f, 0.0f, 0.0f),
    //        DirectX::XMFLOAT3(0.0f, 1.0f, 0.0f),
    //        DirectX::XMFLOAT3(0.0f, 0.0f, 1.0f));
    //    m_modelRenderers.push_back(axisLFRenderer);


    //    winrt::check_hresult(m_pLFCameraSensor->QueryInterface(IID_PPV_ARGS(&pCameraSensor)));

    //    uv[0] = 0.0f;
    //    uv[1] = 0.0f;

    //    pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);

    //    auto vectorOriginRenderer = std::make_shared<VectorModel>(m_deviceResources, 0.1f, 0.0005f, DirectX::XMFLOAT3(xy[0], xy[1], 1.0f));

    //    vectorOriginRenderer->SetGroupScaleFactor(1.0);
    //    vectorOriginRenderer->SetModelTransform(cameraNodeToRigPoseInverted);
    //    vectorOriginRenderer->SetColor(DirectX::XMFLOAT3(1.0f, 1.0f, 1.0f));
    //    m_modelRenderers.push_back(vectorOriginRenderer);

    //    uv[0] = 640.0f;
    //    uv[1] = 0.0f;

    //    pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);

    //    vectorOriginRenderer = std::make_shared<VectorModel>(m_deviceResources, 0.1f, 0.0005f, DirectX::XMFLOAT3(xy[0], xy[1], 1.0f));

    //    vectorOriginRenderer->SetGroupScaleFactor(1.0);
    //    vectorOriginRenderer->SetModelTransform(cameraNodeToRigPoseInverted);
    //    vectorOriginRenderer->SetColor(DirectX::XMFLOAT3(1.0f, 1.0f, 1.0f));
    //    m_modelRenderers.push_back(vectorOriginRenderer);

    //    uv[0] = 640.0f;
    //    uv[1] = 480.0f;

    //    pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);

    //    vectorOriginRenderer = std::make_shared<VectorModel>(m_deviceResources, 0.1f, 0.0005f, DirectX::XMFLOAT3(xy[0], xy[1], 1.0f));

    //    vectorOriginRenderer->SetGroupScaleFactor(1.0);
    //    vectorOriginRenderer->SetModelTransform(cameraNodeToRigPoseInverted);
    //    vectorOriginRenderer->SetColor(DirectX::XMFLOAT3(1.0f, 1.0f, 1.0f));
    //    m_modelRenderers.push_back(vectorOriginRenderer);

    //    uv[0] = 0.0f;
    //    uv[1] = 480.0f;

    //    pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);

    //    vectorOriginRenderer = std::make_shared<VectorModel>(m_deviceResources, 0.1f, 0.0005f, DirectX::XMFLOAT3(xy[0], xy[1], 1.0f));

    //    vectorOriginRenderer->SetGroupScaleFactor(1.0);
    //    vectorOriginRenderer->SetModelTransform(cameraNodeToRigPoseInverted);
    //    vectorOriginRenderer->SetColor(DirectX::XMFLOAT3(1.0f, 1.0f, 1.0f));
    //    m_modelRenderers.push_back(vectorOriginRenderer);

    //    uv[0] = 640.0f / 2;
    //    uv[1] = 480.0f / 2;

    //    pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);

    //    vectorOriginRenderer = std::make_shared<VectorModel>(m_deviceResources, 0.6f, 0.0005f, DirectX::XMFLOAT3(xy[0], xy[1], 1.0f));

    //    vectorOriginRenderer->SetGroupScaleFactor(1.0);
    //    vectorOriginRenderer->SetModelTransform(cameraNodeToRigPoseInverted);
    //    vectorOriginRenderer->SetGroupTransform(groupRotation);
    //    vectorOriginRenderer->SetColor(DirectX::XMFLOAT3(1.0f, 1.0f, 1.0f));
    //    m_modelRenderers.push_back(vectorOriginRenderer);

    //    m_rayLeft = vectorOriginRenderer;

    //    pCameraSensor->Release();
    //}

    auto vectorOriginRenderer = std::make_shared<VectorModel>(m_deviceResources, 0.01f, 0.01f, DirectX::XMFLOAT3(1, 0, 0), DirectX::XMFLOAT3(1.0f, 0.0f, 0.0f));
    vectorOriginRenderer->SetColor(DirectX::XMFLOAT3(1.0f, 0.0f, 0.0f));
    m_modelRenderers.push_back(vectorOriginRenderer);

    vectorOriginRenderer = std::make_shared<VectorModel>(m_deviceResources, 0.01f, 0.01f, DirectX::XMFLOAT3(0, 1, 0), DirectX::XMFLOAT3(0.0f, 1.0f, 0.0f));
    vectorOriginRenderer->SetColor(DirectX::XMFLOAT3(0.0f, 1.0f, 0.0f));
    m_modelRenderers.push_back(vectorOriginRenderer);

    vectorOriginRenderer = std::make_shared<VectorModel>(m_deviceResources, 0.01f, 0.01f, DirectX::XMFLOAT3(0, 0, 1), DirectX::XMFLOAT3(0.0f, 0.0f, 1.0f));
    vectorOriginRenderer->SetColor(DirectX::XMFLOAT3(0.0f, 0.0f, 1.0f));
    m_modelRenderers.push_back(vectorOriginRenderer);

    vectorOriginRenderer = std::make_shared<VectorModel>(m_deviceResources, 0.01f, 0.01f, DirectX::XMFLOAT3(0, 0, 0), DirectX::XMFLOAT3(0, 0, 0));
    vectorOriginRenderer->SetColor(DirectX::XMFLOAT3(1.0f, 1.0f, 1.0f));
    m_modelRenderers.push_back(vectorOriginRenderer);

    auto test_axis = std::make_shared<XYZAxisModel>(m_deviceResources, 1.0f, 0.0005f);
    test_axis->SetPosition({ 1, 1, 1 });
    m_modelRenderers.push_back(test_axis);

    m_aruco_axis_model = std::make_shared<VectorModel>(m_deviceResources, 0.01f, 0.01f, DirectX::XMFLOAT3(0, 0, 0), DirectX::XMFLOAT3(0, 0, 0));
    m_modelRenderers.push_back(m_aruco_axis_model);
}

void CalibrationProjectionVisualizationScenario::InitializeArucoRendering()
{
    if (m_pLFCameraSensor)
    {
        m_sensorReaderLeft = std::make_shared<SensorReaderThread>(m_pLFCameraSensor, camConsentGiven, &camAccessCheck);
        m_arucoDetectorLeft = std::make_shared<ArucoDetector>();
        m_sensorReaderLeft->SetFrameCallBack(ArucoDetector::FrameReadyCallback, m_arucoDetectorLeft.get());
    }

    if (m_pRFCameraSensor)
    {
        m_sensorReaderRight = std::make_shared<SensorReaderThread>(m_pRFCameraSensor, camConsentGiven, &camAccessCheck);
        m_arucoDetectorRight = std::make_shared<ArucoDetector>();
        m_sensorReaderRight->SetFrameCallBack(ArucoDetector::FrameReadyCallback, m_arucoDetectorRight.get());
    }
}

void CalibrationProjectionVisualizationScenario::IntializeModelRendering()
{
    IntializeSensorFrameModelRendering();
    InitializeArucoRendering();
}

void CalibrationProjectionVisualizationScenario::PositionHologram(winrt::Windows::UI::Input::Spatial::SpatialPointerPose const& pointerPose, const DX::StepTimer& timer)
{
    // When a Pressed gesture is detected, the sample hologram will be repositioned
    // two meters in front of the user.
    for (int i = 0; i < m_modelRenderers.size(); i++)
    {
        m_modelRenderers[i]->PositionHologram(pointerPose, timer);
    }
}

void CalibrationProjectionVisualizationScenario::PositionHologramNoSmoothing(winrt::Windows::UI::Input::Spatial::SpatialPointerPose const& pointerPose)
{
    // When a Pressed gesture is detected, the sample hologram will be repositioned
    // two meters in front of the user.
    for (int i = 0; i < m_modelRenderers.size(); i++)
    {
        m_modelRenderers[i]->PositionHologramNoSmoothing(pointerPose);
    }
}

void CalibrationProjectionVisualizationScenario::compensateView(const HolographicCameraPose& cameraPose, const SpatialCoordinateSystem& coordinateSystem) {
    auto viewTransformContainer = cameraPose.TryGetViewTransform(coordinateSystem);

    bool viewTransformAcquired = viewTransformContainer != nullptr;
    if (viewTransformAcquired)
    {
        HolographicStereoTransform viewCoordinateSystemTransform = viewTransformContainer.Value();
        auto view = viewCoordinateSystemTransform.Left;

        OutputDebugStringA(std::format("left view\n{}, {}, {}, {},\n{}, {}, {}, {},\n{}, {}, {}, {},\n{}, {}, {}, {}\n",
             view.m11,
             view.m12,
             view.m13,
             view.m14,
             view.m21,
             view.m22,
             view.m23,
             view.m24,
             view.m31,
             view.m32,
             view.m33,
             view.m34,
             view.m41,
             view.m42,
             view.m43,
             view.m44
             ).c_str());

        auto rotate = view;
        rotate.m41 = 0;
        rotate.m42 = 0;
        rotate.m43 = 0;

        OutputDebugStringA(std::format("left rotate\n{}, {}, {}, {},\n{}, {}, {}, {},\n{}, {}, {}, {},\n{}, {}, {}, {}\n",
            rotate.m11,
            rotate.m12,
            rotate.m13,
            rotate.m14,
            rotate.m21,
            rotate.m22,
            rotate.m23,
            rotate.m24,
            rotate.m31,
            rotate.m32,
            rotate.m33,
            rotate.m34,
            rotate.m41,
            rotate.m42,
            rotate.m43,
            rotate.m44
        ).c_str());

        auto xm_rotate = DirectX::XMLoadFloat4x4(&rotate);
        xm_rotate = DirectX::XMMatrixTranspose(xm_rotate);

        auto translate = view;
        translate.m11 = 0;
        translate.m12 = 0;
        translate.m13 = 0;
        translate.m21 = 0;
        translate.m22 = 0;
        translate.m23 = 0;
        translate.m31 = 0;
        translate.m32 = 0;
        translate.m33 = 0;

        auto inv_translate = translate;

        translate.m41 = -translate.m41;
        translate.m42 = -translate.m42;
        translate.m43 = -translate.m43;

        OutputDebugStringA(std::format("left translate\n{}, {}, {}, {},\n{}, {}, {}, {},\n{}, {}, {}, {},\n{}, {}, {}, {}\n",
            translate.m11,
            translate.m12,
            translate.m13,
            translate.m14,
            translate.m21,
            translate.m22,
            translate.m23,
            translate.m24,
            translate.m31,
            translate.m32,
            translate.m33,
            translate.m34,
            translate.m41,
            translate.m42,
            translate.m43,
            translate.m44
        ).c_str());

        auto xm_translate = DirectX::XMLoadFloat4x4(&translate);

        auto xm_inv_translate = DirectX::XMLoadFloat4x4(&inv_translate);

        auto final_transform = xm_translate * xm_rotate * xm_inv_translate;

        m_aruco_axis_model->SetModelTransform(xm_rotate * xm_translate);
    }
}

void CalibrationProjectionVisualizationScenario::UpdateModels(DX::StepTimer &timer, const SpatialLocation& spatial_location)
{
    float x[2];
    float uv[2];
    ResearchModeSensorTimestamp timeStamp;

    if (m_arucoDetectorLeft->GetFirstCenter(uv, uv + 1, &timeStamp))
    {
        HRESULT hr = S_OK;
        IResearchModeCameraSensor* pCameraSensor = nullptr;

        winrt::check_hresult(m_pLFCameraSensor->QueryInterface(IID_PPV_ARGS(&pCameraSensor)));

        pCameraSensor->MapImagePointToCameraUnitPlane(uv, x);

        auto ori = spatial_location.Orientation();
        auto xm_rotation = DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadQuaternion(&ori));

        float4x4 rot;
        DirectX::XMStoreFloat4x4(&rot, xm_rotation);

        OutputDebugStringA(std::format("left rotate\n{}, {}, {}, {},\n{}, {}, {}, {},\n{}, {}, {}, {},\n{}, {}, {}, {}\n",
            rot.m11,
            rot.m12,
            rot.m13,
            rot.m14,
            rot.m21,
            rot.m22,
            rot.m23,
            rot.m24,
            rot.m31,
            rot.m32,
            rot.m33,
            rot.m34,
            rot.m41,
            rot.m42,
            rot.m43,
            rot.m44
        ).c_str());

        float4 inv_pos{ spatial_location.Position(), 1.0 };

        auto xm_tr_vec = DirectX::XMLoadFloat4(&inv_pos);
        auto xm_translate = DirectX::XMMatrixTranslationFromVector(xm_tr_vec);

        OutputDebugStringA(std::format("holo pos {}, {}, {}\n",
            inv_pos.x,
            inv_pos.y,
            inv_pos.z
        ).c_str());

        //auto final_transform = xm_translate * xm_rotate * xm_inv_translate;

        //m_aruco_axis_model->SetModelTransform(xm_translate * xm_rotation * m_lf_inv_extr);

        float z = -1;
        float4 det_pos{ x[1] * z, x[0] * z, z, 1. };

        auto xm_det_pos = DirectX::XMLoadFloat4(&det_pos);
        xm_det_pos = DirectX::XMVector4Transform(xm_det_pos, xm_rotation * xm_translate * m_lf_inv_extr);

        //auto det_pos = DirectX::XMVector4Transform(DirectX::XMVECTOR{ x[1], x[0], 1. }, cameraNodeToRigPoseInverted);

        float4 pos_world;
        DirectX::XMStoreFloat4(&pos_world, xm_det_pos);

        float4 cur_pos{ m_aruco_axis_model->GetPosition(), 1. };

        float alpha = 0.5;
        pos_world = pos_world * alpha + cur_pos * (1. - alpha);

        m_aruco_axis_model->SetPosition({ pos_world.x, pos_world.y, pos_world.z });

        OutputDebugStringA(std::format("aruco pos world {}, {}, {}\n",
            pos_world.x,
            pos_world.y,
            pos_world.z
        ).c_str());

        //m_aruco_axis_model->SetModelTransform(cameraNodeToRigPoseInverted);

        //m_aruco_axis_model->EnableRendering();

         //OutputDebugStringA(std::format("left position\n{}, {}, {}, {},\n{}, {}, {}, {},\n{}, {}, {}, {},\n{}, {}, {}, {}\n",
         //    extr._11,
         //    extr._12,
         //    extr._13,
         //    extr._14,
         //    extr._21,
         //    extr._22,
         //    extr._23,
         //    extr._24,
         //    extr._31,
         //    extr._32,
         //    extr._33,
         //    extr._34,
         //    extr._41,
         //    extr._42,
         //    extr._43,
         //    extr._44
         //    ).c_str());

         //OutputDebugStringA(std::format("left rotation\n{}, {}, {}, {},\n{}, {}, {}, {},\n{}, {}, {}, {},\n{}, {}, {}, {}\n",
         //    m_LFCameraRotation._11,
         //    m_LFCameraRotation._12,
         //    m_LFCameraRotation._13,
         //    m_LFCameraRotation._14,
         //    m_LFCameraRotation._21,
         //    m_LFCameraRotation._22,
         //    m_LFCameraRotation._23,
         //    m_LFCameraRotation._24,
         //    m_LFCameraRotation._31,
         //    m_LFCameraRotation._32,
         //    m_LFCameraRotation._33,
         //    m_LFCameraRotation._34,
         //    m_LFCameraRotation._41,
         //    m_LFCameraRotation._42,
         //    m_LFCameraRotation._43,
         //    m_LFCameraRotation._44
         //).c_str());

         pCameraSensor->Release();

         OutputDebugStringA(std::format("left aruco corner {{{}, {}}}, mapped {{{}, {}}}\n", uv[0], uv[1], x[0], x[1]).c_str());
    }
    else
    {
        //m_aruco_axis_model->DisableRendering();
    }

    if (m_arucoDetectorRight->GetFirstCenter(uv, uv + 1, &timeStamp))
    {
        HRESULT hr = S_OK;

        IResearchModeCameraSensor *pCameraSensor = nullptr;
        
        winrt::check_hresult(m_pRFCameraSensor->QueryInterface(IID_PPV_ARGS(&pCameraSensor)));

        pCameraSensor->MapImagePointToCameraUnitPlane(uv, x);

        //m_rayRight->SetDirection(DirectX::XMFLOAT3(x[0], x[1], 1.0f));
        //m_rayRight->EnableRendering();

        pCameraSensor->Release();

        OutputDebugStringA(std::format("right aruco corner {{{}, {}}}, mapped {{{}, {}}}\n", uv[0], uv[1], x[0], x[1]).c_str());
    }
    else
    {
        //m_rayRight->DisableRendering();
    }

    for (int i = 0; i < m_modelRenderers.size(); i++)
    {
        m_modelRenderers[i]->Update(timer);
    }
}

void CalibrationProjectionVisualizationScenario::RenderModels()
{
    // Draw the sample hologram.
    for (int i = 0; i < m_modelRenderers.size(); i++)
    {
        m_modelRenderers[i]->Render();
    }
}

void CalibrationProjectionVisualizationScenario::OnDeviceLost()
{
    for (int i = 0; i < m_modelRenderers.size(); i++)
    {
        m_modelRenderers[i]->ReleaseDeviceDependentResources();
    }
}

void CalibrationProjectionVisualizationScenario::OnDeviceRestored()
{
    for (int i = 0; i < m_modelRenderers.size(); i++)
    {
        m_modelRenderers[i]->CreateDeviceDependentResources();
    }
}
