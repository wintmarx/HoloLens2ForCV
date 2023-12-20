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

#include "BasicHologramMain.h"

namespace BasicHologram
{
    class CalibrationProjectionVisualizationScenario : public Scenario
    {
    public:
        CalibrationProjectionVisualizationScenario(std::shared_ptr<DX::DeviceResources> const& deviceResources);
        virtual ~CalibrationProjectionVisualizationScenario();

        void IntializeSensors();
        void IntializeModelRendering();
        void UpdateModels(DX::StepTimer &timer, const winrt::Windows::Perception::Spatial::SpatialLocation& spatial_location);
        void PositionHologram(winrt::Windows::UI::Input::Spatial::SpatialPointerPose const& pointerPose, const DX::StepTimer& timer);
        void PositionHologramNoSmoothing(winrt::Windows::UI::Input::Spatial::SpatialPointerPose const& pointerPose);
        winrt::Windows::Foundation::Numerics::float3 const& GetPosition()
        {
            return m_modelRenderers[0]->GetPosition();
        }
        void RenderModels();
        void OnDeviceLost();
        void OnDeviceRestored();
        static void CamAccessOnComplete(ResearchModeSensorConsent consent);

        virtual void UpdateState();

        void compensateView(winrt::Windows::Graphics::Holographic::HolographicCameraPose const& cameraPose,
            winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& coordinateSystem);

    protected:

        void IntializeSensorFrameModelRendering();
        void InitializeArucoRendering();

        IResearchModeSensorDevice *m_pSensorDevice;
        IResearchModeSensorDeviceConsent* m_pSensorDeviceConsent;
        std::vector<ResearchModeSensorDescriptor> m_sensorDescriptors;
        IResearchModeSensor *m_pLFCameraSensor = nullptr;
        DirectX::XMFLOAT4X4 m_LFCameraPose;
        DirectX::XMFLOAT4X4 m_LFCameraRotation;
        DirectX::XMFLOAT4 m_LFRotDeterminant;
        DirectX::XMMATRIX m_lf_inv_extr;

        IResearchModeSensor *m_pRFCameraSensor = nullptr;
        DirectX::XMFLOAT4X4 m_RFCameraPose;
        DirectX::XMFLOAT4X4 m_RFCameraRotation;
        DirectX::XMFLOAT4 m_RFRotDeterminant;

        std::vector<std::shared_ptr<ModelRenderer>> m_modelRenderers;
        std::shared_ptr<VectorModel> m_aruco_axis_model;
        bool m_aruco_found = false;

        std::shared_ptr<ArucoDetector> m_arucoDetectorLeft;
        std::shared_ptr<ArucoDetector> m_arucoDetectorRight;

        std::shared_ptr<SensorReaderThread> m_sensorReaderLeft;
        std::shared_ptr<SensorReaderThread> m_sensorReaderRight;
        
        int m_state = 0;
    };
}
