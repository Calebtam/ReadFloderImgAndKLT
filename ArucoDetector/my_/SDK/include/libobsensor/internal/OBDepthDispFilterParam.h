#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @file OBDepthDispFilterParam.h
 * @brief 深度滤波参数，仅内部使用，用于工程软件生产标定
 */

#ifdef OB_SENSOR_SDK_DEVELOPER

/**
 * @brief 后处理类型
 */
typedef enum OB_DDO_TYPE : uint32_t {
    NOISE_REMOVAL           = (1 << 0),  // 去噪
    EDGE_NOISE_REMOVAL      = (1 << 1),  // 边缘去噪
    SPATIAL_FILTER_FAST     = (1 << 2),  // 快速空域滤波
    SPATIAL_FILTER_MODERATE = (1 << 3),  // 适中空域滤波
    SPATIAL_FILTER_ADVANCED = (1 << 4),  // 增强空域滤波
    TEMPORAL_FILTER         = (1 << 5),  // 时域滤波
    HOLE_FILLING            = (1 << 6),  // 填洞滤波
} DDOType, ob_ddo_type;

/**
 * @brief 去噪方式
 */
typedef enum OB_DDO_NOISE_REMOVAL_TYPE {
    NR_LUT     = 0,  // SPLIT
    NR_OVERALL = 1,  // NON_SPLIT
} DDONoiseRemovalType,
    ob_ddo_noise_removal_type;

/**
 * @brief 边缘去噪方式
 */
typedef enum OB_DDO_EDGE_NOISE_REMOVAL_TYPE {
    MG_FILTER = 0,
    MGH_FILTER,  // horizontal MG
    MGA_FILTER,  // asym MG
    MGC_FILTER,
    // RM_FILTER,
} DDOEdgeNoiseRemovalType,
    ob_ddo_edge_noise_removal_type;

typedef enum OB_DDO_SPATIAL_FILTER_ADVANCED_TYPE {
    SFA_NONE       = 0x00,  // 0b00,
    SFA_HORIZONTAL = 0x01,  // 0b01,
    SFA_VERTICAL   = 0x02,  // 0b10,
    SFA_ALL        = 0xff,  // 0b11,
} DDOSpatialAdvancedType,
    ob_ddo_spatial_advanced_type;

/**
 * @brief 填洞方式
 */
typedef enum OB_DDO_HOLE_FILLING_TYPE {
    FILL_TOP     = 0,
    FILL_NEAREST = 1,  // "max" means farest for depth, and nearest for disparity; FILL_NEAREST
    FILL_FAREST  = 2,  // FILL_FAREST
} DDOHoleFillingType,
    ob_ddo_hole_filling_type;

typedef struct OBNoiseRemovalFilterParams_t {
    uint16_t            size       = 0;
    uint16_t            disp_diff  = 0;
    DDONoiseRemovalType type       = NR_OVERALL;
    uint16_t            lut[4 * 4] = { 0 };  // max_size_lut
} NoiseRemovalFilterParams, ob_noise_removal_filter_params;

typedef struct OBEdgeNoiseRemovalFilterParams_t {
    DDOEdgeNoiseRemovalType type    = MG_FILTER;
    uint16_t                x1_th   = 0;
    uint16_t                x2_th   = 0;
    uint16_t                y1_th   = 0;
    uint16_t                y2_th   = 0;
    uint16_t                limit_x = 0;
    uint16_t                limit_y = 0;
    uint16_t                R       = 0;  // MGC
    uint16_t                width1  = 0;
    uint16_t                width2  = 0;
    /// RM_filter, intrinsics needed
} EdgeNoiseRemovalFilterParams, ob_edge_noise_removal_filter_params;

typedef struct OBSpatialFastFilterParams_t {
    uint8_t size = 0;
} SpatialFastFilterParams, ob_spatial_faster_filter_params;

typedef struct SpatialModerateFilterParams_t {
    uint8_t  size      = 3;
    uint8_t  iters     = 0;
    uint16_t disp_diff = 0;
} SpatialModerateFilterParams, ob_spatial_moderate_filter_params;

typedef struct SpatialAdvancedFilterParams_t {
    DDOSpatialAdvancedType type      = SFA_NONE;
    uint8_t                iters     = 0;
    float                  alpha     = 0.f;
    uint16_t               disp_diff = 0;
    uint16_t               radius    = 0;
} SpatialAdvancedFilterParams, ob_spatial_advanced_filter_params;

typedef struct OBHoleFillingFilterParams_t {
    DDOHoleFillingType type = FILL_TOP;
} HoleFillingFilterParams, ob_hole_filing_filter_params;

typedef struct OBTemporalFilterParams_t {
    bool  fill   = true;
    float scale  = 0.f;
    float weight = 0.f;
} TemporalFilterParams, ob_temporal_filter_params;

typedef struct DDOConfig_t {
    size_t                       width;
    size_t                       height;
    int32_t                      depth_unit_x100mm;
    uint32_t                     enable_bitmap;
    bool                         depth_flag;
    float                        bxf;
    uint16_t                     disp_bit_size;
    uint16_t                     invalid_value;
    NoiseRemovalFilterParams     noiseRemovalFilterParams;
    EdgeNoiseRemovalFilterParams edgeNoiseRemovalFilterParams;
    SpatialFastFilterParams      spatialFastFilterParams;
    SpatialModerateFilterParams  spatialModerateFilterParams;
    SpatialAdvancedFilterParams  spatialAdvancedFilterParams;
    HoleFillingFilterParams      holeFillingFilterParams;
    TemporalFilterParams         temporalFilterParams;
} DDOConfig, ob_ddo_config;

#endif