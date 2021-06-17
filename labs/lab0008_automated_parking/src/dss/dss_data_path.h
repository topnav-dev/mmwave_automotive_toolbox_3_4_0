/**
 *   @file  dss_data_path.h
 *
 *   @brief
 *      This is the data path processing header.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef DSS_DATA_PATH_H
#define DSS_DATA_PATH_H

#include <ti/sysbios/knl/Semaphore.h>


#include <ti/common/sys_common.h>
#include <ti/common/mmwave_error.h>
#include <ti/drivers/adcbuf/ADCBuf.h>
#include <ti/drivers/edma/edma.h>
#include "../common/mmw_config.h"
#include "../common/detected_obj.h"
#include "../common/pa_config_consts.h"
#include <radarProcess.h>
#include <modules/utilities/radarOsal_malloc.h>



#ifdef __cplusplus
extern "C" {
#endif

//#define OGMAP_INTERFACE 1

#define MMW_SRR_DEMO_EDMASCATCHBUF_SIZE (0x4000)

#define BYTES_PER_SAMP_1D (2*sizeof(int16_t))  /*16 bit real, 16 bit imaginary => 4 bytes */
#define BYTES_PER_SAMP_2D (2*sizeof(int32_t))  /*32 bit real, 32 bit imaginary => 8 bytes */
#define BYTES_PER_SAMP_DET sizeof(uint16_t) /*pre-detection matrix is 16 bit unsigned =>2 bytes*/

//DETECTION (CFAR-CA) related parameters
#define DET_THRESH_MULT 25
#define DET_THRESH_SHIFT 5 //DET_THRESH_MULT and DET_THRESH_SHIFT together define the CFAR-CA threshold
#define DET_GUARD_LEN 4 // this is the one sided guard lenght
#define DET_NOISE_LEN 16 //this is the one sided noise length

#define PI_ 3.1415926535897
#define ONE_Q15 (1 << 15)
#define ONE_Q19 (1 << 19)
#define ONE_Q8 (1 << 8)


extern radarOsal_heapObj gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS];


/*!< Peak grouping scheme of CFAR detected objects based on peaks of neighboring cells taken from detection matrix */
#define MMW_PEAK_GROUPING_DET_MATRIX_BASED 1

/*!< Peak grouping scheme of CFAR detected objects based only on peaks of neighboring cells that are already detected by CFAR */
#define MMW_PEAK_GROUPING_CFAR_PEAK_BASED  2

/*!< cumulative average of left+right */
#define MMW_NOISE_AVG_MODE_CFAR_CA       ((uint8_t)0U)

/*!< cumulative average of the side (left or right) that is greater */
#define MMW_NOISE_AVG_MODE_CFAR_CAGO     ((uint8_t)1U)

/*!< cumulative average of the side (left or right) that is smaller */
#define MMW_NOISE_AVG_MODE_CFAR_CASO     ((uint8_t)2U)

/*! @brief Message ID for the custom messages from the MRR demo. @{*/
#define  MMWDEMO_OUTPUT_MSG_CLUSTERS        2
#define  MMWDEMO_OUTPUT_MSG_TRACKED_OBJECTS 3
#define  MMWDEMO_OUTPUT_MSG_PARKING_ASSIST  4
/*! @}*/

/*! @brief DSP cycle profiling structure to accumulate different
    processing times in chirp and frame processing periods */
typedef struct cycleLog_t_ {
    uint32_t interChirpProcessingTime;  /*!< @brief total processing time during all chirps in a frame excluding EDMA waiting time*/
    uint32_t interChirpWaitTime;        /*!< @brief total wait time for EDMA data transfer during all chirps in a frame*/
    uint32_t interFrameProcessingTime;  /*!< @brief total processing time for 2D and 3D excluding EDMA waiting time*/
    uint32_t interFrameWaitTime;        /*!< @brief total wait time for 2D and 3D EDMA data transfer */
    uint32_t dopplerProcCycles;         /*! @brief cycle cost for Doppler processing. */
    uint32_t cfarProcCycles;            /*! @brief cycle cost for detection CFAR. */
    uint32_t doaProcCycles;             /*! @brief cycle cost for DoA. */
    uint32_t dbscanProcCycles;          /*! @brief cycle cost for DBSCAN. */
} cycleLog_t;

/*!
 *  @brief    pre-computed parameters of the max-velocity-enhancement.
 */
typedef struct maxVelEnhStruct_t_
{
    float velResolutionFastChirp;
    float invVelResolutionSlowChirp;
    uint16_t maxVelAssocThresh;
    uint16_t padding;
}maxVelEnhStruct_t;


/*!
 *  @brief    Parameters of CFAR detected object during the first round of
 *  CFAR detections
 *
 */
typedef struct MmwDemo_objRaw1D
{
    uint16_t   rangeIdx;     /*!< @brief Range index */
    uint16_t   dopplerIdx;   /*!< @brief Doppler index */
    int16_t   velDisambFacValidity; /*!< @brief velocity disambiguation factor */
    uint16_t   dopplerSNRdB;      /*!< @brief Peak value */
} MmwDemo_objRaw1D_t;



/*!
 *  @brief    Parameters of CFAR detected object during the second round of
 *  CFAR detections.
 *
 */
typedef struct MmwDemo_objRaw2D
{
    uint16_t   rangeIdx;     /*!< @brief Range index */
    uint16_t   dopplerIdx;   /*!< @brief Doppler index */
    uint16_t   range;        /*!< @brief Range (in meters * (1 << xyzOutputQFormat)) */
    int16_t   speed;        /*!< @brief relative velocity (in meters/sec * (1 << xyzOutputQFormat)) */
    uint16_t   peakVal;      /*!< @brief Peak value */
    uint16_t   rangeSNRdB;     /*!< @brief SNR of the peak in the range dimension */
    uint16_t   dopplerSNRdB;   /*!< @brief SNR of the peak in the doppler dimension */
} MmwDemo_objRaw2D_t;

/*!
 *  @brief    Detected object estimated parameters.
 *
 */
typedef struct MmwDemo_detectedObjActual_t
{
    uint16_t   rangeIdx;            /*!< @brief Range index */
    uint16_t   dopplerIdx;          /*!< @brief Doppler index */

    uint16_t   range;               /*!< @brief Range (meters in oneQformat) */
    int16_t    speed;               /*!< @brief Doppler (m/s in oneQformat) */
    int16_t    sinAzim;             /*!< @brief wx  sin(Azim).  Q format provides the bitwidth. */

    uint16_t   peakVal;             /*!< @brief Peak value */

    uint16_t   rangeSNRdB;          /*!< @brief Range SNR (dB)  */
    uint16_t   dopplerSNRdB;        /*!< @brief Doppler SNR (dB) */
    uint16_t   sinAzimSNRLin;       /*!< @brief omega SNR (linear scale) */

    int16_t  x;             /*!< @brief x - coordinate in meters. Q format provides the bitwidth. */
    int16_t  y;             /*!< @brief y - coordinate in meters. Q format provides the bitwidth. */
    int16_t  z;             /*!< @brief z - coordinate in meters. Q format provides the bitwidth. */

} MmwDemo_detectedObjActual;


/*!
 *  @brief   Structure for each cluster information report .
 *
 */
typedef struct clusteringDBscanReportForTx_t
{
    int16_t     xCenter;               /**< the clustering center on x direction */
    int16_t     yCenter;               /**< the clustering center on y direction */
    int16_t     xSize;                 /**< the clustering size on x direction */
    int16_t     ySize;                 /**< the clustering size on y direction */
} clusteringDBscanReportForTx;

/*!
 *  @brief   Structure for tracking report.
 *
 */
typedef struct trackingReportForTx_t
{
    int16_t     x;                  /**< the tracking output -> x co-ordinate */
    int16_t     y;                  /**< the tracking output -> y co-ordinate */
    int16_t     xd;                 /**< velocity in the x direction */
    int16_t     yd;                 /**< velocity in the y direction */
    int16_t     xSize;              /**< cluster size (x direction). */
    int16_t     ySize;              /**< cluster size (y direction). */
} trackingReportForTx;

/*!
 *  @brief    Detected object estimated parameters to be transmitted out.
 *
 */
typedef struct MmwDemo_detectedObjForTx_t
{
    int16_t  speedIdx;      /*!< @brief Doppler index */
    int16_t  x;             /*!< @brief x - coordinate in meters. Q format provides the bitwidth. */
    int16_t  y;             /*!< @brief y - coordinate in meters. Q format provides the bitwidth. */
    int16_t  z;             /*!< @brief z - coordinate in meters. Q format provides the bitwidth. */
    int16_t  rangeIdx;      /*!< @brief range index */
    int16_t  peakVal;       /*!< @brief peakVal index */
} MmwDemo_detectedObjForTx;

typedef struct
{
    /** Distance of the object in centimetres. */
    float       range;

    /** Angle of the object in degrees. */
    float       azimuthAngle;

    /** Angle of the object in degrees. */
    float       elevAngle;

    /** Speed of the object in m/s */
    float       velocity;

    /** Azimuth angle variance from DOA stage. */
    float       azimAngleVarEst;

    /** SNR. */
    float       snr;

} RadarDsp_DetObjInfo;

/*!
 *  @brief   Structure for each cluster information report .
 *
 */
typedef struct
{
    int16_t     xCenter;               /**< the clustering center on x direction */
    int16_t     yCenter;               /**< the clustering center on y direction */
    int16_t     zCenter;               /**< the clustering center on z direction */
    int16_t     xSize;                 /**< the clustering size on x direction */
    int16_t     ySize;                 /**< the clustering size on y direction */
    int16_t     zSize;                 /**< the clustering size on z direction */
} radar_dbscanReportForGui;


typedef struct
{
    /** Number of objects (points). */
    int32_t                 numObj;

    RadarDsp_DetObjInfo     objInfo[MAX_RESOLVED_OBJECTS_PER_FRAME];

} radarProcessOutput_t;

/*!
 *  @brief    These parameters allow the SNR requirements to be varied as a
 *  function of range.
 */
typedef struct SNRThresholds
{
    uint16_t   rangelim;        /*!< @brief Range (in meters * (1 << xyzOutputQFormat)) upto which
                                 * the SNR requirement is valid. */
    uint16_t   threshold;        /*!< @brief SNR threshold (dB) for the range. */
} RangeDependantThresh_t;


/*!
 *  @brief    Active Doppler lines, lines (bins) on which the
 *            CFAR detector detected objects during the detections in
 *            Doppler direction
 *
 */
typedef struct MmwDemo_1D_DopplerLines
{
    uint32_t   *dopplerLineMask;      /*!< @brief Doppler line bit mask of active
                                   (CFAR detected) Doppler bins in the first
                                   round of CFAR detections in Doppler direction.
                                   The LSB bit of the first word corresponds to
                                   Doppler bin zero of the range/Doppler
                                   detection matrix*/
    uint16_t   currentIndex;     /*!< @brief starting index for the search
                                   for next active Doppler line */
    uint16_t    dopplerLineMaskLen;   /*!< @brief size of dopplerLineMask array, (number of
                                    32_bit words, for example for Doppler FFT
                                    size of 64 this length is equal to 2)*/
} MmwDemo_1D_DopplerLines_t;

/*!
 *  @brief Timing information
 */
typedef struct MmwDemo_timingInfo
{
    /*! @brief number of processor cycles between frames excluding
           processing time to transmit output on UART */
    uint32_t interFrameProcCycles;

     /*! @brief time to transmit out detection information (in DSP cycles)
           */
    uint32_t transmitOutputCycles;

    /*! @brief Chirp processing end time */
    uint32_t chirpProcessingEndTime;

    /*! @brief Chirp processing end margin in number of cycles before due time
     * to start processing next chirp, minimum value*/
    uint32_t chirpProcessingEndMarginMin;

    /*! @brief Chirp processing end margin in number of cycles before due time
     * to start processing next chirp, maximum value*/
    uint32_t chirpProcessingEndMarginMax;

    /*! @brief Inter frame processing end time */
    uint32_t interFrameProcessingEndTime;

    /*! @brief Inter frame processing end margin in number of cycles before
     * due time to start processing first chirp of the next frame */
    uint32_t interFrameProcessingEndMargin;

    /*! @brief CPU Load during active frame period - i.e. chirping */
    uint32_t activeFrameCPULoad;

    /*! @brief CPU Load during inter frame period - i.e. after chirps
     *  are done and before next frame starts */
    uint32_t interFrameCPULoad;

} MmwDemo_timingInfo_t;


/**
 * @brief
 *  Millimeter Wave Demo Data Path Information.
 *
 * @details
 *  The structure is used to hold all the relevant information for
 *  the data path.
 */
typedef struct MmwDemo_DSS_DataPathObj_t
{
    /*! @brief   configuration struction for signal processing chain */
    radarProcessConfig_t   radarProcConfig;

    /*! @brief   handle to signal processing chain */
    void * radarProcessHandle;

    /*! @brief   Signal processing output: point cloud and heatmap pointers*/
    RadarDsp_outputBuffCntxt * outBuffCntxt;

    /*! @brief   Configuration which is used to execute the demo */
    MmwDemo_Cfg       cfg;

    /*! @brief   ADCBUF handle. */
    ADCBuf_Handle adcbufHandle;

    /*! @brief   Handle of the EDMA driver. */
    EDMA_Handle edmaHandle[2];

    /*! @brief   EDMA error Information when there are errors like missing events */
    EDMA_errorInfo_t  EDMA_errorInfo;

    /*! @brief EDMA transfer controller error information. */
    EDMA_transferControllerErrorInfo_t EDMA_transferControllerErrorInfo;

    /*! @brief pointer to ADC buffer */
    cmplx16ImRe_t *ADCdataBuf;

    /*! @brief twiddle table for 1D FFT */
    cmplx16ImRe_t *twiddle16x16_1D;

    /*! @brief window coefficients for 1D FFT */
    int16_t *window1D;

    /*! @brief ADCBUF input samples in L2 scratch memory */
    cmplx16ImRe_t *adcDataIn;

    /*! @brief 1D FFT output */
    cmplx16ImRe_t *fftOut1D;

    /*! @brief Doppler proc input pointer, ping and pong */
    int32_t  *dopplerProcin[2];

    /*! @brief Doppler proc output pointer, ping and pong */
    float  *dopplerProcOut[2];

    /*! @brief Flag to indicate using Ping-pong buffer scheme for Doppler proc*/
    int8_t dopplerUsePingPong;

    /*! @brief ADCBUF input samples in L2 scratch memory */
    int32_t  *scratchBuf;

    /*! @brief twiddle table for 2D FFT */
    //cmplx32ReIm_t *twiddle32x32_2D;

    /*! @brief window coefficients for 2D FFT */
    //int32_t *window2D;

    /*! @brief ping pong buffer for 2D from radar Cube */
    cmplx16ImRe_t *dstPingPong;

    /*! @brief CFAR output objects index buffer */
    uint16_t *cfarDetObjIndexBuf;

    /*! @brief CFAR output objects' SNR buffer */
    uint16_t *cfarDetObjSNR;

    /*! @brief input for Azimuth FFT */
    cmplx32ReIm_t *azimuthIn;

    /*! @brief input for Azimuth FFT */
    cmplx32ReIm_t *antInp;

    /*! @brief input for Elevation FFT */
    cmplx32ReIm_t *elevationIn;

    /*! @brief output of Azimuth FFT */
    cmplx32ReIm_t *azimuthOut;

    /*! @brief output of Elevation FFT */
    cmplx32ReIm_t *elevationOut;

    /*! @brief output of Azimuth FFT magnitude squared */
    float   *azimuthMagSqr;

    /*! @brief twiddle factors table for Azimuth FFT */
    cmplx32ReIm_t *azimuthTwiddle32x32;

    /*! @brief Pointer to single point DFT coefficients used for Azimuth processing */
    cmplx16ImRe_t *azimuthModCoefs;

    /*! @brief Pointer to DC range signature compensation buffer */
    cmplx32ImRe_t *dcRangeSigMean;

    /*! @brief Pointer to Radar Cube memory in L3 RAM */
    cmplx16ImRe_t *radarCube;

    /*! @brief Pointer to range/Doppler log2 magnitude detection matrix in L3 RAM */
    uint16_t *detMatrix;

    /*! @brief   Processing path  - either point-cloud or max-vel enhancement.*/
    uint8_t processingPath;

    /*! @brief   Chirp Threshold configuration used for ADCBUF driver */
    uint8_t chirpThreshold;

    /*! @brief DC range signature calibration counter */
    uint8_t dcRangeSigCalibCntr;

    /*! @brief log2 of number of averaged chirps */
    uint8_t log2NumAvgChirps;

    /*! @brief log 2 of number of doppler bins */
    uint8_t log2NumDopplerBins;

    /*! @brief Q format of the output x/y/z coordinates */
    uint8_t xyzOutputQFormat;

    /*! @brief index of the subframe to which this object belongs */
    uint8_t subframeIndx;

    /*! @brief log2 of the number of virtual antennas. */
    uint8_t log2numVirtAnt;

    /*! @brief Q format of the sin of the azimuth. */
    uint8_t sinAzimQFormat;

    /*! @brief Number of bins for the parkingAssist module. */
    //uint8_t parkingAssistNumBins;

    /*! @brief log2 of the number of bins for the parkingAssist module (used for scaling operations). */
    //uint8_t parkingAssistNumBinsLog2;

    /*! @brief padding. . */
    uint8_t padding;

    /*! @brief The HPF can mess up the noise floor computation. So for a certain number of indices, we
     *  only use the non-hpf sides. */
    uint16_t cfarCfgRange_minIndxToIgnoreHPF;

    /*! @brief maximum range to look for obstacles. . */
    uint16_t parkingAssistMaxRange;

    /*! @brief minimum range to look for obstacles. . */
    uint16_t parkingAssistMinRange;

    /*! @brief number of chirps per frame */
    uint16_t numChirpsPerFrame;

    /*! @brief Number of detected objects */
    uint16_t numDetObjRaw;

    /*! @brief minimum range at which a target is detected ( in xyzOutputQFormat precision). */
    uint16_t minRange;

    /*! @brief maximum range at which a target is detected ( in xyzOutputQFormat precision). */
    uint16_t maxRange;

    /*! @brief number of active trackers.  */
    uint16_t numActiveTrackers;

    /*! @brief number of objects to be detected in 2D-CFAR.  */
    uint16_t maxNumObj2DRaw;

    /*! @brief   Number of receive channels */
    uint16_t numRxAntennas;

    /*! @brief number of transmit antennas */
    uint16_t numTxAntennas;

    /*! @brief number of virtual antennas */
    uint16_t numVirtualAntennas;

    /*! @brief number of virtual azimuth antennas */
    uint16_t numVirtualAntAzim;

    /*! @brief number of virtual elevation antennas */
    uint16_t numVirtualAntElev;

    /*! @brief number of angle bins */
    uint16_t numAngleBins;

    /*! @brief  number of chirps per chirp type*/
    uint16_t numChirpsPerChirpType;

    /*! @brief number of doppler bins */
    uint16_t numDopplerBins;

    /*! @brief chirp counter modulo number of chirps per frame */
    uint16_t chirpCount;

    /*! @brief chirp counter modulo number of tx antennas */
    uint16_t txAntennaCount;

    /*! @brief chirp counter modulo number of Doppler bins */
    uint16_t dopplerBinCount;

    /*! @brief chirp counter modulo number of subframe*/
    uint16_t chirpTypeCount;

    /*! @brief number of ADC samples */
    uint16_t numAdcSamples;

    /*! @brief number of range bins */
    uint16_t numRangeBins;

    /*! @brief Half bin needed for doppler correction as part of Azimuth processing */
    cmplx16ImRe_t azimuthModCoefsHalfBin;

    /*! @brief range resolution in meters */
    float rangeResolution;

    /*! @brief velocity resolution in meters/sec */
    float velResolution;

    /*! @brief maximum unambiguous velocity (without algorithmic improvements) in meters/sec */
    float maxUnambiguousVel;

    /*! @brief inverse of the oneQformat */
    float invOneQFormat;

    /*! @brief inverse of the oneQformat */
    float invOneSinAzimFormat;

    /*! @brief inverse of the numAngleBins */
    float invNumAngleBins;

    /*! @brief SNR thresholds as a function of range. */
    RangeDependantThresh_t SNRThresholds[MAX_NUM_RANGE_DEPENDANT_SNR_THRESHOLDS] ;

    /*! @brief SNR thresholds as a function of range. */
    RangeDependantThresh_t peakValThresholds[MAX_NUM_RANGE_DEPENDANT_SNR_THRESHOLDS] ;

    /*! @brief Detected Doppler lines */
    MmwDemo_1D_DopplerLines_t detDopplerLines;

    /*! @brief Detected objects after second pass in Range direction.
     *   These objects are send out as point clouds. */
    //MmwDemo_detectedObjActual *detObj2D;

    /*! @brief Detected objects after first pass in Doppler direction. */
    //MmwDemo_objRaw1D_t *detObj1DRaw;

    /*! @brief Detected objects before peak grouping */
    //MmwDemo_objRaw2D_t *detObj2DRaw;

    /*! @brief CFAR configuration in Doppler direction */
    //MmwDemo_CfarCfg cfarCfgDoppler;

    /*! @brief CFAR configuration in Range direction */
    //MmwDemo_CfarCfg cfarCfgRange;

    /*! @brief Multi object beam forming configuration */
    //MmwDemo_MultiObjBeamFormingCfg multiObjBeamFormingCfg;

    /*! @brief DC Range antenna signature callibration configuration */
//    MmwDemo_CalibDcRangeSigCfg calibDcRangeSigCfg;

    /*! @brief Timing information */
    MmwDemo_timingInfo_t timingInfo;

    /*! @brief  DSP cycles for chirp and interframe processing and pending
     *          on EDMA data transferes*/
    cycleLog_t cycleLog;

    /*! @brief  Max-velocity constants. */
    maxVelEnhStruct_t maxVelEnhStruct;

#ifndef OGMAP_INTERFACE
    /*! @brief Number of detected objects */
    uint16_t numDetObj;

    /*! @brief Final list of detected object for transmission.*/
    MmwDemo_detectedObjForTx * detObjFinal;
#else
    /*! @brief Final list of detected object for OGMAP. */
    radarProcessOutput_t * outputForOGMAP;
#endif

#ifdef TMDEMOV1
    /*! @brief Number of clustering */
    uint16_t numCluster;
    bool     clusterGenFlag;
    /*! @brief Final report for clustering output.*/
    radar_dbscanReportForGui * dbscanReportFinal;
#endif


    /*! @brief Final list of tracked objects for transmission.*/
    //trackingReportForTx * trackerOpFinal;

    /*! @brief Nearest object as a function of azimuth.*/
    //uint16_t * parkingAssistBins;

    /*! @brief Filtered result of the nearest object as a function of azimuth.*/
    //uint16_t * parkingAssistBinsState;

    /*! @brief The 'age' of the filtered result of the parking state. Added to make the
     *  'smoother' visually. */
    //uint16_t * parkingAssistBinsStateCnt;

} MmwDemo_DSS_DataPathObj;

/**
 *  @b Description
 *  @n
 *   Initializes data path state variables for 1D processing.
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathInit1Dstate(MmwDemo_DSS_DataPathObj *obj);

/**
 *  @b Description
 *  @n
 *   Delete Semaphores which are created in MmwDemo_dataPathInitEdma().
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathDeleteSemaphore(MmwDemo_DSS_DataPathObj *obj);

/**
 *  @b Description
 *  @n
 *   Initializes EDMA driver.
 *  @retval
 *      Not Applicable.
 */
int32_t MmwDemo_dataPathInitEdma(MmwDemo_DSS_DataPathObj *obj);

/**
 *  @b Description
 *  @n
 *   Since there may be multiple subframes, we keep a copy of
 *   the handle for each data path object.
 *  @retval
 *      Not Applicable.
 */
int32_t MmwDemo_dataPathCopyEdmaHandle(MmwDemo_DSS_DataPathObj *objOutput, MmwDemo_DSS_DataPathObj *objInput);

/**
 *  @b Description
 *  @n
 *   Configures EDMA driver for all of the data path processing.
 *  @retval
 *      Not Applicable.
 */
int32_t MmwDemo_dataPathConfigEdma(MmwDemo_DSS_DataPathObj *obj);

/**
 *  @b Description
 *  @n
 *   Creates heap in L2 and L3 and allocates data path buffers,
 *   The heap is destroyed at the end of the function.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathConfigBuffers(MmwDemo_DSS_DataPathObj *obj, uint32_t adcBufAddress);

/**
 *  @b Description
 *  @n
 *   Configures azimuth heat map related processing.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathConfigAzimuthHeatMap(MmwDemo_DSS_DataPathObj *obj);

/**
 *  @b Description
 *  @n
 *   Configures FFTs (twiddle tables etc) involved in 1D, 2D and Azimuth processing.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathConfigFFTs(MmwDemo_DSS_DataPathObj *obj);

/**
 *  @b Description
 *  @n
 *  Wait for transfer of data corresponding to the last 2 chirps (ping/pong)
 *  to the radarCube matrix before starting interframe processing.
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_waitEndOfChirps(MmwDemo_DSS_DataPathObj *obj, uint8_t subframeIdx);

/**
 *  @b Description
 *  @n
 *    Chirp processing. It is called from MmwDemo_dssDataPathProcessEvents. It
 *    is executed per chirp
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_processChirp(MmwDemo_DSS_DataPathObj *obj, uint8_t subframeIdx);

/**
 *  @b Description
 *  @n
 *    Interframe processing. It is called from MmwDemo_dssDataPathProcessEvents
 *    after all chirps of the frame have been received and 1D FFT processing on them
 *    has been completed.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_interFrameProcessing(MmwDemo_DSS_DataPathObj *obj, uint8_t subframeIdx);

/**
 *  @b Description
 *  @n
 *      Power of 2 round up function.
 */
uint32_t MmwDemo_pow2roundup (uint32_t x);

/**
 *  @b Description
 *  @n
 *    The function computes the CRLB of the given estimate given an  SNR input (dB)
 *    and the number of samples used in the estimate, and the resolution of the estimate.
 *
 *  @param[in]    SNRdB              16 bit input with specified bitwidth.
 *  @param[in]    bitW               input fractional bitwidth.
 *  @param[in]    n_samples          number of samples per chirp.
 *  @param[in]    rangeResolution    range resolution in meters.
 *
 *  @retval
 *      2^(input/(2^fracBitIn))
 */
float convertSNRdBToVar(uint16_t SNRdB,uint16_t bitW, uint16_t n_samples, float resolution);

/**
 *  @b Description
 *  @n
 *    The function computes the CRLB of the given estimate given an  SNR input (linear)
 *    and the number of samples used in the estimate, and the resolution of the estimate.
 *
 *    The CRLB is lower bounded by the resolution.
 *
 *  @param[in]    SNRLin              16 bit input with specified bitwidth.
 *  @param[in]    bitW               input fractional bitwidth.
 *  @param[in]    n_samples          number of samples per chirp.
 *  @param[in]    resolution         resolution in meters.
 *
 *  @retval
 *      2^(input/(2^fracBitIn))
 */
float convertSNRLinToVar(uint16_t SNRLin,uint16_t bitW, uint16_t n_samples, float resolution);

/**
 *  @b Description
 *  @n
 *    The function populates the object location arrays
 *   for transmission to MSS. The reason we do this
 *   additional step is to minimize the size of the
 *   the transmission by populating new structure which
 *   hold only the minimum information necessary for the
 *   GUI.
 *
 *  @param[in]    input              data path object.
 *
 *  @retval
 *      none
 */
void populateOutputs(MmwDemo_DSS_DataPathObj *obj);

/**
 *  @b Description
 *  @n
 *    The function performs a quadractic peak interpolation to compute the
 *  fractional offset of the 'true' peak location. It is primarily intended to be
 *  used in oversampled FFTs.
 *
 *  @param[in]    y                     Input array.
 *  @param[in]    len                   length of the input array.
 *  @param[in]    indx                  indx of the peak location.
 *
 *
 *  @retval
 *      interpolated offset.
 */
float quadraticInterpFltPeakLoc(float * restrict y, int32_t len, int32_t indx);


/*!*****************************************************************************************************************
 * \brief
 * Function Name       :    MmwDemo_DopplerCompensation
 *
 * \par
 * <b>Description</b>  : Compensation of Doppler phase shift in the virtual antennas,
 *                       (corresponding to second Tx antenna chirps). Symbols
 *                       corresponding to virtual antennas, are rotated by half
 *                       of the Doppler phase shift measured by Doppler FFT.
 *                       The phase shift read from the table using half of the
 *                       object Doppler index  value. If the Doppler index is
 *                       odd, an extra half of the bin phase shift is added.
 *
 * @param[in]               dopplerIdx     : Doppler index of the object
 *
 * @param[in]               numDopplerBins : Number of Doppler bins
 *
 * @param[in]               azimuthModCoefs: Table with cos/sin values SIN in even position, COS in odd position
 *                                           exp(1j*2*pi*k/N) for k=0,...,N-1 where N is number of Doppler bins.
 *
 * @param[out]              azimuthModCoefsHalfBin :  exp(1j*2*pi* 0.5 /N)
 *
 * @param[in,out]           azimuthIn        :Pointer to antenna symbols to be Doppler compensated
 *
 * @param[in]              numAnt       : Number of antenna symbols to be Doppler compensated
 *
 * @param[in]              numTxAnt       : Number of Tx antenna
 *
 * @param[in]              txAntIdx       : Tx Antenna index (Tx1:0 Tx2:1 Tx3:2)
 *
 * @return                  void
 *
 *******************************************************************************************************************
 */
void MmwDemo_addDopplerCompensation(int32_t dopplerIdx,
                                    int32_t numDopplerBins,
                                    uint32_t *azimuthModCoefs,
                                    uint32_t *azimuthModCoefsHalfBin,
                                    int64_t *azimuthIn,
                                    uint32_t numAnt,
                                    uint32_t numTxAnt,
                                    uint16_t txAntIdx);
/**
 *  @b Description
 *  @n
 *      Calculates X/Y coordinates in meters based on the maximum position in
 *      the magnitude square of the azimuth FFT output. The function is called
 *      per detected object.
 *
 *  @param[in] obj                Pointer to data path object
 *
 *  @param[in] objIndex           Detected object index
 *
 *  @param[in] azimIdx            Index of the peak position in Azimuth FFT output
 *
 *  @param[in] azimuthMagSqr      azimuth energy array
 *
 *  @retval
 *      NONE
 */
void MmwDemo_XYcalc (MmwDemo_DSS_DataPathObj *obj,
                     uint32_t objIndex,
                     uint16_t azimIdx,
                     float * azimuthMagSqr);


/**
 *  @b Description
 *  @n
 *      Calculates X/Y/Z coordinates in meters based on the maximum position in
 *      the magnitude square of the azimuth FFT output. The function is called
 *      per detected object.
 *
 *  @param[in] obj                Pointer to data path object
 *
 *  @param[in] objIndex           Detected object index
 *
 *  @param[in] azimIdx            Index of the peak position in Azimuth FFT output
 *
 *  @param[in] azimuthMagSqr      azimuth energy array
 *
 *  @retval
 *      NONE
 */
void MmwDemo_XYZcalc (MmwDemo_DSS_DataPathObj *obj,
                    uint32_t objIndex,
                    uint16_t azimIdx,
                    float * azimuthMagSqr);

/**
 *  @b Description
 *  @n
 *      Initialize the 'parking assist bins' state which is essentially the
 *  closest obstruction upper bounded by an initial value
 *
 *  @param[in] obj                Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void parkingAssistInit(MmwDemo_DSS_DataPathObj *obj);



#ifdef __cplusplus
}
#endif

#endif /* DSS_DATA_PATH_H */

