/*
 *   @file  cli.c
 *
 *   @brief
 *      Mmw (Milli-meter wave) DEMO CLI Implementation
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/uart/UART.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>

/* Demo Include Files */
#include "./common/odsdemo_common.h"
#include "./common/mmw_messages.h"
#include "./mss_mmw.h"


/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/* CLI Extended Command Functions */
#if 0 //ODS DEMO: Remove unused cmds
static int32_t MmwDemo_CLICfarCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIPeakGroupingCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIMultiObjBeamForming (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIExtendedMaxVelocity (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLINearFieldCorrection (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIChirpQualityRxSatMonCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIChirpQualitySigImgMonCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIAnalogMonitorCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLILvdsStreamCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISetDataLogger (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
#endif
static int32_t MmwDemo_CLICalibDcRangeSig (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIClutterRemoval (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[]);

/* ODSDEMO Specific Extended Command Functions */
static int32_t ODSDemo_CLIGuiMonSel(int32_t argc, char* argv[]);

/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/

extern MmwDemo_MCB    gMmwMssMCB;
extern int32_t MmwDemo_mboxWrite(MmwDemo_message     * message);

/**************************************************************************
 *************************** CLI  Function Definitions **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor start command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[])
{
    bool doReconfig = true;
    if (argc==2)
    {
        doReconfig = (bool) atoi (argv[1]);
    }
    /* Post sensorSTart event to notify configuration is done */
    MmwDemo_notifySensorStart(doReconfig);
    /* Pend for completion */
    return (MmwDemo_waitSensorStartComplete());
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor stop command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[])
{
    /* Post sensorSTOP event to notify sensor stop command */
    MmwDemo_notifySensorStop();
    /* Pend for completion */
    MmwDemo_waitSensorStopComplete();
    return 0;
}

static int32_t MmwDemo_CLIGetSubframe (int32_t argc, char* argv[], int32_t expectedArgc, int8_t* subFrameNum)
{
    int8_t subframe;

    /* Sanity Check: Minimum argument check */
    if (argc != expectedArgc)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /*Subframe info is always in position 1*/
    subframe = (int8_t) atoi(argv[1]);

    if(subframe >= (int8_t)RL_MAX_SUBFRAMES)
    {
        CLI_write ("Error: Subframe number is invalid\n");
        return -1;
    }

    *subFrameNum = (int8_t)subframe;

    return 0;
}

static void MmwDemo_mssCfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum)
{
    /* if subFrameNum undefined, broadcast to all sub-frames */
    if(subFrameNum == MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
    {
        uint8_t  indx;
        for(indx = 0; indx < RL_MAX_SUBFRAMES; indx++)
        {
            memcpy((void *)((uint32_t) &gMmwMssMCB.cliCfg[indx] + offset), srcPtr, size);
        }

    }
    else
    {
        /* Apply configuration to specific subframe (or to position zero for the legacy case
           where there is no advanced frame config) */
        memcpy((void *)((uint32_t) &gMmwMssMCB.cliCfg[subFrameNum] + offset), srcPtr, size);
    }
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for CFAR configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLICfarCfg (int32_t argc, char* argv[])
{
    MmwDemo_CfarCfg     cfarCfg;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 15)
    {
        CLI_write ("Error: Invalid cfarCfg arg count\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfarCfg, 0, sizeof(MmwDemo_CfarCfg));

    /* Populate configuration: */
    cfarCfg.detectMethod           = (uint16_t) atoi (argv[1]);
    cfarCfg.leftSkipBin            = (uint16_t) atoi (argv[2]);
    cfarCfg.closeInRangeBin        = (uint16_t) atoi (argv[3]);
    cfarCfg.searchWinSizeRange     = (uint16_t) atoi (argv[4]);
    cfarCfg.guardSizeRange         = (uint16_t) atoi (argv[5]);
    cfarCfg.searchWinSizeSpreading = (uint16_t) atoi (argv[6]);
    cfarCfg.guardSizeSpreading     = (uint16_t) atoi (argv[7]);

    cfarCfg.rangeThresh            = (float)    atof (argv[8]) * 0.1f;
    cfarCfg.fftSpreadingThresh     = (float)    atof (argv[9]) * 1.0e-5;
    cfarCfg.noiseCalcType          = (uint16_t) atoi (argv[10]);
    cfarCfg.localPeakEnable        = (uint16_t) atoi (argv[11]);
    cfarCfg.peakAngleDiffThresh    = (uint16_t) atoi (argv[12]);
    cfarCfg.maxRangeForDetection   = (float)    atof (argv[13]) * 0.1f;
    cfarCfg.secondPeakThresh       = (float)    atof (argv[14]);
    uint16_t expensionSize1        = cfarCfg.guardSizeRange + cfarCfg.searchWinSizeRange;
    uint16_t expensionSize2        = cfarCfg.guardSizeSpreading + cfarCfg.searchWinSizeSpreading;
    cfarCfg.expensionSize          = (expensionSize1 > expensionSize2)? expensionSize1: expensionSize2;

    /* Sanity Check flag arguments */
    if ((cfarCfg.detectMethod < 1) || (cfarCfg.detectMethod > 2))
    {
        CLI_write ("Error: Invalid cfarCfg.detectMethod\n");
        return -1;
    }
    if (cfarCfg.noiseCalcType > 2)
    {
        CLI_write ("Error: Invalid cfarCfg.noiseCalcType\n");
        return -1;
    }

    if (cfarCfg.secondPeakThresh < 0.0) //This is the percentage of the primary peak
        cfarCfg.secondPeakThresh = 0.0;
    if (cfarCfg.secondPeakThresh > 99.99)
        cfarCfg.secondPeakThresh = 99.99;

    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&cfarCfg, offsetof(MmwDemo_CliCfg_t, cfarCfg),
        sizeof(MmwDemo_CfarCfg), -1);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_CFAR_CFG;
    message.subFrameNum = -1;
    memcpy((void *)&message.body.cfarCfg, (void *)&cfarCfg, sizeof(MmwDemo_CfarCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for dbscan configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIDbscanCfg (int32_t argc, char* argv[])
{
    MmwDemo_ClusteringCfg     dbscanCfg;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 7)
    {
        CLI_write ("Error: Invalid dbscan arg count\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&dbscanCfg, 0, sizeof(MmwDemo_ClusteringCfg));

    /* Populate configuration: */
    dbscanCfg.nAccFrames        =   (uint16_t) atoi (argv[1]);
    dbscanCfg.epsilon           =   (float)    atof (argv[2]) * 0.1f;
    dbscanCfg.weight            =   (float)    atof (argv[3]) * 0.1f;
    dbscanCfg.vFactor           =   (float)    atof (argv[4]) * 0.1f;
    dbscanCfg.minPointsInCluster    =   (uint16_t) atoi (argv[5]);
    dbscanCfg.fixedPointScale            =   (uint16_t) atoi (argv[6]);

    /* Sanity Check flag arguments */
    if ((dbscanCfg.nAccFrames < 1) || (dbscanCfg.nAccFrames > ODSDEMO_DBSCAN_MAXACCFRAME))
    {
        CLI_write ("Error: Invalid dbscanCfg.nAccFrames\n");
        return -1;
    }

    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&dbscanCfg, offsetof(MmwDemo_CliCfg_t, dbscanCfg),
        sizeof(MmwDemo_ClusteringCfg), -1);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_DBSCAN_CFG;
    message.subFrameNum = -1;
    memcpy((void *)&message.body.dbscanCfg, (void *)&dbscanCfg, sizeof(MmwDemo_ClusteringCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

#if 0 //OD DEMO: Remove unused cmds
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for Peak grouping configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIPeakGroupingCfg (int32_t argc, char* argv[])
{
    MmwDemo_PeakGroupingCfg peakGroupingCfg;
    MmwDemo_message         message;
    int8_t                  subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 7, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&peakGroupingCfg, 0, sizeof(MmwDemo_PeakGroupingCfg));

    /* Populate configuration: */
    peakGroupingCfg.scheme               = (uint8_t) atoi (argv[2]);
    peakGroupingCfg.inRangeDirectionEn   = (uint8_t) atoi (argv[3]);
    peakGroupingCfg.inDopplerDirectionEn = (uint8_t) atoi (argv[4]);
    peakGroupingCfg.minRangeIndex    = (uint16_t) atoi (argv[5]);
    peakGroupingCfg.maxRangeIndex    = (uint16_t) atoi (argv[6]);

    if (peakGroupingCfg.scheme != 1 && peakGroupingCfg.scheme != 2)
    {
        CLI_write ("Error: Invalid peak grouping scheme\n");
        return -1;
    }

    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&peakGroupingCfg, offsetof(MmwDemo_CliCfg_t, peakGroupingCfg),
        sizeof(MmwDemo_PeakGroupingCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_PEAK_GROUPING_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.peakGroupingCfg, (void *)&peakGroupingCfg, sizeof(MmwDemo_PeakGroupingCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for multi object beam forming configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIMultiObjBeamForming (int32_t argc, char* argv[])
{
    MmwDemo_MultiObjBeamFormingCfg cfg;
    MmwDemo_message     message;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 4, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_MultiObjBeamFormingCfg));

    /* Populate configuration: */
    cfg.enabled                     = (uint8_t) atoi (argv[2]);
    cfg.multiPeakThrsScal           = (float) atof (argv[3]);

    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, multiObjBeamFormingCfg),
        sizeof(MmwDemo_MultiObjBeamFormingCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_MULTI_OBJ_BEAM_FORM;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.multiObjBeamFormingCfg, (void *)&cfg, sizeof(MmwDemo_MultiObjBeamFormingCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}
#endif

uint32_t log2Approx(uint32_t x)
{
    uint32_t idx, detectFlag = 0;

    if ( x < 2)
    {
        return (0);
    }

    idx = 32U;
    while((detectFlag==0U) || (idx==0U))
    {
        if(x & 0x80000000U)
        {
            detectFlag = 1;
        }
        x <<= 1U;
        idx--;
    }

    if(x != 0)
    {
        idx = idx + 1;
    }

    return(idx);
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for DC range calibration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLICalibDcRangeSig (int32_t argc, char* argv[])
{
    MmwDemo_CalibDcRangeSigCfg cfg;
    MmwDemo_message            message;
    uint32_t                   log2NumAvgChirps;
    int8_t                     subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_CalibDcRangeSigCfg));

    /* Populate configuration: */
    cfg.enabled          = (uint16_t) atoi (argv[2]);
    cfg.negativeBinIdx   = (int16_t)  atoi (argv[3]);
    cfg.positiveBinIdx   = (int16_t)  atoi (argv[4]);
    cfg.numAvgChirps     = (uint16_t)  atoi (argv[5]);

    if (cfg.negativeBinIdx > 0)
    {
        CLI_write ("Error: Invalid negative bin index\n");
        return -1;
    }
    if ((cfg.positiveBinIdx - cfg.negativeBinIdx + 1) > DC_RANGE_SIGNATURE_COMP_MAX_BIN_SIZE)
    {
        CLI_write ("Error: Number of bins exceeds the limit\n");
        return -1;
    }
    log2NumAvgChirps = (uint32_t) log2Approx (cfg.numAvgChirps);
    if (cfg.numAvgChirps != (1 << log2NumAvgChirps))
    {
        CLI_write ("Error: Number of averaged chirps is not power of two\n");
        return -1;
    }

    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, calibDcRangeSigCfg),
        sizeof(MmwDemo_CalibDcRangeSigCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_CALIB_DC_RANGE_SIG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.calibDcRangeSigCfg, (void *)&cfg, sizeof(MmwDemo_CalibDcRangeSigCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

#if 0 //OD DEMO: Remove unused cmds
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for Velocity Disambiguation Configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIExtendedMaxVelocity (int32_t argc, char* argv[])
{
    MmwDemo_ExtendedMaxVelocityCfg cfg;
    MmwDemo_message                message;
    int8_t                         subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 3, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_ExtendedMaxVelocityCfg));

    /* Populate configuration: */
    cfg.enabled          = (uint16_t) atoi (argv[2]);


    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, extendedMaxVelocityCfg),
        sizeof(MmwDemo_ExtendedMaxVelocityCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_EXTENDED_MAX_VELOCITY;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.extendedMaxVelocityCfg, (void *)&cfg, sizeof(MmwDemo_ExtendedMaxVelocityCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for Near field correction Configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLINearFieldCorrection (int32_t argc, char* argv[])
{
    MmwDemo_NearFieldCorrectionCfg cfg;
    MmwDemo_message                message;
    int8_t                         subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for Near Field Correction */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_NearFieldCorrectionCfg));

    /* Populate configuration: */
    cfg.enabled       = (uint8_t) atoi(argv[2]);
    cfg.startRangeIdx = (uint16_t) atoi(argv[3]);
    cfg.endRangeIdx   = (uint16_t) atoi(argv[4]);


    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, nearFieldCorrectionCfg),
        sizeof(MmwDemo_NearFieldCorrectionCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_NEAR_FIELD_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.nearFieldCorrectionCfg, (void *)&cfg,
           sizeof(MmwDemo_NearFieldCorrectionCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}
#endif

/**
 *  @b Description
 *  @n
 *      Clutter removal Configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIClutterRemoval (int32_t argc, char* argv[])
{
    MmwDemo_ClutterRemovalCfg cfg;
    MmwDemo_message     message;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 3, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for clutter removal */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_ClutterRemovalCfg));

    /* Populate configuration: */
    cfg.enabled          = (uint16_t) atoi (argv[2]);


    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, clutterRemovalCfg),
        sizeof(MmwDemo_ClutterRemovalCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_CLUTTER_REMOVAL;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.clutterRemovalCfg, (void *)&cfg, sizeof(MmwDemo_ClutterRemovalCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

#if 0 //OD DEMO: Remove unused cmds
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for data logger set command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLISetDataLogger (int32_t argc, char* argv[])
{
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }


    /* Save Configuration to use later */
    if (strcmp(argv[1], "mssLogger") == 0)
        gMmwMssMCB.cfg.dataLogger = 0;
    else if (strcmp(argv[1], "dssLogger") == 0)
        gMmwMssMCB.cfg.dataLogger = 1;
    else
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_SET_DATALOGGER;
    message.body.dataLogger = gMmwMssMCB.cfg.dataLogger;

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}
#endif

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for data logger set command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[])
{
    MmwDemo_ADCBufCfg   adcBufCfg;
    MmwDemo_message     message;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&adcBufCfg, 0, sizeof(MmwDemo_ADCBufCfg));

    /* Populate configuration: */
    adcBufCfg.adcFmt          = (uint8_t) atoi (argv[2]);
    adcBufCfg.iqSwapSel       = (uint8_t) atoi (argv[3]);
    adcBufCfg.chInterleave    = (uint8_t) atoi (argv[4]);
    adcBufCfg.chirpThreshold  = (uint8_t) atoi (argv[5]);

    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&adcBufCfg, offsetof(MmwDemo_CliCfg_t, adcBufCfg),
        sizeof(MmwDemo_ADCBufCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_ADCBUFCFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.adcBufCfg, (void *)&adcBufCfg, sizeof(MmwDemo_ADCBufCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


#if 0 //OD DEMO: Remove unused cmds
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for compensation of range bias and channel phase offsets
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    MmwDemo_compRxChannelBiasCfg_t   cfg;
    MmwDemo_message     message;
    int32_t Re, Im;
    int32_t argInd;
    int32_t i;

    /* Sanity Check: Minimum argument check */
    if (argc != (1+1+SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_compRxChannelBiasCfg_t));

    /* Populate configuration: */
    cfg.rangeBias          = (float) atof (argv[1]);

    argInd = 2;
    for (i=0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        Re = (int32_t) (atof (argv[argInd++]) * 32768.);
        Re = MMWDEMO_SATURATE_HIGH(Re);
        Re = MMWDEMO_SATURATE_LOW(Re);
        cfg.rxChPhaseComp[i].real = (int16_t) Re;

        Im = (int32_t) (atof (argv[argInd++]) * 32768.);
        Im = MMWDEMO_SATURATE_HIGH(Im);
        Im = MMWDEMO_SATURATE_LOW(Im);
        cfg.rxChPhaseComp[i].imag = (int16_t) Im;

    }
    /* Save Configuration to use later */
    memcpy((void *) &gMmwMssMCB.cliCommonCfg.compRxChanCfg, &cfg, sizeof(MmwDemo_compRxChannelBiasCfg_t));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE;
    memcpy((void *)&message.body.compRxChanCfg, (void *)&cfg, sizeof(MmwDemo_compRxChannelBiasCfg_t));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for measurement configuration of range bias
 *      and channel phase offsets
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    MmwDemo_measureRxChannelBiasCfg_t   cfg;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_measureRxChannelBiasCfg_t));

    /* Populate configuration: */
    cfg.enabled          = (uint8_t) atoi (argv[1]);
    cfg.targetDistance   = (float) atof (argv[2]);
    cfg.searchWinSize   = (float) atof (argv[3]);

    /* Save Configuration to use later */
    memcpy((void *) &gMmwMssMCB.cliCommonCfg.measureRxChanCfg, &cfg, sizeof(MmwDemo_measureRxChannelBiasCfg_t));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE;
    memcpy((void *)&message.body.measureRxChanCfg, (void *)&cfg, sizeof(MmwDemo_measureRxChannelBiasCfg_t));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for BPM configuration supported by the mmw Demo
 *      Note that there is a generic BPM configuration command supported by
 *      utils/cli and mmwave. The generic BPM command is not supported by the
 *      demo as the mmw demo assumes a specific BPM pattern for the TX antennas.
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIBpmCfg (int32_t argc, char* argv[])
{

    int8_t           subFrameNum;
    MmwDemo_BpmCfg   cfg;
    MmwDemo_message  message;

    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    if(MmwDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_BpmCfg));

    /* Populate configuration: */
    cfg.isEnabled = (bool) atoi(argv[2]) ;
    cfg.chirp0Idx = (uint16_t) atoi(argv[3]) ;
    cfg.chirp1Idx = (uint16_t) atoi(argv[4]) ;

    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, bpmCfg),
        sizeof(MmwDemo_BpmCfg), subFrameNum);

    message.type = MMWDEMO_MSS2DSS_BPM_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.bpmCfg, (void *)&cfg, sizeof(MmwDemo_BpmCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;

}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for configuring CQ RX Saturation monitor
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIChirpQualityRxSatMonCfg (int32_t argc, char* argv[])
{
    rlRxSatMonConf_t        cqSatMonCfg;
    MmwDemo_message         message;

    /* Sanity Check: Minimum argument check */
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cqSatMonCfg, 0, sizeof(rlRxSatMonConf_t));

    /* Populate configuration: */
    cqSatMonCfg.profileIndx                 = (uint8_t) atoi (argv[1]);

    if(cqSatMonCfg.profileIndx < RL_MAX_PROFILES_CNT)
    {

        cqSatMonCfg.satMonSel                   = (uint8_t) atoi (argv[2]);
        cqSatMonCfg.primarySliceDuration        = (uint16_t) atoi (argv[3]);
        cqSatMonCfg.numSlices                   = (uint16_t) atoi (argv[4]);
        cqSatMonCfg.rxChannelMask               = (uint8_t) atoi (argv[5]);

        /* Save Configuration to use later */
        memcpy((void *) &gMmwMssMCB.cliCommonCfg.cqSatMonCfg[cqSatMonCfg.profileIndx],
                       &cqSatMonCfg,
                       sizeof(rlRxSatMonConf_t));

        /* Send configuration to DSS */
        memset((void *)&message, 0, sizeof(MmwDemo_message));
        message.type = MMWDEMO_MSS2DSS_CQ_SATURATION_MONITOR;
        memcpy((void *)&message.body.cqSatMonCfg, (void *)&cqSatMonCfg, sizeof(rlRxSatMonConf_t));

        if (MmwDemo_mboxWrite(&message) == 0)
            return 0;
        else
            return -1;
    }
    else
    {
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for configuring CQ Singal & Image band monitor
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIChirpQualitySigImgMonCfg (int32_t argc, char* argv[])
{
    rlSigImgMonConf_t       cqSigImgMonCfg;
    MmwDemo_message         message;

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cqSigImgMonCfg, 0, sizeof(rlSigImgMonConf_t));

    /* Populate configuration: */
    cqSigImgMonCfg.profileIndx              = (uint8_t) atoi (argv[1]);

    if(cqSigImgMonCfg.profileIndx < RL_MAX_PROFILES_CNT)
    {

        cqSigImgMonCfg.numSlices            = (uint8_t) atoi (argv[2]);
        cqSigImgMonCfg.timeSliceNumSamples  = (uint16_t) atoi (argv[3]);

        /* Save Configuration to use later */
        memcpy((void *) &gMmwMssMCB.cliCommonCfg.cqSigImgMonCfg[cqSigImgMonCfg.profileIndx],
                &cqSigImgMonCfg,
                sizeof(rlSigImgMonConf_t));

        /* Send configuration to DSS */
        memset((void *)&message, 0, sizeof(MmwDemo_message));
        message.type = MMWDEMO_MSS2DSS_CQ_SIGIMG_MONITOR;
        memcpy((void *)&message.body.cqSigImgMonCfg, (void *)&cqSigImgMonCfg, sizeof(rlSigImgMonConf_t));

        if (MmwDemo_mboxWrite(&message) == 0)
            return 0;
        else
            return -1;
    }
    else
    {
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for enabling analog monitors
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIAnalogMonitorCfg (int32_t argc, char* argv[])
{
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Save Configuration to use later */
    gMmwMssMCB.cliCommonCfg.anaMonCfg.rxSatMonEn = atoi (argv[1]);
    gMmwMssMCB.cliCommonCfg.anaMonCfg.sigImgMonEn = atoi (argv[2]);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_ANALOG_MONITOR;
    memcpy((void *)&message.body.anaMonCfg,
            (void *)&gMmwMssMCB.cliCommonCfg.anaMonCfg,
            sizeof(MmwDemo_AnaMonitorCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the High Speed Interface
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLILvdsStreamCfg (int32_t argc, char* argv[])
{

    int8_t                  subFrameNum;
    MmwDemo_LvdsStreamCfg   cfg;
    MmwDemo_message         message;

    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    if(MmwDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_LvdsStreamCfg));

    /* Populate configuration: */
    cfg.isHeaderEnabled = (bool) atoi(argv[2]) ;
    cfg.dataFmt         = (uint8_t) atoi(argv[3]) ;
    cfg.isSwEnabled     = (bool) atoi(argv[4]) ;

    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, lvdsStreamCfg),
        sizeof(MmwDemo_LvdsStreamCfg), subFrameNum);

    message.type = MMWDEMO_MSS2DSS_LVDSSTREAM_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.lvdsStreamCfg, (void *)&cfg, sizeof(MmwDemo_LvdsStreamCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}
#endif

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for gui monitoring configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t ODSDemo_CLIGuiMonSel(int32_t argc, char* argv[])
{
    MmwDemo_GuiMonSel   guiMonSel;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid cfarCfg arg count\n");
        return -1;
    }

    /* Initialize the guiMonSel configuration: */
    memset ((void *)&guiMonSel, 0, sizeof(MmwDemo_GuiMonSel));

    guiMonSel.detectedObjects = atoi (argv[1]);
    guiMonSel.clusteringOutput = atoi (argv[2]);
    guiMonSel.rangeAzimuthHeatMap = atoi (argv[3]);
    guiMonSel.rangeElevationHeatMap = atoi (argv[4]);

    MmwDemo_mssCfgUpdate((void *)&guiMonSel, offsetof(MmwDemo_CliCfg_t, guiMonSel),
        sizeof(MmwDemo_GuiMonSel), -1);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_GUIMON_CFG;
    message.subFrameNum = -1;
    memcpy((void *)&message.body.guiMonSel, (void *)&guiMonSel, sizeof(MmwDemo_GuiMonSel));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_CLIInit (void)
{
    CLI_Cfg     cliCfg;
    char        demoBanner[256];
    uint32_t    cnt;

    /* Create Demo Banner to be printed out by CLI */
    sprintf(&demoBanner[0],
                       "******************************************\n" \
                       "AWR18xx ODS Demo %02d.%02d.%02d.%02d\n"  \
                       "******************************************\n",
                        MMWAVE_SDK_VERSION_MAJOR,
                        MMWAVE_SDK_VERSION_MINOR,
                        MMWAVE_SDK_VERSION_BUGFIX,
                        MMWAVE_SDK_VERSION_BUILD
            );

    /* Initialize the CLI configuration: */
    memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

    /* Populate the CLI configuration: */
    cliCfg.cliPrompt                    = "ODS Demo:/>";
    cliCfg.cliBanner                    = demoBanner;
    cliCfg.socHandle                    = gMmwMssMCB.socHandle;
    cliCfg.cliUartHandle                = gMmwMssMCB.commandUartHandle;
    cliCfg.taskPriority                 = 3;
    cliCfg.mmWaveHandle                 = gMmwMssMCB.ctrlHandle;
    cliCfg.enableMMWaveExtension        = 1U;
    cliCfg.usePolledMode                = true;
    cnt=0;
    cliCfg.tableEntry[cnt].cmd            = "sensorStart";
    cliCfg.tableEntry[cnt].helpString     = "[doReconfig(optional, default:enabled)]";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISensorStart;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "sensorStop";
    cliCfg.tableEntry[cnt].helpString     = "No arguments";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISensorStop;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "guiMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<rangeAzimuthHeatMap> <rangeElevationHeatMap>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = ODSDemo_CLIGuiMonSel;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "cfarCfg";
    cliCfg.tableEntry[cnt].helpString     = "<detectMethod> <leftSkipBin> <closeInRangeBin> <searchWinSizeRange> <searchWinSizeSpreading> <guardSizeRange> <guardSizeSpreading> <rangeThresh> <fftSpreadingThresh> <noiseCalcType> <localPeakEnable> <peakAngleDiffThresh> <maxRangeForDetection>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICfarCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "dbscanCfg";
    cliCfg.tableEntry[cnt].helpString     = "<nAccFrames> <epsilon> <weight> <vfactor> <minPointsInCluster> <inputScale>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIDbscanCfg;
    cnt++;

#if 0 //ODS DEMO: Remove unused cmds
    cliCfg.tableEntry[cnt].cmd            = "peakGrouping";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <groupingMode> <rangeDimEn> <dopplerDimEn> <startRangeIdx> <endRangeIdx>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIPeakGroupingCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "dataLogger";
    cliCfg.tableEntry[cnt].helpString     = "<mssLogger | dssLogger>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISetDataLogger;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "multiObjBeamForming";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <threshold>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMultiObjBeamForming;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "extendedMaxVelocity";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIExtendedMaxVelocity;
    cnt++;
#endif

    cliCfg.tableEntry[cnt].cmd            = "calibDcRangeSig";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <negativeBinIdx> <positiveBinIdx> <numAvgFrames>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICalibDcRangeSig;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "clutterRemoval";
    cliCfg.tableEntry[cnt].helpString     = "<enabled>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIClutterRemoval;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "adcbufCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <adcOutputFmt> <SampleSwap> <ChanInterleave> <ChirpThreshold>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIADCBufCfg;
    cnt++;

#if 0 //ODS DEMO: Remove unused cmds
    cliCfg.tableEntry[cnt].cmd            = "compRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[cnt].helpString     = "<rangeBias> <Re00> <Im00> <Re01> <Im01> <Re02> <Im02> <Re03> <Im03> <Re10> <Im10> <Re11> <Im11> <Re12> <Im12> <Re13> <Im13> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "measureRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[cnt].helpString     = "<enabled> <targetDistance> <searchWin>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "bpmCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <chirp0Idx> <chirp1Idx>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIBpmCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "nearFieldCfg";
    cliCfg.tableEntry[cnt].helpString     = "<enabled> <startRangeIndex> <endRangeIndex>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLINearFieldCorrection;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "CQRxSatMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<profile> <satMonSel> <priSliceDuration> <numSlices> <rxChanMask>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIChirpQualityRxSatMonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "CQSigImgMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<profile> <numSlices> <numSamplePerSlice>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIChirpQualitySigImgMonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "analogMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<rxSaturation> <sigImgBand>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIAnalogMonitorCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "lvdsStreamCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enableHeader> <dataFmt> <enableSW>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLILvdsStreamCfg;
    cnt++;
#endif

    /* Open the CLI: */
    if (CLI_open (&cliCfg) < 0)
    {
        System_printf ("Error: Unable to open the CLI\n");
        return;
    }
    System_printf ("Debug: CLI is operational\n");
    return;
}
