/*!
 *  \file   RADARDEMO_aoaEstBF2D.c
 *  \brief   Estimate the angle of arrival using Capon BF.
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
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
 *
*/

#include <math.h>
#include <string.h>

/* C674x mathlib */
#include <ti/mathlib/mathlib.h>
#include "radar_c674x.h"
#include "DSPF_sp_fftSPxSP.h"
#include "./common/odsdemo_common.h"
#include "./common/mmw_config.h"
#include <float.h>
#include "dss_mmw.h"
#include "swpform.h"
#include "odsdemo_aoaEstBF2D.h"

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
//#include "c6x.h"
#endif


extern ODSDemo_DataPathObj odsdemo_dataPathObj;
extern MmwDemo_DSS_MCB     gMmwDssMCB;

ODSDEMO_aoAEstBF2D_output  odsdemo_doaOutput;
extern uint16_t antenna2DMAP_a[ODSDEMO_MAX_RX_ANT];
extern uint16_t antenna2DMAP_e[ODSDEMO_MAX_RX_ANT];

#define SECOND_PEAK_SEARCH /* move this out of the code - add to project defines */

#ifdef SECOND_PEAK_SEARCH
uint16_t isInBox(int16_t row, int16_t col, uint16_t *idx)
{
    uint16_t inRow = 0;
    uint16_t inCol = 0;

    //check row
    if (idx[0] == idx[1]) // if top == bottom
        inRow = 1;
    else if (idx[0] < idx[1])
        inRow = ((row > idx[0]) && (row < idx[1])); //idx[0] = top, idx[1] = bottom
    else
        inRow = ((row > idx[0]) || (row < idx[1]));

    //check col
    if (idx[2] == idx[3]) // if left == right
        inCol = 1;
    else if (idx[2] < idx[3])
        inCol = ((col > idx[2]) && (col < idx[3])); //idx[2] = left, idx[3] = right
    else
        inCol = ((col > idx[2]) || (col < idx[3]));

    return inRow && inCol;
}

#define DOA_FFTSIZE_MINUS1   (ODSDEMO_DOA2D_FFTSIZE - 1)

//This function locates the edges of the first detected peak, searching
//circularly when the physical edge is reached.
void peakEdgeSearch(int16_t row_idx, int16_t col_idx, float *mag_data, uint16_t *idx_vals)
{
    uint16_t topSearchIdx;
    uint16_t bottomSearchIdx;
    uint16_t leftSearchIdx;
    uint16_t rightSearchIdx;
    uint32_t offset1;
    uint32_t offset2;
    uint32_t row_offset;
    uint16_t row, col;
    int16_t  loops;

    /* Find top edge of the peak */
    topSearchIdx = row_idx;
    row = (row_idx - 1) & DOA_FFTSIZE_MINUS1;
    offset1 = (topSearchIdx * ODSDEMO_DOA2D_FFTSIZE) + col_idx;
    offset2 = (row * ODSDEMO_DOA2D_FFTSIZE) + col_idx;
    loops = ODSDEMO_DOA2D_FFTSIZE;

    while ((mag_data[offset1] >= mag_data[offset2]) && (loops > 0))
    {
        topSearchIdx = (topSearchIdx - 1) & DOA_FFTSIZE_MINUS1;
        row = (row - 1) & DOA_FFTSIZE_MINUS1;
        offset1 = (topSearchIdx * ODSDEMO_DOA2D_FFTSIZE) + col_idx;
        offset2 = (row * ODSDEMO_DOA2D_FFTSIZE) + col_idx;
        loops--;
    }

    /* Find bottom edge of the peak */
    bottomSearchIdx = row_idx;
    row = (row_idx + 1) & DOA_FFTSIZE_MINUS1;
    offset1 = (bottomSearchIdx * ODSDEMO_DOA2D_FFTSIZE) + col_idx;
    offset2 = (row * ODSDEMO_DOA2D_FFTSIZE) + col_idx;
    loops = ODSDEMO_DOA2D_FFTSIZE;

    while ((mag_data[offset1] >= mag_data[offset2]) && (loops > 0))
    {
        bottomSearchIdx = (bottomSearchIdx + 1) & DOA_FFTSIZE_MINUS1;
        row = (row + 1) & DOA_FFTSIZE_MINUS1;
        offset1 = (bottomSearchIdx * ODSDEMO_DOA2D_FFTSIZE) + col_idx;
        offset2 = (row * ODSDEMO_DOA2D_FFTSIZE) + col_idx;
        loops --;

        if (bottomSearchIdx == topSearchIdx) break;
    }

    /* Find left edge of the peak */
    leftSearchIdx = col_idx;
    col = (col_idx - 1) & DOA_FFTSIZE_MINUS1;
    row_offset = row_idx * ODSDEMO_DOA2D_FFTSIZE;
    offset1 = row_offset + leftSearchIdx;
    offset2 = row_offset + col;
    loops = ODSDEMO_DOA2D_FFTSIZE;

    while ((mag_data[offset1] >= mag_data[offset2]) && (loops > 0))
    {
        leftSearchIdx = (leftSearchIdx - 1) & DOA_FFTSIZE_MINUS1;
        col = (col - 1) & DOA_FFTSIZE_MINUS1;
        offset1 = row_offset + leftSearchIdx;
        offset2 = row_offset + col;
        loops--;
    }

    /* Find right edge of the peak */
    rightSearchIdx = col_idx;
    col = (col_idx + 1) & DOA_FFTSIZE_MINUS1;
    offset1 = row_offset + rightSearchIdx;
    offset2 = row_offset + col;
    loops = ODSDEMO_DOA2D_FFTSIZE;

    while ((mag_data[offset1] >= mag_data[offset2]) && (loops > 0))
    {
        rightSearchIdx = (rightSearchIdx + 1) & DOA_FFTSIZE_MINUS1;
        col = (col + 1) & DOA_FFTSIZE_MINUS1;
        offset1 = row_offset + rightSearchIdx;
        offset2 = row_offset + col;
        loops--;

        if (rightSearchIdx == leftSearchIdx) break;
    }

    idx_vals[0] = topSearchIdx;
    idx_vals[1] = bottomSearchIdx;
    idx_vals[2] = leftSearchIdx;
    idx_vals[3] = rightSearchIdx;
}

int16_t  first_peak_row_idx;
int16_t  first_peak_col_idx;
#endif


//! \copydoc ODSDEMO__aoaEstBF2D
ODSDEMO_aoaEstBF2D_errorCode    ODSDEMO__aoaEstBF2D(
                            IN  float range,
                            IN  float speed,
                            IN  uint16_t detIdx,
                            IN  cplxf_t * inputSignal,
                            OUT ODSDEMO_aoAEstBF2D_output * estOutput)
{
    uint16_t    antInd, eInd, aInd, col_idx, row_idx, offset, bufferSize;
    int16_t     fft_2D_peak_row_idx, fft_2D_peak_col_idx;
    float       x, y, z;
    float       el_freq, az_freq;
    float       theta, phi, temp;
    float       maxVal, mag_sqr;
    unsigned char *brev = NULL;
    cplxfReIm_t   *scratchPadPtr;
    cplxfReIm_t   *DOA_2D_storage;
    float       *fftIn, *fftOut;

    #ifdef SECOND_PEAK_SEARCH
    float *secondFftData = gMmwDssMCB.dataPathObj[0].second_fftData;
    MmwDemo_CfarCfg *cfarCfg = &gMmwDssMCB.cliCfg[0].cfarCfg;
    #endif

    ODSDEMO_aoaEstBF2D_errorCode errorCode = ODSDEMO_AOABF2D_NO_ERROR;

    scratchPadPtr = (cplxfReIm_t *)&odsdemo_dataPathObj.scratchPad[0];
    fftIn  = (float *)&scratchPadPtr[0];
    fftOut = (float *)&scratchPadPtr[ODSDEMO_DOA2D_FFTSIZE];
    DOA_2D_storage =  &scratchPadPtr[(ODSDEMO_DOA2D_FFTSIZE << 1)];

    // set zero to all the entry in DOA_2D_storage
    bufferSize = ODSDEMO_DOA2D_FFTSIZE * (odsdemo_dataPathObj.maxIndElevationAntMapping + 1) * sizeof(cplxfReIm_t);
    memset(DOA_2D_storage, 0, bufferSize);

    /*Arrange the complex input across virtual antennas in 2D grid based on ODS antenna placement*/
    //This fills in data into DOA_2D - for elev * azim antennas - the rest are zero.
    //For non-rectangular virtual antenna arrays, this will leave zeros where no antenna is.
    for (antInd = 0; antInd < gMmwDssMCB.dataPathObj[0].numVirtualAntennas; antInd ++)
    {
        aInd = antenna2DMAP_a[antInd];
        eInd = antenna2DMAP_e[antInd];
        DOA_2D_storage[eInd * ODSDEMO_DOA2D_FFTSIZE + aInd].real = inputSignal[antInd].real;
        DOA_2D_storage[eInd * ODSDEMO_DOA2D_FFTSIZE + aInd].imag = inputSignal[antInd].imag;
    }

    // first FFT
    offset = 0;
    for (eInd = 0; eInd <= odsdemo_dataPathObj.maxIndElevationAntMapping; eInd ++)
    {
        // FFT input take the real first
        DSPF_sp_fftSPxSP (
                ODSDEMO_DOA2D_FFTSIZE,
                (float *)&DOA_2D_storage[offset],
                (float *)&gMmwDssMCB.dataPathObj[0].twiddle_DOA[0],
                fftOut,
                brev,
                gMmwDssMCB.dataPathObj[0].DOA2D_rad2D,
                0,
                ODSDEMO_DOA2D_FFTSIZE);

        // FFT output give the real first
        //this code writes the 64 complex points into the current row of DOA_2D_
        for (col_idx = 0;col_idx < ODSDEMO_DOA2D_FFTSIZE; col_idx++)
        {
            DOA_2D_storage[offset + col_idx].real = fftOut[col_idx * 2];
            DOA_2D_storage[offset + col_idx].imag = fftOut[col_idx * 2 + 1];
        }
        offset += ODSDEMO_DOA2D_FFTSIZE;
    }

    #ifdef SECOND_PEAK_SEARCH
    memset(secondFftData, 0, ODSDEMO_DOA2D_FFTSIZE * ODSDEMO_DOA2D_FFTSIZE * sizeof(float));
    #endif

    // second FFT
    maxVal = 0;
    bufferSize = ODSDEMO_DOA2D_FFTSIZE * sizeof(cplxfReIm_t);
    for (col_idx = 0; col_idx < ODSDEMO_DOA2D_FFTSIZE; col_idx ++)
    {
        // FFT input take the real first
        offset = 0;
        memset(fftIn, 0, bufferSize);

        //this code takes a column out of DOA_2D (# elev) and the rest are zeros.
        for (row_idx = 0; row_idx <= odsdemo_dataPathObj.maxIndElevationAntMapping; row_idx ++)
        {
            fftIn[row_idx * 2] = DOA_2D_storage[offset + col_idx].real;
            fftIn[row_idx * 2 + 1] = DOA_2D_storage[offset + col_idx].imag;
            offset += ODSDEMO_DOA2D_FFTSIZE;
        }

        DSPF_sp_fftSPxSP (
                ODSDEMO_DOA2D_FFTSIZE,
                fftIn,
                (float *)&gMmwDssMCB.dataPathObj[0].twiddle_DOA[0],
                fftOut,
                brev,
                gMmwDssMCB.dataPathObj[0].DOA2D_rad2D,
                0,
                ODSDEMO_DOA2D_FFTSIZE);

        // FFT output give real first.  Instead of storing the FFT output
        // only record the peak.
        offset = 0;
        for (row_idx = 0; row_idx < ODSDEMO_DOA2D_FFTSIZE; row_idx++)
        {
            //fft output is real (float) followed by imag (float) times 64.
            mag_sqr = fftOut[offset] * fftOut[offset] + fftOut[offset+1] * fftOut[offset+1];
            if (mag_sqr > maxVal)
            {
                fft_2D_peak_row_idx = row_idx;
                fft_2D_peak_col_idx = col_idx;
                maxVal = mag_sqr;
            }

            #ifdef SECOND_PEAK_SEARCH
            //Store the data by row, like the DOA_2D_storage table
            secondFftData[row_idx * ODSDEMO_DOA2D_FFTSIZE + col_idx] = mag_sqr;
            first_peak_row_idx = fft_2D_peak_row_idx;
            first_peak_col_idx = fft_2D_peak_col_idx;
            #endif

            offset += 2;
        }
    }

    /* convert the peak indices b/w [-Fs/2, Fs/2]*/
    if (fft_2D_peak_row_idx > ODSDEMO_DOA2D_HALFFFTSIZE)
    {
        fft_2D_peak_row_idx -= ODSDEMO_DOA2D_FFTSIZE;
    }

    if (fft_2D_peak_col_idx > ODSDEMO_DOA2D_HALFFFTSIZE)
    {
        fft_2D_peak_col_idx -= ODSDEMO_DOA2D_FFTSIZE;
    }

    /* Based on detected peak indices,  compute the azimuth and elevation frequency corresponding to peak value */
    el_freq = fft_2D_peak_row_idx * ODSDEMO_DOA2D_HALFFFTSIZEINV;
    az_freq = fft_2D_peak_col_idx * ODSDEMO_DOA2D_HALFFFTSIZEINV;

    /*Compute the elevation angle */
    phi= (float)(asin(el_freq));
    temp = az_freq/((float)(cos(phi)));

    /*Check if azimuth angle can be computed or not */
    if ((temp <= 1.0f) && (temp >= -1.0f))
    {
        theta =  (float)(asin(temp));
    }
    else
    {
        errorCode = ODSDEMO_AOABF2D_FAILTOGIVEOUTPUT;
        return(errorCode);
    }

    /* Compute (x,y,z) cordinates of the detected object */
    estOutput->x = range*(float)(sin(theta))*(float)(cos(phi));
    estOutput->y = range*(float)(cos(theta))*(float)(cos(phi));
    estOutput->z = range*(float)(sin(phi));
    estOutput->aAngleEst = theta;
    estOutput->eAngleEst = phi;

    #ifdef SECOND_PEAK_SEARCH
    uint16_t idx_vals[4];
    float    firstMaxVal = maxVal;
    maxVal = 0;

    // Find the rectangle containing the first peak.
    peakEdgeSearch(first_peak_row_idx, first_peak_col_idx, secondFftData, idx_vals);

    /* Find the second peak value in the magnitude 2D-DOA */
    for (row_idx = 0; row_idx < ODSDEMO_DOA2D_FFTSIZE; row_idx++)
    {
        for (col_idx = 0; col_idx < ODSDEMO_DOA2D_FFTSIZE; col_idx++)
        {
            if (!isInBox(row_idx, col_idx, idx_vals))
            {
                mag_sqr = secondFftData[(row_idx * ODSDEMO_DOA2D_FFTSIZE) + col_idx];

                if (mag_sqr > maxVal)
                {
                    fft_2D_peak_row_idx = row_idx;
                    fft_2D_peak_col_idx = col_idx;
                    maxVal = mag_sqr;
                }
            }
        }
    }

    //Reject the second peak if it's not strong enough
    if (maxVal < (cfarCfg->secondPeakThresh * firstMaxVal))
    {
        return(errorCode);
    }

    /* convert the second peak indices b/w [-Fs/2, Fs/2]*/
    if (fft_2D_peak_row_idx > ODSDEMO_DOA2D_HALFFFTSIZE)
    {
        fft_2D_peak_row_idx -= ODSDEMO_DOA2D_FFTSIZE;
    }

    if (fft_2D_peak_col_idx > ODSDEMO_DOA2D_HALFFFTSIZE)
    {
        fft_2D_peak_col_idx -= ODSDEMO_DOA2D_FFTSIZE;
    }

    /* Based on detected peak indices,  compute the azimuth and elevation frequency corresponding to peak value */
    el_freq = fft_2D_peak_row_idx * ODSDEMO_DOA2D_HALFFFTSIZEINV;
    az_freq = fft_2D_peak_col_idx * ODSDEMO_DOA2D_HALFFFTSIZEINV;

    /*Compute the elevation angle */
    phi= (float)(asin(el_freq));
    temp = az_freq/((float)(cos(phi)));

    /*Check if azimuth angle can be computed or not */
    if ((temp <= 1.0f) && (temp >= -1.0f))
    {
        theta = (float)(asin(temp));
    }
    else
    {
        return(errorCode);
    }

    x = (int16_t)(range*(float)(sin(theta))*(float)(cos(phi)) * gMmwDssMCB.cliCfg[0].dbscanCfg.fixedPointScale + 0.5);
    y = (int16_t)(range*(float)(cos(theta))*(float)(cos(phi)) * gMmwDssMCB.cliCfg[0].dbscanCfg.fixedPointScale + 0.5);
    z = (int16_t)(range*(float)(sin(phi)) * gMmwDssMCB.cliCfg[0].dbscanCfg.fixedPointScale + 0.5);

    if (odsdemo_dataPathObj.dbscanInputData->numPoints < ODSDEMO_DBSCAN_MAXINPUTPOINTS)
    {
        ODSDemo_clusteringDBscan3DPoint3dfxdp *pointArrayPtr;
        pointArrayPtr = &odsdemo_dataPathObj.dbscanInputData->pointArray[odsdemo_dataPathObj.dbscanInputData->numPoints];

        odsdemo_dataPathObj.dbscanInputData->numPoints ++;

        /* Compute (x,y,z) cordinates of the detected object */
        pointArrayPtr->x = x;
        pointArrayPtr->y = y;
        pointArrayPtr->z = z;
    }

    // Record the second peak detection output. This will appear in the .detObjRaw
    // table before the primary peak, because the primary is not stored until this
    // function exits.
    odsdemo_dataPathObj.detObjRaw[odsdemo_dataPathObj.numDetObj].x = x;
    odsdemo_dataPathObj.detObjRaw[odsdemo_dataPathObj.numDetObj].y = y;
    odsdemo_dataPathObj.detObjRaw[odsdemo_dataPathObj.numDetObj].z = z;
    odsdemo_dataPathObj.detObjRaw[odsdemo_dataPathObj.numDetObj].speed = speed;
    odsdemo_dataPathObj.detObjRaw[odsdemo_dataPathObj.numDetObj].speedIdx = odsdemo_dataPathObj.dopplerProcOut->dopplerPeakIndex;
    odsdemo_dataPathObj.detObjRaw[odsdemo_dataPathObj.numDetObj].range = odsdemo_dataPathObj.cfarOut->rangeEst[detIdx];
    odsdemo_dataPathObj.detObjRaw[odsdemo_dataPathObj.numDetObj].rangeIdx = odsdemo_dataPathObj.cfarOut->rangeInd[detIdx];
    odsdemo_dataPathObj.detObjRaw[odsdemo_dataPathObj.numDetObj].snrEst = odsdemo_dataPathObj.cfarOut->snrEst[detIdx];;
    odsdemo_dataPathObj.numDetObj ++;
    #endif

    return(errorCode);
}
