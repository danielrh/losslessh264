/*!
 * \copy
 *     Copyright (c)  2008-2013, Cisco Systems
 *     All rights reserved.
 *
 *     Redistribution and use in source and binary forms, with or without
 *     modification, are permitted provided that the following conditions
 *     are met:
 *
 *        * Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *
 *        * Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in
 *          the documentation and/or other materials provided with the
 *          distribution.
 *
 *     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *     COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *     POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *  Abstract
 *      current slice decoding
 *
 *  History
 *      07/10/2008 Created
 *      08/09/2013 Modified
 *
 *****************************************************************************/


#include "deblocking.h"

#include "decode_slice.h"

#include "parse_mb_syn_cavlc.h"
#include "parse_mb_syn_cabac.h"
#include "rec_mb.h"
#include "mv_pred.h"

#include "cpu_core.h"
#include "compression_stream.h"
#include "decoded_macroblock.h"
#include "macroblock_model.h"

#include "encoder_from_decoder.h"

void DecodedMacroblock::preInit(const WelsDec::PSlice pSlice) {
    WelsDec::PSliceHeader pSliceHeader = &pSlice->sSliceHeaderExt.sSliceHeader;
    iLastMbQp = pSlice->iLastMbQp;
    eSliceType = pSliceHeader->eSliceType;
    uiChromaQpIndexOffset = pSliceHeader->pPps->iChromaQpIndexOffset[0];
}

namespace WelsDec {
static void initCoeffsFromCoefPtr(DecodedMacroblock &rtd,
                                  int16_t * pScaledTCoeffQorNot) {
    memcpy(rtd.odata.lumaAC, pScaledTCoeffQorNot, sizeof(rtd.odata.lumaAC));
    memcpy(rtd.odata.chromaAC, pScaledTCoeffQorNot + sizeof(rtd.odata.lumaAC) / sizeof(rtd.odata.lumaAC[0]), sizeof(rtd.odata.chromaAC));
    for (int i = 0; i < 256; i += 16) {
        rtd.odata.lumaDC[i / 16] = pScaledTCoeffQorNot[i];
    }
    for (int i = 256; i < 384; i += 16) {
        rtd.odata.chromaDC[(i / 16) - 16] = pScaledTCoeffQorNot[i];
    }
}

static void initRTDFromDecoderState(DecodedMacroblock &rtd,
        PDqLayer pCurLayer) {
    uint32_t iMbXy = pCurLayer->iMbXyIndex;
    PSlice decoderpSlice = &pCurLayer->sLayerInfo.sSliceInLayer;
    PSliceHeader pSliceHeader = &decoderpSlice->sSliceHeaderExt.sSliceHeader;

    int uiCbp = pCurLayer->pCbp[pCurLayer->iMbXyIndex];
    rtd.uiCbpC = uiCbp >> 4;
    rtd.uiCbpL = uiCbp & 15;

    rtd.eSliceType = pSliceHeader->eSliceType;
    rtd.uiChromaQpIndexOffset = pSliceHeader->pPps->iChromaQpIndexOffset[0];

    rtd.uiChmaI8x8Mode = pCurLayer->pChromaPredMode[iMbXy];
    //from WelsDec::WelsActualDecodeMbCavlcISlice pCurLayer->pIntraPredMode[iMbXy][7] = (uiMbType - 1) & 3
    rtd.uiLumaI16x16Mode = pCurLayer->pIntraPredMode[iMbXy][7];
    rtd.uiMbType = pCurLayer->pMbType[iMbXy];
    for (int i = 0; i < 4; i++) {
        // only 8x8
        rtd.uiSubMbType[i] = pCurLayer->pSubMbType[iMbXy][i];
    }
    rtd.uiNumRefIdxL0Active = pSliceHeader->uiRefCount[0]; // Number of reference frames.
    rtd.uiLumaQp = pCurLayer->pLumaQp[iMbXy];

    initCoeffsFromCoefPtr(rtd, pCurLayer->pScaledTCoeffQuant[iMbXy]);
}

int32_t WelsTargetSliceConstruction (PWelsDecoderContext pCtx) {
  PDqLayer pCurLayer = pCtx->pCurDqLayer;
  PSlice pCurSlice = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeader pSliceHeader = &pCurSlice->sSliceHeaderExt.sSliceHeader;

  int32_t iTotalMbTargetLayer = pSliceHeader->pSps->uiTotalMbCount;

  int32_t iCurLayerWidth  = pCurLayer->iMbWidth << 4;
  int32_t iCurLayerHeight = pCurLayer->iMbHeight << 4;

  int32_t iNextMbXyIndex = 0;
  PFmo pFmo = pCtx->pFmo;

  int32_t iTotalNumMb = pCurSlice->iTotalMbInCurSlice;
  int32_t iCountNumMb = 0;
  PDeblockingFilterMbFunc pDeblockMb;

  if (!pCtx->bAvcBasedFlag && iCurLayerWidth != pCtx->iCurSeqIntervalMaxPicWidth) {
    return -1;
  }

  iNextMbXyIndex   = pSliceHeader->iFirstMbInSlice;
  pCurLayer->iMbX  = iNextMbXyIndex % pCurLayer->iMbWidth;
  pCurLayer->iMbY  = iNextMbXyIndex / pCurLayer->iMbWidth;
  pCurLayer->iMbXyIndex = iNextMbXyIndex;

  if (0 == iNextMbXyIndex) {
    pCurLayer->pDec->iSpsId = pCtx->pSps->iSpsId;
    pCurLayer->pDec->iPpsId = pCtx->pPps->iPpsId;

    pCurLayer->pDec->uiQualityId = pCurLayer->sLayerInfo.sNalHeaderExt.uiQualityId;
  }

  do {
    if (iCountNumMb >= iTotalNumMb) {
      break;
    }

    if (!pCtx->bParseOnly) { //for parse only, actual recon MB unnecessary
      if (WelsTargetMbConstruction (pCtx)) {
        WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING,
                 "WelsTargetSliceConstruction():::MB(%d, %d) construction error. pCurSlice_type:%d",
                 pCurLayer->iMbX, pCurLayer->iMbY, pCurSlice->eSliceType);

        return -1;
      }
    }

    ++iCountNumMb;
    if (!pCurLayer->pMbCorrectlyDecodedFlag[iNextMbXyIndex]) { //already con-ed, overwrite
      pCurLayer->pMbCorrectlyDecodedFlag[iNextMbXyIndex] = true;
      pCtx->pDec->iMbEcedPropNum += (pCurLayer->pMbRefConcealedFlag[iNextMbXyIndex] ? 1 : 0);
      ++pCtx->iTotalNumMbRec;
    }

    if (pCtx->iTotalNumMbRec > iTotalMbTargetLayer) {
      WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING,
               "WelsTargetSliceConstruction():::pCtx->iTotalNumMbRec:%d, iTotalMbTargetLayer:%d",
               pCtx->iTotalNumMbRec, iTotalMbTargetLayer);

      return -1;
    }

    if (pSliceHeader->pPps->uiNumSliceGroups > 1) {
      iNextMbXyIndex = FmoNextMb (pFmo, iNextMbXyIndex);
    } else {
      ++iNextMbXyIndex;
    }
    if (-1 == iNextMbXyIndex || iNextMbXyIndex >= iTotalMbTargetLayer) { // slice group boundary or end of a frame
      break;
    }
    pCurLayer->iMbX  = iNextMbXyIndex % pCurLayer->iMbWidth;
    pCurLayer->iMbY  = iNextMbXyIndex / pCurLayer->iMbWidth;
    pCurLayer->iMbXyIndex = iNextMbXyIndex;
  } while (1);

  pCtx->pDec->iWidthInPixel  = iCurLayerWidth;
  pCtx->pDec->iHeightInPixel = iCurLayerHeight;

  if ((pCurSlice->eSliceType != I_SLICE) && (pCurSlice->eSliceType != P_SLICE))
    return 0;

  if (pCtx->bParseOnly) //for parse only, deblocking should not go on
    return 0;

  pDeblockMb = WelsDeblockingMb;

  if (1 == pSliceHeader->uiDisableDeblockingFilterIdc
      || pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer.iTotalMbInCurSlice <= 0) {
    return 0;//NO_SUPPORTED_FILTER_IDX
  } else {
    WelsDeblockingFilterSlice (pCtx, pDeblockMb);
  }
  // any other filter_idc not supported here, 7/22/2010

  return 0;
}

int32_t WelsMbInterSampleConstruction (PWelsDecoderContext pCtx, PDqLayer pCurLayer,
                                       uint8_t* pDstY, uint8_t* pDstU, uint8_t* pDstV, int32_t iStrideL, int32_t iStrideC) {
  int32_t iMbXy = pCurLayer->iMbXyIndex;
  int32_t i, iIndex, iOffset;

  WelsChromaDcIdct (pCurLayer->pScaledTCoeff[iMbXy] + 256);     // 256 = 16*16
  WelsChromaDcIdct (pCurLayer->pScaledTCoeff[iMbXy] + 320);     // 320 = 16*16 + 16*4

  if (pCurLayer->pTransformSize8x8Flag[iMbXy]) {
    for (i = 0; i < 4; i++) {
      iIndex = g_kuiMbCountScan4Idx[i << 2];
      if (pCurLayer->pNzc[iMbXy][iIndex] || pCurLayer->pNzc[iMbXy][iIndex + 1] || pCurLayer->pNzc[iMbXy][iIndex + 4]
          || pCurLayer->pNzc[iMbXy][iIndex + 5]) {
        iOffset = ((iIndex >> 2) << 2) * iStrideL + ((iIndex % 4) << 2);
        pCtx->pIdctResAddPredFunc8x8 (pDstY + iOffset, iStrideL, pCurLayer->pScaledTCoeff[iMbXy] + (i << 6));
      }
    }
  } else {
    for (i = 0; i < 16; i++) { //luma
      iIndex = g_kuiMbCountScan4Idx[i];
      if (pCurLayer->pNzc[iMbXy][iIndex]) {
        iOffset = ((iIndex >> 2) << 2) * iStrideL + ((iIndex % 4) << 2);
        pCtx->pIdctResAddPredFunc (pDstY + iOffset, iStrideL, pCurLayer->pScaledTCoeff[iMbXy] + (i << 4));
      }
    }
  }

  for (i = 0; i < 4; i++) { //chroma
    iIndex = g_kuiMbCountScan4Idx[i + 16]; //Cb
    if (pCurLayer->pNzc[iMbXy][iIndex] || * (pCurLayer->pScaledTCoeff[iMbXy] + ((i + 16) << 4))) {
      iOffset = (((iIndex - 16) >> 2) << 2) * iStrideC + (((iIndex - 16) % 4) << 2);
      pCtx->pIdctResAddPredFunc (pDstU + iOffset, iStrideC, pCurLayer->pScaledTCoeff[iMbXy] + ((i + 16) << 4));
    }

    iIndex = g_kuiMbCountScan4Idx[i + 20]; //Cr
    if (pCurLayer->pNzc[iMbXy][iIndex] || * (pCurLayer->pScaledTCoeff[iMbXy] + ((i + 20) << 4))) {
      iOffset = (((iIndex - 18) >> 2) << 2) * iStrideC + (((iIndex - 18) % 4) << 2);
      pCtx->pIdctResAddPredFunc (pDstV + iOffset, iStrideC , pCurLayer->pScaledTCoeff[iMbXy] + ((i + 20) << 4));
    }
  }

  return 0;
}
int32_t WelsMbInterConstruction (PWelsDecoderContext pCtx, PDqLayer pCurLayer) {
  int32_t iMbX = pCurLayer->iMbX;
  int32_t iMbY = pCurLayer->iMbY;
  uint8_t*  pDstY, *pDstCb, *pDstCr;

  int32_t iLumaStride   = pCtx->pDec->iLinesize[0];
  int32_t iChromaStride = pCtx->pDec->iLinesize[1];

  pDstY  = pCurLayer->pDec->pData[0] + ((iMbY * iLumaStride + iMbX) << 4);
  pDstCb = pCurLayer->pDec->pData[1] + ((iMbY * iChromaStride + iMbX) << 3);
  pDstCr = pCurLayer->pDec->pData[2] + ((iMbY * iChromaStride + iMbX) << 3);

  GetInterPred (pDstY, pDstCb, pDstCr, pCtx);
  WelsMbInterSampleConstruction (pCtx, pCurLayer, pDstY, pDstCb, pDstCr, iLumaStride, iChromaStride);

  pCtx->sBlockFunc.pWelsSetNonZeroCountFunc (
    pCurLayer->pNzc[pCurLayer->iMbXyIndex]); // set all none-zero nzc to 1; dbk can be opti!
  return 0;
}

void WelsLumaDcDequantIdct (int16_t* pBlock, int32_t iQp, PWelsDecoderContext pCtx) {
  const int32_t kiQMul = pCtx->bUseScalingList ? pCtx->pDequant_coeff4x4[0][iQp][0] >> 4 : g_kuiDequantCoeff[iQp][0];
#define STRIDE 16
  int32_t i;
  int32_t iTemp[16]; //FIXME check if this is a good idea
  int16_t* pBlk = pBlock;
  static const int32_t kiXOffset[4] = {0, STRIDE, STRIDE << 2,  5 * STRIDE};
  static const int32_t kiYOffset[4] = {0, STRIDE << 1, STRIDE << 3, 10 * STRIDE};

  for (i = 0; i < 4; i++) {
    const int32_t kiOffset = kiYOffset[i];
    const int32_t kiX1 = kiOffset + kiXOffset[2];
    const int32_t kiX2 = STRIDE + kiOffset;
    const int32_t kiX3 = kiOffset + kiXOffset[3];
    const int32_t kiI4 = i << 2; // 4*i
    const int32_t kiZ0 = pBlk[kiOffset] + pBlk[kiX1];
    const int32_t kiZ1 = pBlk[kiOffset] - pBlk[kiX1];
    const int32_t kiZ2 = pBlk[kiX2] - pBlk[kiX3];
    const int32_t kiZ3 = pBlk[kiX2] + pBlk[kiX3];

    iTemp[kiI4]  = kiZ0 + kiZ3;
    iTemp[1 + kiI4] = kiZ1 + kiZ2;
    iTemp[2 + kiI4] = kiZ1 - kiZ2;
    iTemp[3 + kiI4] = kiZ0 - kiZ3;
  }

  for (i = 0; i < 4; i++) {
    const int32_t kiOffset = kiXOffset[i];
    const int32_t kiI4 = 4 + i;
    const int32_t kiZ0 = iTemp[i] + iTemp[4 + kiI4];
    const int32_t kiZ1 = iTemp[i] - iTemp[4 + kiI4];
    const int32_t kiZ2 = iTemp[kiI4] - iTemp[8 + kiI4];
    const int32_t kiZ3 = iTemp[kiI4] + iTemp[8 + kiI4];

    pBlk[kiOffset] = ((kiZ0 + kiZ3) * kiQMul + 2) >> 2; //FIXME think about merging this into decode_resdual
    pBlk[kiYOffset[1] + kiOffset] = ((kiZ1 + kiZ2) * kiQMul + 2) >> 2;
    pBlk[kiYOffset[2] + kiOffset] = ((kiZ1 - kiZ2) * kiQMul + 2) >> 2;
    pBlk[kiYOffset[3] + kiOffset] = ((kiZ0 - kiZ3) * kiQMul + 2) >> 2;
  }
#undef STRIDE
}

int32_t WelsMbIntraPredictionConstruction (PWelsDecoderContext pCtx, PDqLayer pCurLayer, bool bOutput) {
//seems IPCM should not enter this path
  int32_t iMbXy = pCurLayer->iMbXyIndex;

  WelsFillRecNeededMbInfo (pCtx, bOutput, pCurLayer);

  if (IS_INTRA16x16 (pCurLayer->pMbType[iMbXy])) {
    WelsLumaDcDequantIdct (pCurLayer->pScaledTCoeff[iMbXy], pCurLayer->pLumaQp[iMbXy], pCtx);
    RecI16x16Mb (iMbXy, pCtx, pCurLayer->pScaledTCoeff[iMbXy], pCurLayer);

    return 0;
  }

  if (IS_INTRA8x8 (pCurLayer->pMbType[iMbXy])) {
    RecI8x8Mb (iMbXy, pCtx, pCurLayer->pScaledTCoeff[iMbXy], pCurLayer);
  }

  if (IS_INTRA4x4 (pCurLayer->pMbType[iMbXy]))
    RecI4x4Mb (iMbXy, pCtx, pCurLayer->pScaledTCoeff[iMbXy], pCurLayer);

  return 0;
}

int32_t WelsMbInterPrediction (PWelsDecoderContext pCtx, PDqLayer pCurLayer) {
  int32_t iMbX = pCurLayer->iMbX;
  int32_t iMbY = pCurLayer->iMbY;
  uint8_t*  pDstY, *pDstCb, *pDstCr;

  int32_t iLumaStride   = pCtx->pDec->iLinesize[0];
  int32_t iChromaStride = pCtx->pDec->iLinesize[1];

  pDstY  = pCurLayer->pDec->pData[0] + ((iMbY * iLumaStride + iMbX) << 4);
  pDstCb = pCurLayer->pDec->pData[1] + ((iMbY * iChromaStride + iMbX) << 3);
  pDstCr = pCurLayer->pDec->pData[2] + ((iMbY * iChromaStride + iMbX) << 3);

  GetInterPred (pDstY, pDstCb, pDstCr, pCtx);

  return 0;
}

int32_t WelsTargetMbConstruction (PWelsDecoderContext pCtx) {
  PDqLayer pCurLayer = pCtx->pCurDqLayer;
  if (MB_TYPE_INTRA_PCM == pCurLayer->pMbType[pCurLayer->iMbXyIndex]) {
    //already decoded and reconstructed when parsing
    return 0;
  } else if (IS_INTRA (pCurLayer->pMbType[pCurLayer->iMbXyIndex])) {
    WelsMbIntraPredictionConstruction (pCtx, pCurLayer, 1);
  } else if (IS_INTER (pCurLayer->pMbType[pCurLayer->iMbXyIndex])) { //InterMB
    if (0 == pCurLayer->pCbp[pCurLayer->iMbXyIndex]) { //uiCbp==0 include SKIP
      WelsMbInterPrediction (pCtx, pCurLayer);
    } else {
      WelsMbInterConstruction (pCtx, pCurLayer);
    }
  } else {
    WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING, "WelsTargetMbConstruction():::::Unknown MB type: %d",
             pCurLayer->pMbType[pCurLayer->iMbXyIndex]);
    return -1;
  }

  return 0;
}

void WelsChromaDcIdct (int16_t* pBlock) {
  int32_t iStride = 32;
  int32_t iXStride = 16;
  int32_t iStride1 = iXStride + iStride;
  int16_t* pBlk = pBlock;
  int32_t iA, iB, iC, iD, iE;

  iA = pBlk[0];
  iB = pBlk[iXStride];
  iC = pBlk[iStride];
  iD = pBlk[iStride1];

  iE = iA - iB;
  iA += iB;
  iB = iC - iD;
  iC += iD;

  pBlk[0] = (iA + iC) >> 1;
  pBlk[iXStride] = (iE + iB) >> 1;
  pBlk[iStride] = (iA - iC) >> 1;
  pBlk[iStride1] = (iE - iB) >> 1;
}

void WelsMapNxNNeighToSampleNormal (PWelsNeighAvail pNeighAvail, int32_t* pSampleAvail) {
  if (pNeighAvail->iLeftAvail) {  //left
    pSampleAvail[ 6] =
      pSampleAvail[12] =
        pSampleAvail[18] =
          pSampleAvail[24] = 1;
  }
  if (pNeighAvail->iLeftTopAvail) { //top_left
    pSampleAvail[0] = 1;
  }
  if (pNeighAvail->iTopAvail) { //top
    pSampleAvail[1] =
      pSampleAvail[2] =
        pSampleAvail[3] =
          pSampleAvail[4] = 1;
  }
  if (pNeighAvail->iRightTopAvail) { //top_right
    pSampleAvail[5] = 1;
  }
}

void WelsMapNxNNeighToSampleConstrain1 (PWelsNeighAvail pNeighAvail, int32_t* pSampleAvail) {
  if (pNeighAvail->iLeftAvail && IS_INTRA (pNeighAvail->iLeftType)) {   //left
    pSampleAvail[ 6] =
      pSampleAvail[12] =
        pSampleAvail[18] =
          pSampleAvail[24] = 1;
  }
  if (pNeighAvail->iLeftTopAvail && IS_INTRA (pNeighAvail->iLeftTopType)) {  //top_left
    pSampleAvail[0] = 1;
  }
  if (pNeighAvail->iTopAvail && IS_INTRA (pNeighAvail->iTopType)) {  //top
    pSampleAvail[1] =
      pSampleAvail[2] =
        pSampleAvail[3] =
          pSampleAvail[4] = 1;
  }
  if (pNeighAvail->iRightTopAvail && IS_INTRA (pNeighAvail->iRightTopType)) {  //top_right
    pSampleAvail[5] = 1;
  }
}
void WelsMap16x16NeighToSampleNormal (PWelsNeighAvail pNeighAvail, uint8_t* pSampleAvail) {
  if (pNeighAvail->iLeftAvail) {
    *pSampleAvail = (1 << 2);
  }
  if (pNeighAvail->iLeftTopAvail) {
    *pSampleAvail |= (1 << 1);
  }
  if (pNeighAvail->iTopAvail) {
    *pSampleAvail |= 1;
  }
}

void WelsMap16x16NeighToSampleConstrain1 (PWelsNeighAvail pNeighAvail, uint8_t* pSampleAvail) {
  if (pNeighAvail->iLeftAvail && IS_INTRA (pNeighAvail->iLeftType)) {
    *pSampleAvail = (1 << 2);
  }
  if (pNeighAvail->iLeftTopAvail && IS_INTRA (pNeighAvail->iLeftTopType)) {
    *pSampleAvail |= (1 << 1);
  }
  if (pNeighAvail->iTopAvail && IS_INTRA (pNeighAvail->iTopType)) {
    *pSampleAvail |= 1;
  }
}

int32_t ParseIntra4x4Mode (PWelsDecoderContext pCtx, PWelsNeighAvail pNeighAvail, int8_t* pIntraPredMode,
                           PBitStringAux pBs, PDqLayer pCurDqLayer, DecodedMacroblock *rtd) {
  int32_t iSampleAvail[5 * 6] = { 0 }; //initialize as 0
  int32_t iMbXy = pCurDqLayer->iMbXyIndex;
  int32_t iFinalMode, i;

  uint8_t uiNeighAvail = 0;
  uint32_t uiCode;
  int32_t iCode;
  pCtx->pMapNxNNeighToSampleFunc (pNeighAvail, iSampleAvail);
  uiNeighAvail = (iSampleAvail[6] << 2) | (iSampleAvail[0] << 1) | (iSampleAvail[1]);
  for (i = 0; i < 16; i++) {
    int32_t iPrevIntra4x4PredMode = 0;
    rtd->iRemIntra4x4PredMode[i] = 0;
    if (pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag) {
#ifdef BILLING
      curBillTag = PIP_PRED_MODE_TAG;
#endif
      WELS_READ_VERIFY (ParseIntraPredModeLumaCabac (pCtx, iCode));
      iPrevIntra4x4PredMode = iCode;
    } else {
#ifdef BILLING
      curBillTag = PIP_PRED_MODE_TAG;
#endif
      WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode));
      iPrevIntra4x4PredMode = uiCode;
    }
    const int32_t kiPredMode = PredIntra4x4Mode (pIntraPredMode, i);

    int8_t iBestMode;
    if (pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag) {
      if (iPrevIntra4x4PredMode == -1)
        iBestMode = kiPredMode;
      else
        iBestMode = iPrevIntra4x4PredMode + (iPrevIntra4x4PredMode >= kiPredMode);
    } else {
      if (iPrevIntra4x4PredMode) {
        iBestMode = kiPredMode;
      } else {
#ifdef BILLING
      curBillTag = PIP_PRED_MODE_TAG;
#endif

        WELS_READ_VERIFY (BsGetBits (pBs, 3, &uiCode));
        rtd->iRemIntra4x4PredMode[i] = uiCode;
        iBestMode = uiCode + ((int32_t) uiCode >= kiPredMode);
      }
    }
    rtd->iPrevIntra4x4PredMode[i] = iPrevIntra4x4PredMode;

    iFinalMode = CheckIntraNxNPredMode (&iSampleAvail[0], &iBestMode, i, false);
    if (iFinalMode  == ERR_INVALID_INTRA4X4_MODE) {
      return ERR_INFO_INVALID_I4x4_PRED_MODE;
    }

    pCurDqLayer->pIntra4x4FinalMode[iMbXy][g_kuiScan4[i]] = iFinalMode;

    pIntraPredMode[g_kuiScan8[i]] = iBestMode;

    iSampleAvail[g_kuiCache30ScanIdx[i]] = 1;
  }
  ST32 (&pCurDqLayer->pIntraPredMode[iMbXy][0], LD32 (&pIntraPredMode[1 + 8 * 4]));
  pCurDqLayer->pIntraPredMode[iMbXy][4] = pIntraPredMode[4 + 8 * 1];
  pCurDqLayer->pIntraPredMode[iMbXy][5] = pIntraPredMode[4 + 8 * 2];
  pCurDqLayer->pIntraPredMode[iMbXy][6] = pIntraPredMode[4 + 8 * 3];

  if (pCtx->pSps->uiChromaFormatIdc == 0)//no need parse chroma
    return ERR_NONE;

  if (pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag) {
    WELS_READ_VERIFY (ParseIntraPredModeChromaCabac (pCtx, uiNeighAvail, iCode));
    if (iCode > MAX_PRED_MODE_ID_CHROMA) {
      return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
    }
    pCurDqLayer->pChromaPredMode[iMbXy] = iCode;
  } else {
#ifdef BILLING
      curBillTag = PIP_PRED_MODE_TAG;
#endif

    WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //intra_chroma_pred_mode
    if (uiCode > MAX_PRED_MODE_ID_CHROMA) {
      return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
    }
    pCurDqLayer->pChromaPredMode[iMbXy] = uiCode;
  }

  if (-1 == pCurDqLayer->pChromaPredMode[iMbXy]
      || CheckIntraChromaPredMode (uiNeighAvail, &pCurDqLayer->pChromaPredMode[iMbXy])) {
    return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
  }
  return ERR_NONE;
}

int32_t ParseIntra8x8Mode (PWelsDecoderContext pCtx, PWelsNeighAvail pNeighAvail, int8_t* pIntraPredMode,
                           PBitStringAux pBs, PDqLayer pCurDqLayer, DecodedMacroblock *rtd) {
  // Similar with Intra_4x4, can put them together when needed
  int32_t iSampleAvail[5 * 6] = { 0 }; //initialize as 0
  int32_t iMbXy = pCurDqLayer->iMbXyIndex;
  int32_t iFinalMode, i;

  uint8_t uiNeighAvail = 0;
  uint32_t uiCode;
  int32_t iCode;
  pCtx->pMapNxNNeighToSampleFunc (pNeighAvail, iSampleAvail);
  // Top-Right : Left : Top-Left : Top
  uiNeighAvail = (iSampleAvail[5] << 3) | (iSampleAvail[6] << 2) | (iSampleAvail[0] << 1) | (iSampleAvail[1]);

  pCurDqLayer->pIntraNxNAvailFlag[iMbXy] = uiNeighAvail;

  for (i = 0; i < 4; i++) {
    int32_t iPrevIntra4x4PredMode = 0;
    rtd->iRemIntra4x4PredMode[i] = 0;
    if (pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag) {
      WELS_READ_VERIFY (ParseIntraPredModeLumaCabac (pCtx, iCode));
      iPrevIntra4x4PredMode = iCode;
    } else {
#ifdef BILLING
      curBillTag = PIP_PRED_MODE_TAG;
#endif
      WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode));
      iPrevIntra4x4PredMode = uiCode;
    }
    const int32_t kiPredMode = PredIntra4x4Mode (pIntraPredMode, i << 2);

    int8_t iBestMode;
    if (pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag) {
      if (iPrevIntra4x4PredMode == -1)
        iBestMode = kiPredMode;
      else
        iBestMode = iPrevIntra4x4PredMode + (iPrevIntra4x4PredMode >= kiPredMode);
    } else {
      if (iPrevIntra4x4PredMode) {
        iBestMode = kiPredMode;
      } else {
#ifdef BILLING
      curBillTag = PIP_PRED_MODE_TAG;
#endif
        WELS_READ_VERIFY (BsGetBits (pBs, 3, &uiCode));
        rtd->iRemIntra4x4PredMode[i] = uiCode;
        iBestMode = uiCode + ((int32_t) uiCode >= kiPredMode);
      }
    }
    rtd->iPrevIntra4x4PredMode[i] = iPrevIntra4x4PredMode;

    iFinalMode = CheckIntraNxNPredMode (&iSampleAvail[0], &iBestMode, i << 2, true);

    if (iFinalMode  == ERR_INVALID_INTRA4X4_MODE) {
      return ERR_INFO_INVALID_I4x4_PRED_MODE;
    }

    for (int j = 0; j < 4; j++) {
      pCurDqLayer->pIntra4x4FinalMode[iMbXy][g_kuiScan4[ (i << 2) + j]] = iFinalMode;
      pIntraPredMode[g_kuiScan8[ (i << 2) + j]] = iBestMode;
      iSampleAvail[g_kuiCache30ScanIdx[ (i << 2) + j]] = 1;
    }
  }
  ST32 (&pCurDqLayer->pIntraPredMode[iMbXy][0], LD32 (&pIntraPredMode[1 + 8 * 4]));
  pCurDqLayer->pIntraPredMode[iMbXy][4] = pIntraPredMode[4 + 8 * 1];
  pCurDqLayer->pIntraPredMode[iMbXy][5] = pIntraPredMode[4 + 8 * 2];
  pCurDqLayer->pIntraPredMode[iMbXy][6] = pIntraPredMode[4 + 8 * 3];
  if (pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag) {
    WELS_READ_VERIFY (ParseIntraPredModeChromaCabac (pCtx, uiNeighAvail, iCode));
    if (iCode > MAX_PRED_MODE_ID_CHROMA) {
      return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
    }
    pCurDqLayer->pChromaPredMode[iMbXy] = iCode;
  } else {
#ifdef BILLING
      curBillTag = PIP_PRED_MODE_TAG;
#endif
    WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //intra_chroma_pred_mode
    if (uiCode > MAX_PRED_MODE_ID_CHROMA) {
      return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
    }
    pCurDqLayer->pChromaPredMode[iMbXy] = uiCode;
  }

  if (-1 == pCurDqLayer->pChromaPredMode[iMbXy]
      || CheckIntraChromaPredMode (uiNeighAvail, &pCurDqLayer->pChromaPredMode[iMbXy])) {
    return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
  }

  return ERR_NONE;
}

int32_t ParseIntra16x16Mode (PWelsDecoderContext pCtx, PWelsNeighAvail pNeighAvail, PBitStringAux pBs,
                             PDqLayer pCurDqLayer) {
  int32_t iMbXy = pCurDqLayer->iMbXyIndex;
  uint8_t uiNeighAvail = 0; //0x07 = 0 1 1 1, means left, top-left, top avail or not. (1: avail, 0: unavail)
  uint32_t uiCode;
  int32_t iCode;
  pCtx->pMap16x16NeighToSampleFunc (pNeighAvail, &uiNeighAvail);

  if (CheckIntra16x16PredMode (uiNeighAvail,
                               &pCurDqLayer->pIntraPredMode[iMbXy][7])) { //invalid iPredMode, must stop decoding
    return ERR_INFO_INVALID_I16x16_PRED_MODE;
  }
  if (pCtx->pSps->uiChromaFormatIdc == 0)
    return ERR_NONE;
 // Only for intra4x4 // Only for intra4xe
  if (pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag) {
    WELS_READ_VERIFY (ParseIntraPredModeChromaCabac (pCtx, uiNeighAvail, iCode));
    if (iCode > MAX_PRED_MODE_ID_CHROMA) {
      return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
    }
    pCurDqLayer->pChromaPredMode[iMbXy] = iCode;
  } else {
#ifdef BILLING
      curBillTag = PIP_PRED_MODE_TAG;
#endif
    WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //intra_chroma_pred_mode
    if (uiCode > MAX_PRED_MODE_ID_CHROMA) {
      return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
    }
    pCurDqLayer->pChromaPredMode[iMbXy] = uiCode;
  }
  if (-1 == pCurDqLayer->pChromaPredMode[iMbXy]
      || CheckIntraChromaPredMode (uiNeighAvail, &pCurDqLayer->pChromaPredMode[iMbXy])) {
    return ERR_INFO_INVALID_I_CHROMA_PRED_MODE;
  }

  return ERR_NONE;
}

int32_t WelsDecodeMbCabacISliceBaseMode0 (PWelsDecoderContext pCtx, uint32_t& uiEosFlag, DecodedMacroblock*rtd) {
  PDqLayer pCurLayer             = pCtx->pCurDqLayer;
  PBitStringAux pBsAux           = pCurLayer->pBitStringAux;
  PSlice pSlice                  = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeader pSliceHeader      = &pSlice->sSliceHeaderExt.sSliceHeader;
  SWelsNeighAvail sNeighAvail;
  int32_t iScanIdxStart = pSlice->sSliceHeaderExt.uiScanIdxStart;
  int32_t iScanIdxEnd   = pSlice->sSliceHeaderExt.uiScanIdxEnd;
  int32_t iMbXy = pCurLayer->iMbXyIndex;
  int32_t i;
  uint32_t uiMbType = 0, uiCbp = 0, uiCbpLuma = 0, uiCbpChroma = 0;

  ENFORCE_STACK_ALIGN_1D (uint8_t, pNonZeroCount, 48, 16);
  { // scope for copy on exit

  pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
  pCurLayer->pTransformSize8x8Flag[iMbXy] = false;

  pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;
  pCurLayer->pResidualPredFlag[iMbXy] = pSlice->sSliceHeaderExt.bDefaultResidualPredFlag;
  GetNeighborAvailMbType (&sNeighAvail, pCurLayer);
  WELS_READ_VERIFY (ParseMBTypeISliceCabac (pCtx, &sNeighAvail, uiMbType));
  if (uiMbType > 25) {
    return ERR_INFO_INVALID_MB_TYPE;
  } else if (!pCtx->pSps->uiChromaFormatIdc && ((uiMbType >= 5 && uiMbType <= 12) || (uiMbType >= 17
             && uiMbType <= 24))) {
    return ERR_INFO_INVALID_MB_TYPE;
  } else if (25 == uiMbType) {   //I_PCM
    WELS_READ_VERIFY (ParseIPCMInfoCabac (pCtx));
    pSlice->iLastDeltaQp = 0;
    WELS_READ_VERIFY (ParseEndOfSliceCabac (pCtx, uiEosFlag));
    if (uiEosFlag) {
      RestoreCabacDecEngineToBS (pCtx->pCabacDecEngine, pCtx->pCurDqLayer->pBitStringAux);
    }
    return ERR_NONE;
  } else if (0 == uiMbType) { //I4x4
    ENFORCE_STACK_ALIGN_1D (int8_t, pIntraPredMode, 48, 16);
    pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA4x4;
    if (pCtx->pPps->bTransform8x8ModeFlag) {
      // Transform 8x8 cabac will be added soon
      WELS_READ_VERIFY (ParseTransformSize8x8FlagCabac (pCtx, &sNeighAvail, pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]));
    }
    if (pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]) {
      uiMbType = pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA8x8;
      pCtx->pFillInfoCacheIntraNxNFunc (&sNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
      WELS_READ_VERIFY (ParseIntra8x8Mode (pCtx, &sNeighAvail, pIntraPredMode, pBsAux, pCurLayer, rtd));
    } else {
      pCtx->pFillInfoCacheIntraNxNFunc (&sNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
      WELS_READ_VERIFY (ParseIntra4x4Mode (pCtx, &sNeighAvail, pIntraPredMode, pBsAux, pCurLayer, rtd));
    }
    //get uiCbp for I4x4
    WELS_READ_VERIFY (ParseCbpInfoCabac (pCtx, &sNeighAvail, uiCbp));
    pCurLayer->pCbp[iMbXy] = uiCbp;
    pSlice->iLastDeltaQp = uiCbp == 0 ? 0 : pSlice->iLastDeltaQp;
    uiCbpChroma = pCtx->pSps->uiChromaFormatIdc ? uiCbp >> 4 : 0;
    uiCbpLuma = uiCbp & 15;
  } else { //I16x16;
    pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA16x16;
    pCurLayer->pTransformSize8x8Flag[iMbXy] = false;
    pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
    pCurLayer->pIntraPredMode[iMbXy][7] = (uiMbType - 1) & 3;
    pCurLayer->pCbp[iMbXy] = g_kuiI16CbpTable[ (uiMbType - 1) >> 2];
    uiCbpChroma = pCtx->pSps->uiChromaFormatIdc ? pCurLayer->pCbp[iMbXy] >> 4 : 0 ;
    uiCbpLuma = pCurLayer->pCbp[iMbXy] & 15;
    WelsFillCacheNonZeroCount (&sNeighAvail, pNonZeroCount, pCurLayer);
    WELS_READ_VERIFY (ParseIntra16x16Mode (pCtx, &sNeighAvail, pBsAux, pCurLayer));
  }

  ST32 (&pCurLayer->pNzc[iMbXy][0], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][4], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][8], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][12], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][16], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][20], 0);
  pCurLayer->pCbfDc[iMbXy] = 0;

  if (pCurLayer->pCbp[iMbXy] == 0 && IS_INTRANxN (pCurLayer->pMbType[iMbXy])) {
    pCurLayer->pLumaQp[iMbXy] = pSlice->iLastMbQp;
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 ((pCurLayer->pLumaQp[iMbXy] +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i]), 0, 51)];
    }
  }

  if (pCurLayer->pCbp[iMbXy] || MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
    memset (pCurLayer->pScaledTCoeff[iMbXy], 0, 384 * sizeof (pCurLayer->pScaledTCoeff[iMbXy][0]));
    memset (pCurLayer->pScaledTCoeffQuant[iMbXy], 0, 384 * sizeof (pCurLayer->pScaledTCoeffQuant[iMbXy][0]));
    int32_t iQpDelta, iId8x8, iId4x4;
    WELS_READ_VERIFY (ParseDeltaQpCabac (pCtx, iQpDelta));
    if (iQpDelta > 25 || iQpDelta < -26) {//out of iQpDelta range
      return ERR_INFO_INVALID_QP;
    }
    pCurLayer->pLumaQp[iMbXy] = (pSlice->iLastMbQp + iQpDelta + 52) % 52; //update last_mb_qp
    pSlice->iLastMbQp = pCurLayer->pLumaQp[iMbXy];
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 ((pSlice->iLastMbQp +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i]), 0, 51)];
    }
    if (MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
      //step1: Luma DC
      WELS_READ_VERIFY (ParseResidualBlockCabac (&sNeighAvail, pNonZeroCount, pBsAux, 0, 16, g_kuiLumaDcZigzagScan,
                        I16_LUMA_DC, pCurLayer->pScaledTCoeff[iMbXy], pCurLayer->pLumaQp[iMbXy], pCtx));
      //step2: Luma AC
      if (uiCbpLuma) {
        for (i = 0; i < 16; i++) {
          WELS_READ_VERIFY (ParseResidualBlockCabac (&sNeighAvail, pNonZeroCount, pBsAux, i,
                            iScanIdxEnd - WELS_MAX (iScanIdxStart, 1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1), I16_LUMA_AC,
                            pCurLayer->pScaledTCoeff[iMbXy] + (i << 4), pCurLayer->pLumaQp[iMbXy], pCtx));
        }
        ST32 (&pCurLayer->pNzc[iMbXy][0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32 (&pCurLayer->pNzc[iMbXy][4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32 (&pCurLayer->pNzc[iMbXy][8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32 (&pCurLayer->pNzc[iMbXy][12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      } else { //pNonZeroCount = 0
        ST32 (&pCurLayer->pNzc[iMbXy][0], 0);
        ST32 (&pCurLayer->pNzc[iMbXy][4], 0);
        ST32 (&pCurLayer->pNzc[iMbXy][8], 0);
        ST32 (&pCurLayer->pNzc[iMbXy][12], 0);
      }
    } else { //non-MB_TYPE_INTRA16x16
      if (pCurLayer->pTransformSize8x8Flag[iMbXy]) {
        // Transform 8x8 support for CABAC
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          if (uiCbpLuma & (1 << iId8x8)) {
            WELS_READ_VERIFY (ParseResidualBlockCabac8x8 (&sNeighAvail, pNonZeroCount, pBsAux, (iId8x8 << 2),
                              iScanIdxEnd - iScanIdxStart + 1, g_kuiZigzagScan8x8 + iScanIdxStart, LUMA_DC_AC_INTRA_8,
                              pCurLayer->pScaledTCoeff[iMbXy] + (iId8x8 << 6), pCurLayer->pLumaQp[iMbXy], pCtx));
          } else {
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[ (iId8x8 << 2)]], 0);
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32 (&pCurLayer->pNzc[iMbXy][0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32 (&pCurLayer->pNzc[iMbXy][4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32 (&pCurLayer->pNzc[iMbXy][8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32 (&pCurLayer->pNzc[iMbXy][12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      } else {
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          if (uiCbpLuma & (1 << iId8x8)) {
            int32_t iIdx = (iId8x8 << 2);
            for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
              //Luma (DC and AC decoding together)
              WELS_READ_VERIFY (ParseResidualBlockCabac (&sNeighAvail, pNonZeroCount, pBsAux, iIdx, iScanIdxEnd - iScanIdxStart + 1,
                                g_kuiZigzagScan + iScanIdxStart, LUMA_DC_AC_INTRA, pCurLayer->pScaledTCoeff[iMbXy] + (iIdx << 4),
                                pCurLayer->pLumaQp[iMbXy], pCtx));
              iIdx++;
            }
          } else {
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[ (iId8x8 << 2)]], 0);
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32 (&pCurLayer->pNzc[iMbXy][0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32 (&pCurLayer->pNzc[iMbXy][4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32 (&pCurLayer->pNzc[iMbXy][8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32 (&pCurLayer->pNzc[iMbXy][12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      }
    }
    int32_t iMbResProperty;
    //chroma
    //step1: DC
    if (1 == uiCbpChroma || 2 == uiCbpChroma) {
      //Cb Cr
      for (i = 0; i < 2; i++) {
        iMbResProperty = i ? CHROMA_DC_V : CHROMA_DC_U;
        WELS_READ_VERIFY (ParseResidualBlockCabac (&sNeighAvail, pNonZeroCount, pBsAux, 16 + (i << 2), 4, g_kuiChromaDcScan,
                          iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + 256 + (i << 6), pCurLayer->pChromaQp[iMbXy][i], pCtx));
      }
    }

    //step2: AC
    if (2 == uiCbpChroma) {
      for (i = 0; i < 2; i++) { //Cb Cr
        iMbResProperty = i ? CHROMA_AC_V : CHROMA_AC_U;
        int32_t iIdx = 16 + (i << 2);
        for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
          WELS_READ_VERIFY (ParseResidualBlockCabac (&sNeighAvail, pNonZeroCount, pBsAux, iIdx,
                            iScanIdxEnd - WELS_MAX (iScanIdxStart, 1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1), iMbResProperty,
                            pCurLayer->pScaledTCoeff[iMbXy] + (iIdx << 4), pCurLayer->pChromaQp[iMbXy][i], pCtx));
          iIdx++;
        }
      }
      ST16 (&pCurLayer->pNzc[iMbXy][16], LD16 (&pNonZeroCount[6 + 8 * 1]));
      ST16 (&pCurLayer->pNzc[iMbXy][20], LD16 (&pNonZeroCount[6 + 8 * 2]));
      ST16 (&pCurLayer->pNzc[iMbXy][18], LD16 (&pNonZeroCount[6 + 8 * 4]));
      ST16 (&pCurLayer->pNzc[iMbXy][22], LD16 (&pNonZeroCount[6 + 8 * 5]));
    } else {
      ST16 (&pCurLayer->pNzc[iMbXy][16], 0);
      ST16 (&pCurLayer->pNzc[iMbXy][20], 0);
      ST16 (&pCurLayer->pNzc[iMbXy][18], 0);
      ST16 (&pCurLayer->pNzc[iMbXy][22], 0);
    }
  } else {
    ST32 (&pCurLayer->pNzc[iMbXy][0], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][4], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][8], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][12], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][16], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][20], 0);
  }
  WELS_READ_VERIFY (ParseEndOfSliceCabac (pCtx, uiEosFlag));
  }
  if (uiEosFlag) {
    RestoreCabacDecEngineToBS (pCtx->pCabacDecEngine, pCtx->pCurDqLayer->pBitStringAux);
  }
  return ERR_NONE;
}

int32_t WelsDecodeMbCabacISlice (PWelsDecoderContext pCtx, PNalUnit pNalCur, uint32_t& uiEosFlag, DecodedMacroblock *rtd) {
  WELS_READ_VERIFY (WelsDecodeMbCabacISliceBaseMode0 (pCtx, uiEosFlag, rtd));
  return ERR_NONE;
}

int32_t WelsDecodeMbCabacPSliceBaseMode0 (PWelsDecoderContext pCtx, PWelsNeighAvail pNeighAvail, uint32_t& uiEosFlag, DecodedMacroblock *rtd) {
  PDqLayer pCurLayer             = pCtx->pCurDqLayer;
  PBitStringAux pBsAux           = pCurLayer->pBitStringAux;
  PSlice pSlice                  = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeader pSliceHeader      = &pSlice->sSliceHeaderExt.sSliceHeader;

  int32_t iScanIdxStart = pSlice->sSliceHeaderExt.uiScanIdxStart;
  int32_t iScanIdxEnd   = pSlice->sSliceHeaderExt.uiScanIdxEnd;
  int32_t iMbXy = pCurLayer->iMbXyIndex;
  int32_t iMbResProperty;
  int32_t i;
  uint32_t uiMbType = 0, uiCbp = 0, uiCbpLuma = 0, uiCbpChroma = 0;

  ENFORCE_STACK_ALIGN_1D (uint8_t, pNonZeroCount, 48, 16);
  {
  pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;

  WELS_READ_VERIFY (ParseMBTypePSliceCabac (pCtx, pNeighAvail, uiMbType));
  // uiMbType = 4 is not allowded.
  if (uiMbType < 4) { //Inter mode
    int16_t pMotionVector[LIST_A][30][MV_A];
    int16_t pMvdCache[LIST_A][30][MV_A];
    int8_t  pRefIndex[LIST_A][30];
    pCurLayer->pMbType[iMbXy] = g_ksInterMbTypeInfo[uiMbType].iType;
    WelsFillCacheInterCabac (pNeighAvail, pNonZeroCount, pMotionVector, pMvdCache, pRefIndex, pCurLayer);
    WELS_READ_VERIFY (ParseInterMotionInfoCabac (pCtx, pNeighAvail, pNonZeroCount, pMotionVector, pMvdCache, pRefIndex));
    pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;
  } else { //Intra mode
    uiMbType -= 5;
    if (uiMbType > 25)
      return ERR_INFO_INVALID_MB_TYPE;
    if (!pCtx->pSps->uiChromaFormatIdc && ((uiMbType >= 5 && uiMbType <= 12) || (uiMbType >= 17 && uiMbType <= 24)))
      return ERR_INFO_INVALID_MB_TYPE;

    if (25 == uiMbType) {   //I_PCM
      WELS_READ_VERIFY (ParseIPCMInfoCabac (pCtx));
      pSlice->iLastDeltaQp = 0;
      WELS_READ_VERIFY (ParseEndOfSliceCabac (pCtx, uiEosFlag));
      if (uiEosFlag) {
        RestoreCabacDecEngineToBS (pCtx->pCabacDecEngine, pCtx->pCurDqLayer->pBitStringAux);
      }
      return ERR_NONE;
    } else { //normal Intra mode
      if (0 == uiMbType) { //Intra4x4
        ENFORCE_STACK_ALIGN_1D (int8_t, pIntraPredMode, 48, 16);
        pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA4x4;
        if (pCtx->pPps->bTransform8x8ModeFlag) {
          WELS_READ_VERIFY (ParseTransformSize8x8FlagCabac (pCtx, pNeighAvail, pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]));
        }
        if (pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]) {
          uiMbType = pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA8x8;
          pCtx->pFillInfoCacheIntraNxNFunc (pNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
          WELS_READ_VERIFY (ParseIntra8x8Mode (pCtx, pNeighAvail, pIntraPredMode, pBsAux, pCurLayer, rtd));
        } else {
          pCtx->pFillInfoCacheIntraNxNFunc (pNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
          WELS_READ_VERIFY (ParseIntra4x4Mode (pCtx, pNeighAvail, pIntraPredMode, pBsAux, pCurLayer, rtd));
        }
      } else { //Intra16x16
        pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA16x16;
        pCurLayer->pTransformSize8x8Flag[iMbXy] = false;
        pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
        pCurLayer->pIntraPredMode[iMbXy][7] = (uiMbType - 1) & 3;
        pCurLayer->pCbp[iMbXy] = g_kuiI16CbpTable[ (uiMbType - 1) >> 2];
        uiCbpChroma = pCtx->pSps->uiChromaFormatIdc ? pCurLayer->pCbp[iMbXy] >> 4 : 0;
        uiCbpLuma = pCurLayer->pCbp[iMbXy] & 15;
        WelsFillCacheNonZeroCount (pNeighAvail, pNonZeroCount, pCurLayer);
        WELS_READ_VERIFY (ParseIntra16x16Mode (pCtx, pNeighAvail, pBsAux, pCurLayer));
      }
    }
  }

  ST32 (&pCurLayer->pNzc[iMbXy][0], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][4], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][8], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][12], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][16], 0);
  ST32 (&pCurLayer->pNzc[iMbXy][20], 0);

  if (MB_TYPE_INTRA16x16 != pCurLayer->pMbType[iMbXy]) {
    WELS_READ_VERIFY (ParseCbpInfoCabac (pCtx, pNeighAvail, uiCbp));

    pCurLayer->pCbp[iMbXy] = uiCbp;
    pSlice->iLastDeltaQp = uiCbp == 0 ? 0 : pSlice->iLastDeltaQp;
    uiCbpChroma = pCtx->pSps->uiChromaFormatIdc ? pCurLayer->pCbp[iMbXy] >> 4 : 0 ;
    uiCbpLuma = pCurLayer->pCbp[iMbXy] & 15;
  }

  if (pCurLayer->pCbp[iMbXy] || MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {

    if (MB_TYPE_INTRA16x16 != pCurLayer->pMbType[iMbXy]) {
      // Need modification when B picutre add in
      bool bNeedParseTransformSize8x8Flag =
        (((pCurLayer->pMbType[iMbXy] >= MB_TYPE_16x16 && pCurLayer->pMbType[iMbXy] <= MB_TYPE_8x16)
          || pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy])
         && (pCurLayer->pMbType[iMbXy] != MB_TYPE_INTRA8x8)
         && (pCurLayer->pMbType[iMbXy] != MB_TYPE_INTRA4x4)
         && ((pCurLayer->pCbp[iMbXy] & 0x0F) > 0)
         && (pCtx->pPps->bTransform8x8ModeFlag));

      if (bNeedParseTransformSize8x8Flag) {
        WELS_READ_VERIFY (ParseTransformSize8x8FlagCabac (pCtx, pNeighAvail,
                          pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy])); //transform_size_8x8_flag
      }
    }

    memset (pCurLayer->pScaledTCoeff[iMbXy], 0, 384 * sizeof (pCurLayer->pScaledTCoeff[iMbXy][0]));

    int32_t iQpDelta, iId8x8, iId4x4;

    WELS_READ_VERIFY (ParseDeltaQpCabac (pCtx, iQpDelta));
    if (iQpDelta > 25 || iQpDelta < -26) { //out of iQpDelta range
      return ERR_INFO_INVALID_QP;
    }
    pCurLayer->pLumaQp[iMbXy] = (pSlice->iLastMbQp + iQpDelta + 52) % 52; //update last_mb_qp
    pSlice->iLastMbQp = pCurLayer->pLumaQp[iMbXy];
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pSlice->iLastMbQp +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i], 0, 51)];
    }

    if (MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
      //step1: Luma DC
      WELS_READ_VERIFY (ParseResidualBlockCabac (pNeighAvail, pNonZeroCount, pBsAux, 0, 16, g_kuiLumaDcZigzagScan,
                        I16_LUMA_DC, pCurLayer->pScaledTCoeff[iMbXy], pCurLayer->pLumaQp[iMbXy], pCtx));
      //step2: Luma AC
      if (uiCbpLuma) {
        for (i = 0; i < 16; i++) {
          WELS_READ_VERIFY (ParseResidualBlockCabac (pNeighAvail, pNonZeroCount, pBsAux, i, iScanIdxEnd - WELS_MAX (iScanIdxStart,
                            1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1), I16_LUMA_AC, pCurLayer->pScaledTCoeff[iMbXy] + (i << 4),
                            pCurLayer->pLumaQp[iMbXy], pCtx));
        }
        ST32 (&pCurLayer->pNzc[iMbXy][0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32 (&pCurLayer->pNzc[iMbXy][4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32 (&pCurLayer->pNzc[iMbXy][8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32 (&pCurLayer->pNzc[iMbXy][12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      } else {
        ST32 (&pCurLayer->pNzc[iMbXy][0], 0);
        ST32 (&pCurLayer->pNzc[iMbXy][4], 0);
        ST32 (&pCurLayer->pNzc[iMbXy][8], 0);
        ST32 (&pCurLayer->pNzc[iMbXy][12], 0);
      }
    } else { //non-MB_TYPE_INTRA16x16
      if (pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]) {
        // Transform 8x8 support for CABAC
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          if (uiCbpLuma & (1 << iId8x8)) {
            WELS_READ_VERIFY (ParseResidualBlockCabac8x8 (pNeighAvail, pNonZeroCount, pBsAux, (iId8x8 << 2),
                              iScanIdxEnd - iScanIdxStart + 1, g_kuiZigzagScan8x8 + iScanIdxStart,
                              IS_INTRA (pCurLayer->pMbType[iMbXy]) ? LUMA_DC_AC_INTRA_8 : LUMA_DC_AC_INTER_8,
                              pCurLayer->pScaledTCoeff[iMbXy] + (iId8x8 << 6), pCurLayer->pLumaQp[iMbXy], pCtx));
          } else {
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[ (iId8x8 << 2)]], 0);
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32 (&pCurLayer->pNzc[iMbXy][0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32 (&pCurLayer->pNzc[iMbXy][4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32 (&pCurLayer->pNzc[iMbXy][8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32 (&pCurLayer->pNzc[iMbXy][12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      } else {
        iMbResProperty = (IS_INTRA (pCurLayer->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA : LUMA_DC_AC_INTER;
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          if (uiCbpLuma & (1 << iId8x8)) {
            int32_t iIdx = (iId8x8 << 2);
            for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
              //Luma (DC and AC decoding together)
              WELS_READ_VERIFY (ParseResidualBlockCabac (pNeighAvail, pNonZeroCount, pBsAux, iIdx, iScanIdxEnd - iScanIdxStart + 1,
                                g_kuiZigzagScan + iScanIdxStart, iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + (iIdx << 4),
                                pCurLayer->pLumaQp[iMbXy],
                                pCtx));
              iIdx++;
            }
          } else {
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[iId8x8 << 2]], 0);
            ST16 (&pNonZeroCount[g_kCacheNzcScanIdx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32 (&pCurLayer->pNzc[iMbXy][0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32 (&pCurLayer->pNzc[iMbXy][4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32 (&pCurLayer->pNzc[iMbXy][8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32 (&pCurLayer->pNzc[iMbXy][12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      }
    }

    //chroma
    //step1: DC
    if (1 == uiCbpChroma || 2 == uiCbpChroma) {
      for (i = 0; i < 2; i++) {
        if (IS_INTRA (pCurLayer->pMbType[iMbXy]))
          iMbResProperty = i ? CHROMA_DC_V : CHROMA_DC_U;
        else
          iMbResProperty = i ? CHROMA_DC_V_INTER : CHROMA_DC_U_INTER;

        WELS_READ_VERIFY (ParseResidualBlockCabac (pNeighAvail, pNonZeroCount, pBsAux, 16 + (i << 2), 4, g_kuiChromaDcScan,
                          iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + 256 + (i << 6), pCurLayer->pChromaQp[iMbXy][i], pCtx));
      }
    }
    //step2: AC
    if (2 == uiCbpChroma) {
      for (i = 0; i < 2; i++) {
        if (IS_INTRA (pCurLayer->pMbType[iMbXy]))
          iMbResProperty = i ? CHROMA_AC_V : CHROMA_AC_U;
        else
          iMbResProperty = i ? CHROMA_AC_V_INTER : CHROMA_AC_U_INTER;
        int32_t index = 16 + (i << 2);
        for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
          WELS_READ_VERIFY (ParseResidualBlockCabac (pNeighAvail, pNonZeroCount, pBsAux, index,
                            iScanIdxEnd - WELS_MAX (iScanIdxStart, 1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1),
                            iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + (index << 4), pCurLayer->pChromaQp[iMbXy][i], pCtx));
          index++;
        }
      }
      ST16 (&pCurLayer->pNzc[iMbXy][16], LD16 (&pNonZeroCount[6 + 8 * 1]));
      ST16 (&pCurLayer->pNzc[iMbXy][20], LD16 (&pNonZeroCount[6 + 8 * 2]));
      ST16 (&pCurLayer->pNzc[iMbXy][18], LD16 (&pNonZeroCount[6 + 8 * 4]));
      ST16 (&pCurLayer->pNzc[iMbXy][22], LD16 (&pNonZeroCount[6 + 8 * 5]));
    } else {
      ST32 (&pCurLayer->pNzc[iMbXy][16], 0);
      ST32 (&pCurLayer->pNzc[iMbXy][20], 0);
    }
  } else {
    pCurLayer->pLumaQp[iMbXy] = pSlice->iLastMbQp;
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pCurLayer->pLumaQp[iMbXy] +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i], 0, 51)];
    }
  }

  WELS_READ_VERIFY (ParseEndOfSliceCabac (pCtx, uiEosFlag));
  if (uiEosFlag) {
    RestoreCabacDecEngineToBS (pCtx->pCabacDecEngine, pCtx->pCurDqLayer->pBitStringAux);
  }
  }
  return ERR_NONE;
}

int32_t WelsDecodeMbCabacPSlice (PWelsDecoderContext pCtx, PNalUnit pNalCur, uint32_t& uiEosFlag, DecodedMacroblock *rtd) {
  PDqLayer pCurLayer             = pCtx->pCurDqLayer;
  PSlice pSlice                  = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeader pSliceHeader      = &pSlice->sSliceHeaderExt.sSliceHeader;
  PPicture* ppRefPic = pCtx->sRefPic.pRefList[LIST_0];
  uint32_t uiCode;
  int32_t iMbXy = pCurLayer->iMbXyIndex;
  int32_t i;
  SWelsNeighAvail uiNeighAvail;
  pCurLayer->pCbp[iMbXy] = 0;
  pCurLayer->pCbfDc[iMbXy] = 0;
  pCurLayer->pChromaPredMode[iMbXy] = C_PRED_DC;

  pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
  pCurLayer->pTransformSize8x8Flag[iMbXy] = false;

  GetNeighborAvailMbType (&uiNeighAvail, pCurLayer);
  WELS_READ_VERIFY (ParseSkipFlagCabac (pCtx, &uiNeighAvail, uiCode));

  if (uiCode) {
    int16_t pMv[2] = {0};
    pCurLayer->pMbType[iMbXy] = MB_TYPE_SKIP;
    ST32 (&pCurLayer->pNzc[iMbXy][0], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][4], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][8], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][12], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][16], 0);
    ST32 (&pCurLayer->pNzc[iMbXy][20], 0);

    pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;
    memset (pCurLayer->pRefIndex[0][iMbXy], 0, sizeof (int8_t) * 16);
    pCtx->bMbRefConcealed = pCtx->bRPLRError || pCtx->bMbRefConcealed || ! (ppRefPic[0] && ppRefPic[0]->bIsComplete);
    //predict mv
    PredPSkipMvFromNeighbor (pCurLayer, pMv);
    for (i = 0; i < 16; i++) {
      ST32 (pCurLayer->pMv[0][iMbXy][i], * (uint32_t*)pMv);
      ST32 (pCurLayer->pMvd[0][iMbXy][i], 0);
    }

    //if (!pSlice->sSliceHeaderExt.bDefaultResidualPredFlag) {
    //  memset (pCurLayer->pScaledTCoeff[iMbXy], 0, 384 * sizeof (int16_t));
    //}

    //reset rS
    pCurLayer->pLumaQp[iMbXy] = pSlice->iLastMbQp; //??????????????? dqaunt of previous mb
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pCurLayer->pLumaQp[iMbXy] +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i], 0, 51)];
    }

    //for neighboring CABAC usage
    pSlice->iLastDeltaQp = 0;

    WELS_READ_VERIFY (ParseEndOfSliceCabac (pCtx, uiEosFlag));

    return ERR_NONE;
  }

  WELS_READ_VERIFY (WelsDecodeMbCabacPSliceBaseMode0 (pCtx, &uiNeighAvail, uiEosFlag, rtd));
  return ERR_NONE;
}
// Calculate deqaunt coeff scaling list value
int32_t  WelsCalcDeqCoeffScalingList (PWelsDecoderContext pCtx) {
  if (pCtx->pSps->bSeqScalingMatrixPresentFlag || pCtx->pPps->bPicScalingMatrixPresentFlag) {
    pCtx->bUseScalingList = true;

    if (!pCtx->bDequantCoeff4x4Init || (pCtx->iDequantCoeffPpsid != pCtx->pPps->iPpsId)) {
      int i, q, x, y;
      // Rewrite pps scaling list for scalingList present flag=0
      if (pCtx->bSpsLatePps) {
        for (i = 0; i < 12; i++) {
          //if (!pCtx->pSps->bSeqScalingListPresentFlag[i]) {
          if (!pCtx->pPps->bPicScalingListPresentFlag[i]) {
            if (i < 6) {
              if (i == 0 || i == 3)
                memcpy (pCtx->pPps->iScalingList4x4[i], pCtx->pSps->iScalingList4x4[i], 16 * sizeof (uint8_t));
              else
                memcpy (pCtx->pPps->iScalingList4x4[i], pCtx->pPps->iScalingList4x4[i - 1], 16 * sizeof (uint8_t));
            } else {
              if (i == 6 || i == 7)
                memcpy (pCtx->pPps->iScalingList8x8[ i - 6 ], pCtx->pSps->iScalingList8x8[ i - 6 ], 64 * sizeof (uint8_t));
              else
                memcpy (pCtx->pPps->iScalingList8x8[ i - 6 ], pCtx->pPps->iScalingList8x8[i - 8], 64 * sizeof (uint8_t));
            }
          }
        }
      }
      //Init dequant coeff value for different QP
      for (i = 0; i < 6; i++) {
        pCtx->pDequant_coeff4x4[i] = pCtx->pDequant_coeff_buffer4x4[i];
        pCtx->pDequant_coeff8x8[i] = pCtx->pDequant_coeff_buffer8x8[i];
        for (q = 0; q < 51; q++) {
          for (x = 0; x < 16; x++) {
            pCtx->pDequant_coeff4x4[i][q][x] = pCtx->pPps->bPicScalingMatrixPresentFlag ? pCtx->pPps->iScalingList4x4[i][x] *
                                               g_kuiDequantCoeff[q][x & 0x07] : pCtx->pSps->iScalingList4x4[i][x] * g_kuiDequantCoeff[q][x & 0x07];
          }
          for (y = 0; y < 64; y++) {
            pCtx->pDequant_coeff8x8[i][q][y] = pCtx->pPps->bPicScalingMatrixPresentFlag ? pCtx->pPps->iScalingList8x8[i][y] *
                                               g_kuiMatrixV[q % 6][y / 8][y % 8] : pCtx->pSps->iScalingList8x8[i][y] * g_kuiMatrixV[q % 6][y / 8][y % 8];
          }
        }
      }
      pCtx->bDequantCoeff4x4Init = true;
      pCtx->iDequantCoeffPpsid = pCtx->pPps->iPpsId;
    }
  } else
    pCtx->bUseScalingList = false;
  return ERR_NONE;
}
typedef unsigned int bititem; // this makes it easier to gdb
struct EmitDefBitsToOMovie {
    void operator()(const uint8_t data, uint8_t nBits) const {
        oMovie().def().emitBits(data, nBits);
    }
};
struct EmitDefBitsToBoolVector {
    std::vector<bititem> *output;
    void operator()(const uint8_t data, uint8_t nBits) const {
        for (uint8_t i = 0; i < nBits; ++i) {
            if (data & (1<<(7 - i))) {
                output->push_back(true);
            } else {
                output->push_back(false);
            }

        }
    }
};
template<class Functor> void copySBitStringAux(const SBitStringAux& orig, Functor f) {
    for (const uint8_t *ptr = orig.pStartBuf; ptr != orig.pCurBuf; ++ptr) {
        f(*ptr, 8);
    }
    if (orig.iLeftBits < 32) {
        int nBits;
        uint32_t curBits;
        if (orig.iLeftBits < 0) { // decoding
            nBits = -orig.iLeftBits;
            curBits = orig.uiCurBits >> (-orig.iLeftBits);
        } else { // encoding
            nBits = 32 - orig.iLeftBits;
            curBits = orig.uiCurBits;
        }
        while (nBits > 0) {
            if (nBits < 8) {
                f((curBits) & ((1 << nBits) - 1), nBits);
            } else {
                f(((curBits) >> (nBits - 8)) & 0xff, 8);
            }
            nBits -= 8;
        }
    }
}
std::vector<bititem> bitStringToVector(const SBitStringAux& orig) {
    std::vector<bititem> retval;
    EmitDefBitsToBoolVector f;
    f.output = &retval;
    copySBitStringAux(orig, f);
    return retval;
}
bool trailing_zeros(const std::vector<bititem> &rvec, size_t longest_substring) {
    for(size_t i = longest_substring; i < rvec.size(); ++i) {
        if (rvec[i] != 0) {
            return false;
        }
    }
    return true;
}
/*
 * Currently returns true if rt is a bitwise substring of orig
 * Eventually will check for bitwise equality
 */
bool stringBitCompare(const std::vector<bititem> &ovec,
                      const std::vector<bititem> &rvec, size_t offset = 0) {
    size_t longest_substring = 0;
    size_t longest_offset = 0;
    size_t longest_rt_offset = 0;
    std::vector<bititem>::const_iterator oi = ovec.begin() + offset;
    (void)oi;
    std::vector<bititem>::const_iterator oend = ovec.end();
    (void)oend;
    std::vector<bititem>::const_iterator ri = rvec.begin(), rend = rvec.end();
    for (std::vector<bititem>::const_iterator oi = ovec.begin(), oend = ovec.end(); oi != oend;++oi) {
        /*for (std::vector<bititem>::const_iterator ri = rvec.begin(),
          rend = rvec.end(); ri != rend; ++ri)*/{
            size_t cur_substring = 0;
            std::vector<bititem>::const_iterator orig_cmp_iter = oi;
            std::vector<bititem>::const_iterator r_cmp_iter = ri;
            for (;orig_cmp_iter != oend && r_cmp_iter != rend; ++r_cmp_iter,++orig_cmp_iter) {
                if (*orig_cmp_iter != *r_cmp_iter) {
                    break;
                }
                ++cur_substring;
            }
            if (cur_substring > longest_substring) {
                longest_substring = cur_substring;
                longest_offset = oi - ovec.begin();
                longest_rt_offset = ri - rvec.begin();
            }
        }
    }
    bool ret;
    if (oMovie().isRecoding) {
        ret = longest_rt_offset == 0 && longest_substring == rvec.size();
    } else {
        ret = longest_rt_offset == 0 && longest_substring + 8 > rvec.size(); // && trailing_zeros(rvec, longest_substring); // need to allow zero-padding at the end of the stream
    }
    /*if (!ret) {
        std::string s = "";
        for (std::vector<bititem>::const_iterator oi = ovec.begin(), oend = ovec.end(), ri = rvec.begin(), rend = rvec.end(); oi != oend || ri != rend;){
            int bits = 0;
            bool odone = false;
            bool rdone = false;
            if (oi != oend) {
                bits |= ((int)(*oi)) << 1;
                ++oi;
                odone = (oi == oend);
            }
            if (ri != rend) {
                bits |= ((int)(*ri));
                ++ri;
                rdone = (ri == rend);
            }
            s += (char)('0' + bits);
            if (odone) {
                s += '<';
            }
            if (rdone) {
                s += '>';
            }
        }
        fprintf(stderr, "Bitstrings not equal! %s\n", s.c_str());
    }*/
    if (!ret ) {
        if (longest_substring < rvec.size()) {
            fprintf(stderr, "Longest prefix of rt[%ld] contained is %ld/%ld at orig[%ld] orig.size = %ld\n",
                    longest_rt_offset, longest_substring, rvec.size(), longest_offset, ovec.size());
        }
        std::string s = "";
        for (std::vector<bititem>::const_iterator oi = ovec.begin(); oi != ovec.end(); ++oi) {
            s += ('0' + (int)(*oi));
        }
        fprintf(stderr, "Origin bitstring %s\n", s.c_str());
        s = "";
        for (std::vector<bititem>::const_iterator ri = rvec.begin(); ri != rvec.end(); ++ri) {
            s += ('0' + (int)(*ri));
        }
        fprintf(stderr, "Result bitstring %s\n", s.c_str());
    }
    return ret;
}

bool stringBitCompare(const std::vector<bititem> &ovec,
                      const SBitStringAux& rt, size_t offset = 0) {
    std::vector<bititem> rvec = bitStringToVector(rt);
    return stringBitCompare(ovec, rvec, offset);
}

bool stringBitCompare(const PBitStringAux& orig,
                      const SBitStringAux& rt, size_t offset = 0) {
    std::vector<bititem> ovec = bitStringToVector(*orig);
    std::vector<bititem> rvec = bitStringToVector(rt);
    return stringBitCompare(ovec, rvec, offset);
}

struct EncoderState {
    WelsEnc::SDCTCoeff pDct;
    WelsEnc::sWelsEncCtx pEncCtx;
    WelsEnc::SSlice pSlice;
    WelsEnc::SMB pCurMb;
    WelsEnc::SDqLayer pCurDqLayer;
    WelsEnc::SWelsPPS pPpsP;
    WelsEnc::SMVUnitXY sMv[16];
    SBitStringAux wrBs;
#ifdef CABAC_HACK
    std::vector<uint8_t> buf;
#else
    uint8_t buf[MAX_MACROBLOCK_SIZE_IN_BYTE_x2 * 2];
#endif
    bool prevIntra4x4PredModeFlag[16];
    int8_t remIntra4x4PredModeFlag[16];
    int8_t pRefIndex[4];

    EncoderState(size_t size=MAX_MACROBLOCK_SIZE_IN_BYTE_x2 * 2)
            : pDct(), pEncCtx(), pSlice(), pCurMb(), pCurDqLayer(), pPpsP(),
             wrBs(), buf(), prevIntra4x4PredModeFlag(),
             remIntra4x4PredModeFlag(), pRefIndex() {
        pEncCtx.pCurDqLayer = &pCurDqLayer;
        pCurDqLayer.sLayerInfo.pPpsP = &pPpsP;
        pEncCtx.pFuncList = gFuncPtrList;
        pSlice.sMbCacheInfo.pDct = &pDct;
        pCurMb.pRefIndex = &pRefIndex[0];

        pSlice.sMbCacheInfo.pPrevIntra4x4PredModeFlag = prevIntra4x4PredModeFlag;
        pSlice.sMbCacheInfo.pRemIntra4x4PredModeFlag = remIntra4x4PredModeFlag;

#ifdef CABAC_HACK
        buf.resize(size);
        InitBits (&wrBs, &buf[0], buf.size());
#else
        InitBits (&wrBs, buf, sizeof(buf));
#endif
        pSlice.sMbCacheInfo.pDct = &pDct;
        pSlice.pSliceBsa = &wrBs;
    }
    void zigCopy(int16_t *zigdest, const int16_t *source, size_t num_components, bool dc) {
        for (size_t i = 0; i < num_components; ++i) {
            zigdest[i] = source[((i >> 4) << 4) | g_kuiZigzagScan[ i & 0xf ]];
        }
        if (! dc) {
            for (size_t i = 0; i < num_components; ++i) {
                if ((i & 0xf) == 0xf) {
                    zigdest[i] = 0;
                } else {
                    zigdest[i] = zigdest[i + 1];
                }
            }
        }

    }

    void lZigCopyDC(int16_t *zigdest, const int16_t *source, size_t num_components) {
        for (size_t i = 0; i < num_components; ++i) {
            zigdest[i] = source[((i >> 4) << 4) | (g_kuiLumaDcZigzagScan[ i & 0xf ] >> 4)];
        }
    }

    void setupCoefficientsFromOdata(const DecodedMacroblock::RawDCTData&odata) {
        zigCopy(&pDct.iLumaBlock[0][0], odata.lumaAC, sizeof(odata.lumaAC)/ sizeof(odata.lumaAC[0]), pCurMb.uiMbType != MB_TYPE_INTRA16x16);
        zigCopy(&pDct.iChromaBlock[0][0], odata.chromaAC, sizeof(odata.chromaAC)/ sizeof(odata.chromaAC[0]), false);
        //memcpy(pDct.iLumaI16x16Dc, odata.lumaDC, sizeof(odata.lumaDC));
        lZigCopyDC(&pDct.iLumaI16x16Dc[0], odata.lumaDC, sizeof(odata.lumaDC)/ sizeof(odata.lumaDC[0]));
        memcpy(pDct.iChromaDc, odata.chromaDC, sizeof(odata.chromaDC));
    }

    void initNonZeroCount(PDqLayer pCurLayer, const DecodedMacroblock::RawDCTData &odata) {
      int8_t *pNonZeroCount = pSlice.sMbCacheInfo.iNonZeroCoeffCount;
      memset(pNonZeroCount, 0xe7, 48);
      {
        SWelsNeighAvail sNeighAvail;
        GetNeighborAvailMbType (&sNeighAvail, pCurLayer);
        WelsFillCacheNonZeroCount (
            &sNeighAvail, (uint8_t*)pNonZeroCount, pCurLayer);
      }
      if (pCurMb.uiCbp & 15) {
        int start = 0;
        if (MB_TYPE_INTRA16x16 == pCurMb.uiMbType) {
          start = 1;
        }
        for (int i = 0; i < 16; i++) {
          int numnzc = 0;
          for (int coef = start; coef < 16; coef++) {
            if (odata.lumaAC[i * 16 + coef]) {
              numnzc++;
            }
          }
          pNonZeroCount[g_kuiCache48CountScan4Idx[i]] = numnzc;
        }
      }
      if (pCurMb.uiCbp >= 32) {
        for (int cbcr = 0; cbcr < 2; cbcr++) {
          for (int i = 0; i < 4; i++) {
            int numnzc = 0;
            for (int coef = 1; coef < 16; coef++) {
              if (odata.chromaAC[cbcr * 64 + i * 16 + coef]) {
                numnzc++;
              }
            }
            pNonZeroCount[g_kuiCache48CountScan4Idx[cbcr * 4 + i + 16]] = numnzc;
          }
        }
      }
      if (pCurMb.uiCbp && MB_TYPE_INTRA16x16 != pCurMb.uiMbType) {
        for (int iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          if (!(pCurMb.uiCbp & (1 << iId8x8))) {
            pNonZeroCount[g_kuiCache48CountScan4Idx[iId8x8 << 2]] = 0;
            pNonZeroCount[g_kuiCache48CountScan4Idx[iId8x8 << 2] + 1] = 0;
            pNonZeroCount[g_kuiCache48CountScan4Idx[(iId8x8 << 2) + 2]] = 0;
            pNonZeroCount[g_kuiCache48CountScan4Idx[(iId8x8 << 2) + 2] + 1] = 0;
          }
        }
      }
    }

    void init(DecodedMacroblock *rtd) {

        pPpsP.uiChromaQpIndexOffset = rtd->uiChromaQpIndexOffset;

        // pEncCtx->pCurDqLayer->sLayerInfo.pPpsP->uiChromaQpIndexOffset FIXME?
        pEncCtx.eSliceType = WelsCommon::EWelsSliceType(rtd->eSliceType);
        pSlice.sSliceHeaderExt.sSliceHeader.eSliceType = pEncCtx.eSliceType;

        // pSlice.sMbCacheInfo.sMvComponents may be need for cabac

        size_t num_components = 0;
        if (rtd->uiMbType == MB_TYPE_INTRA8x8) {
            num_components = 4;
        } else if (rtd->uiMbType == MB_TYPE_INTRA4x4) {
            num_components = 16;
        }
        for (size_t i = 0; i < num_components; i++) {
            prevIntra4x4PredModeFlag[i] = !!(rtd->iPrevIntra4x4PredMode[i]);
            remIntra4x4PredModeFlag[i] = (int8_t)(rtd->iRemIntra4x4PredMode[i]);
        }

        pSlice.sMbCacheInfo.uiChmaI8x8Mode = rtd->uiChmaI8x8Mode;
        // from: 1932  pCurLayer->pIntraPredMode[iMbXy][7] = (uiMbType - 1) & 3; in WelsDec::WelsActualDecodeMbCavlcISlice
        pSlice.sMbCacheInfo.uiLumaI16x16Mode = rtd->uiLumaI16x16Mode;

        // pMbCache->sMbMvp is left as 0 so that we can just write the MV deltas we read in.
        pSlice.sSliceHeaderExt.sSliceHeader.uiNumRefIdxL0Active = rtd->uiNumRefIdxL0Active;
        pSlice.uiLastMbQp = rtd->iLastMbQp;
        pSlice.iMbSkipRun = rtd->iMbSkipRun;
        pSlice.uiSliceIdx = 0;
        for (int i = 0; i < 4; i++) {
            // only 8x8
            pCurMb.uiSubMbType[i] = rtd->uiSubMbType[i];
        }

        if (rtd->uiMbType == MB_TYPE_16x8) {
          pCurMb.pRefIndex[0] = rtd->iRefIdx[0];
          pCurMb.pRefIndex[2] = rtd->iRefIdx[1];
        } else {
          for (int i = 0; i < 4; i++) {
            pCurMb.pRefIndex[i] = rtd->iRefIdx[i];
          }
        }
        pCurMb.sMv = &sMv[0];
        for (int i = 0; i < 16; i++) {
            pCurMb.sMv[i].iMvX = rtd->sMbMvp[i][0];
            pCurMb.sMv[i].iMvY = rtd->sMbMvp[i][1];
        }
        pCurMb.uiMbType = rtd->uiMbType;
        pCurMb.uiCbp = ((rtd->uiCbpC << 4) | (rtd->uiCbpL & 15));
        // CbpC bits for which 8x8 block to encode luma?
        // CbpL 0 = no chroma; 1 = dc only; 2 = dc&ac
        pCurMb.uiLumaQp = rtd->uiLumaQp;
        // FIXME: add support for Cb and Cr in uiChromaQp
        // and reference uiChromaQpIndexOffset[i] from pPpsp.
        for (int i = 0; i < 2; i++) {
          pCurMb.uiChromaQp = g_kuiChromaQpTable[WELS_CLIP3 (rtd->uiLumaQp +
              rtd->uiChromaQpIndexOffset, 0, 51)];
        }
    }

};

bool knownCodeUnitTest(DecodedMacroblock::RawDCTData &odata,
                       const std::vector<bititem> expectedPrefix) {
    SBitStringAux wrBs;
    uint8_t buf[MAX_MACROBLOCK_SIZE_IN_BYTE_x2 * 2] = {0}; // Cannot be larger than raw input.
    WelsEnc::SDCTCoeff pDct = {};
    memcpy(pDct.iLumaBlock, odata.lumaAC, sizeof(odata.lumaAC));
    memcpy(pDct.iChromaBlock, odata.chromaAC, sizeof(odata.chromaAC));
    memcpy(pDct.iLumaI16x16Dc, odata.lumaDC, sizeof(odata.lumaDC));
    memcpy(pDct.iChromaDc, odata.chromaDC, sizeof(odata.chromaDC));

    InitBits (&wrBs, buf, sizeof(buf));
    ENFORCE_STACK_ALIGN_1D (uint8_t, pNonZeroCount, 48, 16);
    memset(pNonZeroCount, 0, 48 * sizeof(*pNonZeroCount));
    //pNonZeroCount[0] = 5;
    //pNonZeroCount[1] = 3;
    pNonZeroCount[9] = 5;

    WelsEnc::sWelsEncCtx pEncCtx = {};
    WelsEnc::SSlice pSlice = {};
    WelsEnc::SMB pCurMb = {};

    // This stuff isn't really used.
    WelsEnc::SDqLayer pCurDqLayer = {};
    WelsEnc::SWelsPPS pPpsP = {};
    pEncCtx.pCurDqLayer = &pCurDqLayer;
    pCurDqLayer.sLayerInfo.pPpsP = &pPpsP;
    pPpsP.uiChromaQpIndexOffset = 0;

    // pEncCtx->pCurDqLayer->sLayerInfo.pPpsP->uiChromaQpIndexOffset
    pEncCtx.eSliceType = I_SLICE;
    pSlice.sSliceHeaderExt.sSliceHeader.eSliceType = pEncCtx.eSliceType;
    pEncCtx.pFuncList = gFuncPtrList;

    memcpy(pSlice.sMbCacheInfo.iNonZeroCoeffCount, pNonZeroCount, 48);
    // pSlice.sMbCacheInfo.sMvComponents may be need for cabac
    // pMbCache->pPrevIntra4x4PredModeFlag // Only for intra4x4
    // pMbCache->pRemIntra4x4PredModeFlag // Only for intra4x4
    // pMbCache->uiChmaI8x8Mode // Only for intra4x4 or intra16x16
    // pMbCache->sMbMvp
    pSlice.sMbCacheInfo.pDct = &pDct;
    pSlice.pSliceBsa = &wrBs;
    // pSlice.sSliceHeaderExt.sSliceHeader.uiNumRefIdxL0Active // Number of reference frames.
    // pSlice.uiLastMbQp = decoder.pSlice->iLastMbQp
    // iMbSkipRun
    // uiSliceIdx
    // pCurMb.uiSubMbType // only 8x8
    // pCurMb.pRefIndex[0..3]
    // pCurMb.sMv
    pCurMb.uiMbType = MB_TYPE_INTRA16x16;
    // pCurMb->uiCbp = uiCbpC << 4 | (uiCbpL & 15)
    // CbpC bits for which 8x8 block to encode luma?
    // CbpL 0 = no chroma; 1 = dc only; 2 = dc&ac
    pCurMb.uiCbp = 15;
    // pCurMb.uiLumaQp
    // pCurMb->uiChromaQp

    if (WelsEnc::WelsSpatialWriteMbSyn (
                &pEncCtx, &pSlice, &pCurMb)) {
        fprintf(stderr, "Encode failed!");
        return false;
    }
    bool retval = stringBitCompare(expectedPrefix, wrBs);
    return retval;
}

bool knownCodeUnitTest0() {
    DecodedMacroblock::RawDCTData odata;
    memset(&odata, 0, sizeof(odata));

    std::vector<bititem> expectedPrefix;
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    bool retval = knownCodeUnitTest(odata, expectedPrefix);
    return retval; // I think the expectedPrefix is wrong
    //return true;
}

bool knownCodeUnitTest1() {
    DecodedMacroblock::RawDCTData odata;
    memset(&odata, 0, sizeof(odata));
    odata.lumaAC[0] = 0;
    odata.lumaAC[1] = 3;
    odata.lumaAC[2] = 0;
    odata.lumaAC[3] = 1;
    odata.lumaAC[4] = -1;
    odata.lumaAC[5] = -1;
    odata.lumaAC[6] = 0;
    odata.lumaAC[7] = 1;
    odata.lumaAC[8] = 0;
/*
    odata.lumaAC[1] = 3;
    odata.lumaAC[2] = -1;
    int w = 16;// is this 4?
    odata.lumaAC[1 + w] = -1;
    odata.lumaAC[2 + w] = 1;
    odata.lumaAC[0 + 2*w] = 1;
*/

    std::vector<bititem> expectedPrefix;
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    return knownCodeUnitTest(odata, expectedPrefix);
}

bool knownCodeUnitTest2() {
    DecodedMacroblock::RawDCTData odata;
    memset(&odata, 0, sizeof(odata));
    odata.lumaAC[0] = -2;
    odata.lumaAC[1] = 4;
    odata.lumaAC[2] = 3;
    odata.lumaAC[3] = -3;
    odata.lumaAC[4] = 0;
    odata.lumaAC[5] = 0;
    odata.lumaAC[6] =-1;
    odata.lumaAC[7] = 0;
    odata.lumaAC[8] = 0;
/* if row major
    odata.lumaAC[0] = -2;
    odata.lumaAC[1] = 4;
    odata.lumaAC[2] =-1;

    int w = 16; // is this 4?
    odata.lumaAC[w] = 3;
    odata.lumaAC[2*w] = -3;
*/

    std::vector<bititem> expectedPrefix;
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(1);
    expectedPrefix.push_back(0);
    expectedPrefix.push_back(0);

    return knownCodeUnitTest(odata, expectedPrefix);
}

const uint8_t kzz[16] ={
    0, 1,  4, 8,
    5, 2, 3, 6,
    9, 12, 13, 10,
    7, 11, 14, 15
};
const uint8_t unzz[16] ={
    0, 1, 5, 6,
    2, 4, 7, 12,
    3, 8, 11, 13,
    9, 10, 14, 15
};
uint8_t log2(uint16_t v) {
    const unsigned int b[] = {0x2, 0xC, 0xF0, 0xFF00};
    const unsigned int S[] = {1, 2, 4, 8, 16};
    int i;

    unsigned int r = 0; // result of log2(v) will go here
    for (i = 3; i >= 0; i--) // unroll for speed...
    {
      if (v & b[i])
      {
        v >>= S[i];
        r |= S[i];
      }
    }
    return r;
}
uint8_t bit_length(uint16_t value) {
    if (value == 0) return 0;
    uint16_t ret = log2(value) + 1;
    return (uint8_t)ret;
}

void encode4x4(const int16_t *ac, int index, bool emit_dc, int color) {
    (void)unzz;
    int num_nonzeros = oMovie().model().get4x4NumNonzeros(index, color);
    int num_nonzeros_left = num_nonzeros;
    bool nonzero[16] = {0};
    for (int coef_uzz = emit_dc ? 0 : 1; coef_uzz < 16 && num_nonzeros_left; ++coef_uzz) {
        int coef = kzz[coef_uzz];
        int stream_id = (color ? PIP_CRAC_TAG0 : PIP_LAC_TAG0) + PIP_AC_STEP * (coef);
        if (ac[coef]) {
            nonzero[coef] = true;
            --num_nonzeros_left;
        }
        oMovie().tag(stream_id).emitBit(!!nonzero[coef],
                                        oMovie().model().getNonzeroPrior(nonzero,
                                                                         index,
                                                                         coef,
                                                                         emit_dc,
                                                                         color));
    }
    for (int coef_uzz = 15; coef_uzz >= (emit_dc ? 0 : 1); --coef_uzz) {
        int coef = kzz[coef_uzz];
        if (nonzero[coef]) {
            int stream_id = (color ? PIP_CRAC_TAG0 : PIP_LAC_TAG0) + PIP_AC_STEP * (coef);
            uint16_t abs_ac = ac[coef] < 0 ? -ac[coef] : ac[coef];
            abs_ac -= 1; // we know it ain't zero
            int bit_len = bit_length(abs_ac);
            //fprintf(stderr, "Encoding %d(%d) ", bit_len, abs_ac);
            oMovie().tag(stream_id).emitBits(bit_len,
                                             oMovie().model().getAcExpPrior(nonzero,
                                                                             ac,
                                                                             index,
                                                                             coef,
                                                                             emit_dc,
                                                                             color));
            if (bit_len > 1) {
                for(int which_bit = bit_len - 2; which_bit >=0; --which_bit) {
                    oMovie().tag(stream_id).emitBits((abs_ac & (1 << which_bit)) ? 1 : 0, 1);
                    //fprintf(stderr, " %d ", (abs_ac & (1 << which_bit)) ? 1 : 0);
                }
            }
            //fprintf(stderr, "%c\n", (ac[coef] < 0 ? '-' : '+'));
            oMovie().tag(stream_id).emitBits(ac[coef] < 0 ? 1 : 0, 1);
        }
    }
}

void decode4x4(int16_t *ac, int index, bool emit_dc, int color) {
    memset(ac, 0, sizeof(int16_t)*16);
    int num_nonzeros = oMovie().model().get4x4NumNonzeros(index, color);
    int num_nonzeros_left = num_nonzeros;
    bool nonzero[16] = {0};
    for (int coef_uzz = emit_dc ? 0 : 1; coef_uzz < 16 && num_nonzeros_left; ++coef_uzz) {
        int coef = kzz[coef_uzz];
        int stream_id = (color ? PIP_CRAC_TAG0 : PIP_LAC_TAG0) + PIP_AC_STEP * (coef);
        bool nz = iMovie().tag(stream_id).scanBit(oMovie().model().getNonzeroPrior(nonzero,
                                                                                   index,
                                                                                   coef,
                                                                                   emit_dc,
                                                                                   color));
        if (nz) {
            nonzero[coef] = true;
            --num_nonzeros_left;
        }
    }
    for (int coef_uzz = 15; coef_uzz >= (emit_dc ? 0 : 1); --coef_uzz) {
        int coef = kzz[coef_uzz];
        if (nonzero[coef]) {
            BitStream::uint32E res;
            int stream_id = (color ? PIP_CRAC_TAG0 : PIP_LAC_TAG0) + PIP_AC_STEP * (coef);
            res = iMovie().tag(stream_id).scanBits(oMovie().model().getAcExpPrior(nonzero,
                                                                                   ac,
                                                                                   index,
                                                                                   coef,
                                                                                   emit_dc,
                                                                                   color));
            if (res.second) {
                fprintf(stderr, "Cannot decode AC component %d, %d\n", coef, (int)res.second);
            }
            int bit_len = res.first;
            //fprintf(stderr, "Decoding %d ", bit_len);
            if (bit_len) {
                ac[coef] = (1 << (bit_len - 1));
            } else {
                ac[coef] = 0;
            }
            if (bit_len > 1) {
                for(int which_bit = bit_len - 2; which_bit >=0; --which_bit) {
                    std::pair<uint32_t, H264Error> res = iMovie().tag(stream_id).scanBits(1);
                    if (res.second) {
                        fprintf(stderr,"Cannot scan bit for nonzero %d \n", which_bit);
                    }
                    if (res.first) {
                        ac[coef] |= (1 << which_bit);
                    }
                    //fprintf(stderr, " %d ", res.first);
                }
            }
            ++ac[coef];
            std::pair<uint32_t, H264Error> sign = iMovie().tag(stream_id).scanBits(1);
            if (sign.second) {
                fprintf(stderr,"Cannot scan sign bit for ceof %d \n", coef);
            }
            //fprintf(stderr, "%c\n", (sign.first ? '-' : '+'));
            if (sign.first) {
                ac[coef] = -ac[coef];
            }
        } else {
            assert(ac[coef] == 0);
        }
    }
}

int32_t WelsDecodeSliceForNonRecoding(PWelsDecoderContext pCtx,
                                      PNalUnit pNalCur,
                                      PSlice pSlice,
                                      int32_t iNextMbXyIndex,
                                      uint32_t& uiEosFlag,
                                      DecodedMacroblock& rtd,
                                      PWelsDecMbFunc pDecMbFunc,
                                      PDqLayer pCurLayer,
                                      int& origSkipped) {

  bool writeSkipRun = (-1 == pSlice->iMbSkipRun);
  int32_t iRet = pDecMbFunc (pCtx,  pNalCur, uiEosFlag, &rtd);
  PBitStringAux pBs = pCurLayer->pBitStringAux;
  int32_t iUsedBits = ((pBs->pCurBuf - pBs->pStartBuf) << 3) - (16 - pBs->iLeftBits);
  bool hasExactlyOneStopBit = (iUsedBits == (pBs->iBits - 1));
#ifdef DEBUG_PRINTS
  fprintf(stderr, "EOSTEST which_block=%d origSkipped=%d skip=%d endofslice=%d uiEosFlag=%d\n", which_block, origSkipped == -1 ? 0 : origSkipped, pSlice->iMbSkipRun, hasExactlyOneStopBit, uiEosFlag);
#endif
  if (writeSkipRun) {
#ifdef DEBUG_PRINTS
    fprintf(stderr, "block=%d write skip&eos!\n", which_block);
#endif
    oMovie().tag(PIP_SKIP_TAG).emitBits(rtd.iMbSkipRun, 16);
  } else {
#ifdef DEBUG_PRINTS
    fprintf(stderr, "block=%d not write skip&eos!\n", which_block);
#endif
  }
  if (rtd.iMbSkipRun == 1) {
    // We are done while finishing a skip. writeBlock will not be true.
    oMovie().tag(PIP_SKIP_END_TAG).emitBits(hasExactlyOneStopBit, 1);
  }
  bool initialSkip = (rtd.iMbSkipRun > 0 && origSkipped == -1);
  bool finalSkip = (rtd.iMbSkipRun == 0 && origSkipped != -1);
  bool writeBlock = rtd.iMbSkipRun == 0;
#ifdef DEBUG_PRINTS
  fprintf(stderr, "OUTSIDE skiprun=%d ; origSkip%d ; which_block=%d initSkp=%d finalSkp=%d writeBlock=%d | iMbXyIndex=%d iMbX=%d iMbY=%d\n", rtd.iMbSkipRun, origSkipped, (int)(uint16_t)which_block, (int)initialSkip, (int)finalSkip, (int)writeBlock, pCurLayer->iMbXyIndex, pCurLayer->iMbX, pCurLayer->iMbY);
#endif
  if (initialSkip) {
    origSkipped = rtd.iMbSkipRun;
  }
  if (finalSkip) {
    rtd.iMbSkipRun = origSkipped;
    origSkipped = -1;
  }
  pCurLayer->pMbRefConcealedFlag[iNextMbXyIndex] = pCtx->bMbRefConcealed;
  if (iRet != ERR_NONE) {
    return iRet;
  }
  if (writeBlock) {
    initRTDFromDecoderState(rtd, pCurLayer);

    oMovie().tag(PIP_SKIP_END_TAG).emitBits(hasExactlyOneStopBit, 1);
    oMovie().tag(PIP_MB_TYPE_TAG).emitBits(oMovie().model().encodeMacroblockType(rtd.uiMbType), oMovie().model().getMacroblockTypePrior());
    uint16_t numNonzerosL = oMovie().model().getAndUpdateMacroblockLumaNumNonzeros();
    uint8_t numNonzerosC = oMovie().model().getAndUpdateMacroblockChromaNumNonzeros();
#ifdef HIERARCHICAL_NONZEROS
    oMovie().tag(PIP_NZC_TAG).emitBitsZeroToPow2Inclusive<8>(numNonzerosL, oMovie().model().getLumaNumNonzerosPrior()); //Valid values are 0..256 incl
    oMovie().tag(PIP_NZC_TAG).emitBitsZeroToPow2Inclusive<7>(numNonzerosC, oMovie().model().getChromaNumNonzerosPrior()); //Valid values are 0..128 incl
#else
    (void)numNonzerosL;
    (void)numNonzerosC;
#endif
    uint8_t runningCount = 0;
    for (int i = 0; i < 16; ++i) {
      oMovie().tag(PIP_NZC_TAG).emitBitsZeroToPow2Inclusive<4>(rtd.numSubLumaNonzeros_[i],
                                                               oMovie().model().getSubLumaNumNonzerosPrior(i, runningCount));
      runningCount += rtd.numSubLumaNonzeros_[i];
    }
    runningCount = 0;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 8; j += 4) {
        oMovie().tag(PIP_NZC_TAG).emitBitsZeroToPow2Inclusive<4>(rtd.numSubChromaNonzeros_[i + j],
                                                                 oMovie().model().getSubChromaNumNonzerosPrior(i + j, runningCount));
        runningCount += rtd.numSubChromaNonzeros_[i + j];
      }
    }
    if (pCtx->pSps->uiChromaFormatIdc != 0) {
      oMovie().tag(PIP_CBPC_TAG).emitBits(rtd.uiCbpC, 8); // Valid values are 0..2
    }
    oMovie().tag(PIP_CBPL_TAG).emitBits(rtd.uiCbpL, 8); // Valid values are 0..15
    oMovie().tag(PIP_LAST_MB_TAG).emitBits((uint16_t)rtd.iLastMbQp, 16);
    oMovie().tag(PIP_QPL_TAG).emitBits(rtd.uiLumaQp, 16);
    oMovie().tag(PIP_REF_TAG).emitBits(rtd.uiNumRefIdxL0Active, 8);
    std::pair<Sirikata::Array1d<DynProb, 8>::Slice, uint32_t> chroma_prior_pair
      = oMovie().model().getChromaI8x8ModePrior();
    oMovie().tag(PIP_8x8_TAG).emitBitsZeroToPow2Inclusive<3>(rtd.uiChmaI8x8Mode,
                                                             chroma_prior_pair.first,
                                                             chroma_prior_pair.second);
    std::pair<Sirikata::Array1d<DynProb, 8>::Slice, uint32_t> luma_prior_pair
      = oMovie().model().getLumaI16x16ModePrior();
    oMovie().tag(PIP_16x16_TAG).emitBitsZeroToPow2Inclusive<3>(rtd.uiLumaI16x16Mode,
                                                               luma_prior_pair.first,
                                                               luma_prior_pair.second);

    if (MB_TYPE_INTRA4x4 == rtd.uiMbType) {
      for (int i = 0; i < 16; i++) {
        oMovie().tag(PIP_PREV_PRED_TAG).emitBits((uint16_t)rtd.iPrevIntra4x4PredMode[i], 8);
      }
      for (int i = 0; i < 16; i++) {
        oMovie().tag(PIP_PRED_TAG).emitBits((uint16_t)rtd.iRemIntra4x4PredMode[i], 8);
      }
    }
    if (MB_TYPE_INTRA8x8 == rtd.uiMbType) {
      for (int i = 0; i < 4; i++) {
        oMovie().tag(PIP_PREV_PRED_MODE_TAG).emitBits((uint8_t)rtd.iPrevIntra4x4PredMode[i], 8);
      }
      for (int i = 0; i < 4; i++) {
        oMovie().tag(PIP_PRED_MODE_TAG).emitBits((uint8_t)rtd.iRemIntra4x4PredMode[i], 8);
      }
      for (int i = 0; i < 4; i++) {
        oMovie().tag(PIP_SUB_MB_TAG).emitBits(rtd.uiSubMbType[i], 8);
      }
      for (int i = 0; i < 4; i++) {
        oMovie().tag(PIP_REF_TAG).emitBits((uint8_t)rtd.iRefIdx[i], 8);
      }
    } else if (MB_TYPE_8x8 == rtd.uiMbType) {
      for (int i = 0; i < 4; i++) {
        oMovie().tag(PIP_SUB_MB_TAG).emitBits(rtd.uiSubMbType[i], 8);
      }
      for (int i = 0; i < 4; i++) {
        oMovie().tag(PIP_REF_TAG).emitBits((uint8_t)rtd.iRefIdx[i], 8);
      }
    } else if (MB_TYPE_8x8_REF0 == rtd.uiMbType) {
      for (int i = 0; i < 4; i++) {
        oMovie().tag(PIP_SUB_MB_TAG).emitBits(rtd.uiSubMbType[i], 8);
      }
    } else if (MB_TYPE_8x16 == rtd.uiMbType ||
               MB_TYPE_16x8 == rtd.uiMbType) {
      for (int i = 0; i < 2; i++) {
        oMovie().tag(PIP_REF_TAG).emitBits(rtd.iRefIdx[i], 8);
      }
    } else if (MB_TYPE_16x16 == rtd.uiMbType) {
      oMovie().tag(PIP_REF_TAG).emitBits(rtd.iRefIdx[0], 8);
    }
    if (MB_TYPE_INTRA16x16 != rtd.uiMbType &&
        MB_TYPE_INTRA8x8 != rtd.uiMbType &&
        MB_TYPE_INTRA4x4 != rtd.uiMbType) {
      for (int i = 0; i < 16; i++) {
        oMovie().tag(PIP_MVX_TAG).emitBits((uint16_t)rtd.sMbMvp[i][0], 16);
        oMovie().tag(PIP_MVY_TAG).emitBits((uint16_t)rtd.sMbMvp[i][1], 16);
      }
    }
    bool emitted_luma_dc = false;
    bool emitted_chroma_dc = false;
    if (MB_TYPE_INTRA16x16 == rtd.uiMbType) {
      emitted_luma_dc = true;
      for (int i = 0; i < 16; i++) {
        oMovie().tag(PIP_LDC_TAG).emitBits((uint16_t)rtd.odata.lumaDC[i], 16);
      }
    }
    if (1 == rtd.uiCbpC || 2 == rtd.uiCbpC) {
      emitted_chroma_dc = true;
      for (int i = 0; i < 8; i++) {
        oMovie().tag(PIP_CRDC_TAG).emitBits((uint16_t)rtd.odata.chromaDC[i], 16);
      }
    }
    if (rtd.uiCbpL) {
      for (int i = 0; i < 16; i++) {
        encode4x4(&rtd.odata.lumaAC[i * 16], i, !emitted_luma_dc, 0);
        /*
          if ((i & 15) != 0 || !emitted_luma_dc) { // the dc hasn't been emitted, we need to emit it now (or any of the AC's)
          oMovie().tag(PIP_LAC_TAG0 + PIP_AC_STEP * i % 16).emitBits((uint16_t)rtd.odata.lumaAC[i], 16);
          }*/
      }
    }
    if (rtd.uiCbpC == 2) {
      for (int i = 0; i < 8; i++) {
        encode4x4(&rtd.odata.chromaAC[i * 16], (i&3), !emitted_chroma_dc, i < 4 ? 1 : 2);
        /*
          if ((i & 15) != 0 || !emitted_chroma_dc) { // the dc hasn't been emitted, we need to emit it now (or any of the AC's)
          oMovie().tag(PIP_CRAC_TAG0 + PIP_AC_STEP * i % 8).emitBits((uint16_t)rtd.odata.chromaAC[i], 16);
          }*/
      }
    }
#ifdef DEBUG_PRINTS
    fprintf(stderr, "INSIDE skiprun=%d ; origSkip%d ; which_block=%d\n", rtd.iMbSkipRun, origSkipped, (int)(uint16_t)which_block);
    fprintf(stderr, "all done!\n");
#endif


#ifdef ROUNDTRIP_TEST
    {
      EncoderState es;
      es.init(&rtd);
      es.setupCoefficientsFromOdata(rtd.odata);
      es.initNonZeroCount(pCurLayer, rtd.odata);
      woffset = 0;
      WelsEnc::WelsSpatialWriteMbSyn (
                                      &es.pEncCtx, &es.pSlice, &es.pCurMb);
      assert(stringBitCompare(pCurLayer->pBitStringAux, es.wrBs, 22));
    }
#endif
  }
}

int32_t WelsDecodeSliceForRecoding(PWelsDecoderContext pCtx,
                                   PNalUnit pNalCur,
                                   PSlice pSlice,
                                   int32_t iNextMbXyIndex,
                                   uint32_t& uiEosFlag,
                                   DecodedMacroblock& rtd,
                                   PWelsDecMbFunc pDecMbFunc,
                                   PDqLayer pCurLayer,
                                   int& origSkipped,
                                   int& curSkipped) {
  bool endOfSlice = false;

#ifdef DEBUG_PRINTS
  fprintf(stderr, "START skiprun=%d ; origSkip%d ; which_block=%d curSkip=%d |1187XyIndex=%d iMbX=%d iMbY=%d\n", rtd.iMbSkipRun, origSkipped, (int)(uint16_t)which_block, (int)curSkipped, pCurLayer->iMbXyIndex, pCurLayer->iMbX, pCurLayer->iMbY);
#endif
  if (curSkipped == -1) {
    origSkipped = -1;
  }
  BitStream::uint32E res;
  rtd.uiMbType = MB_TYPE_SKIP;
  if (curSkipped == -1) {
#ifdef DEBUG_PRINTS
    fprintf(stderr, "block=%d read skip&eos!\n", which_block);
#endif
    res = iMovie().tag(PIP_SKIP_TAG).scanBits(16);
    if (res.second) {
      fprintf(stderr, "failed to read iMbSkipRun!\n");
    } else {
      origSkipped = curSkipped = res.first;
    }
  } else {
#ifdef DEBUG_PRINTS
    fprintf(stderr, "block=%d not read skip&eos!\n", which_block);
#endif
  }
  if (curSkipped == 1) {
    // We need to know if this macroblock ends in a skip or ends in a block.
    res = iMovie().tag(PIP_SKIP_END_TAG).scanBits(1);
    if (res.second) {
      fprintf(stderr, "failed to read eos!\n");
    } else {
      endOfSlice = res.first;
    }
  }
  if (curSkipped == 0) { // Decrementing after decode
    res = iMovie().tag(PIP_SKIP_END_TAG).scanBits(1);
    if (res.second) {
      fprintf(stderr, "failed to read eos!\n");
    } else {
      endOfSlice = res.first;
    }
    res = iMovie().tag(PIP_MB_TYPE_TAG).scanBits(oMovie().model().getMacroblockTypePrior());
    if (res.second) {
      fprintf(stderr, "failed to read iMbType!\n");
      rtd.uiMbType = 0;
    } else {
      rtd.uiMbType = oMovie().model().decodeMacroblockType(res.first);
    }

#ifdef HIERARCHICAL_NONZEROS
    res = iMovie().tag(PIP_NZC_TAG).scanBitsZeroToPow2Inclusive<8>(oMovie().model().getLumaNumNonzerosPrior());
    rtd.numLumaNonzeros_ = res.first;
    if (res.second) {
      fprintf(stderr, "failed to read Luma Nonzeros!\n");
      rtd.numLumaNonzeros_ = 0;
    }
    res = iMovie().tag(PIP_NZC_TAG).scanBitsZeroToPow2Inclusive<7>(oMovie().model().getChromaNumNonzerosPrior());
    rtd.numChromaNonzeros_ = res.first;
    if (res.second) {
      fprintf(stderr, "failed to read Chroma Nonzeros!\n");
    }
#endif

    uint8_t runningCount = 0;
    for (int i = 0; i < 16; ++i) {
      res = iMovie().tag(PIP_NZC_TAG).scanBitsZeroToPow2Inclusive<4>(oMovie().model().getSubLumaNumNonzerosPrior(i, runningCount));
      rtd.numSubLumaNonzeros_[i] = res.first;
      runningCount += res.first;
      if (res.second) {
        fprintf(stderr, "failed to read Sub Luma Nonzeros!\n");
      }
    }
    rtd.numLumaNonzeros_ = runningCount;
    runningCount = 0;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 8; j += 4) {
        res = iMovie().tag(PIP_NZC_TAG).scanBitsZeroToPow2Inclusive<4>(oMovie().model().getSubChromaNumNonzerosPrior(i + j, runningCount));
        rtd.numSubChromaNonzeros_[i + j] = res.first;
        runningCount += res.first;
        if (res.second) {
          fprintf(stderr, "failed to read Sub Chroma Nonzeros!\n");
        }
      }
    }
    rtd.numChromaNonzeros_ = runningCount;
    if (pCtx->pSps->uiChromaFormatIdc != 0) {
      res = iMovie().tag(PIP_CBPC_TAG).scanBits(8);
      if (res.second) {
        fprintf(stderr, "failed to read uiCbpC!\n");
        rtd.uiCbpC = 255;
      } else {
        rtd.uiCbpC = res.first;
      }
    }
    res = iMovie().tag(PIP_CBPL_TAG).scanBits(8);
    if (res.second) {
      fprintf(stderr, "failed to read uiCbpL!\n");
      rtd.uiCbpL = 255;
    } else {
      rtd.uiCbpL = res.first;
    }
    res = iMovie().tag(PIP_LAST_MB_TAG).scanBits(16);
    if (res.second) {
      fprintf(stderr, "failed to read iLastMbQp!\n");
      rtd.iLastMbQp = 255;
    } else {
      rtd.iLastMbQp = res.first;
    }
    res = iMovie().tag(PIP_QPL_TAG).scanBits(16);
    if (res.second) {
      fprintf(stderr, "failed to read uiLumaQp!\n");
      rtd.uiLumaQp = 255;
    } else {
      rtd.uiLumaQp = res.first;
    }
    res = iMovie().tag(PIP_REF_TAG).scanBits(8);
    if (res.second) {
      fprintf(stderr, "failed to read uiRefIdxL0!\n");
      rtd.uiNumRefIdxL0Active = 255;
    } else {
      rtd.uiNumRefIdxL0Active = res.first;
    }
    std::pair<Sirikata::Array1d<DynProb, 8>::Slice, uint32_t> chroma_prior_pair
      = oMovie().model().getChromaI8x8ModePrior();
    res = iMovie().tag(PIP_8x8_TAG).scanBitsZeroToPow2Inclusive<3>(chroma_prior_pair.first, chroma_prior_pair.second);
    if (res.second) {
      fprintf(stderr, "failed to read uiChmaI8x8Mode!\n");
      rtd.uiChmaI8x8Mode = 255;
    } else {
      rtd.uiChmaI8x8Mode = res.first;
    }
    std::pair<Sirikata::Array1d<DynProb, 8>::Slice, uint32_t> luma_prior_pair
      = oMovie().model().getLumaI16x16ModePrior();
    res = iMovie().tag(PIP_16x16_TAG).scanBitsZeroToPow2Inclusive<3>(luma_prior_pair.first, luma_prior_pair.second);
    if (res.second) {
      fprintf(stderr, "failed to read uiLumaI16x16Mode!\n");
      rtd.uiLumaI16x16Mode = 255;
    } else {
      rtd.uiLumaI16x16Mode = res.first;
    }

    if (MB_TYPE_INTRA4x4 == rtd.uiMbType) {
      for (int i = 0; i < 16; i++) {
        res = iMovie().tag(PIP_PREV_PRED_TAG).scanBits(8);
        if (res.second) {
          fprintf(stderr, "failed to read prevPredMode!\n");
          rtd.iPrevIntra4x4PredMode[i] = 0;
        } else {
          rtd.iPrevIntra4x4PredMode[i] = res.first;
        }
      }
      for (int i = 0; i < 16; i++) {
        res = iMovie().tag(PIP_PRED_TAG).scanBits(8);
        if (res.second) {
          fprintf(stderr, "failed to read remPredMode!\n");
          rtd.iRemIntra4x4PredMode[i] = 0;
        } else {
          rtd.iRemIntra4x4PredMode[i] = res.first;
        }
      }
    } else if (MB_TYPE_INTRA8x8 == rtd.uiMbType) {
      for (int i = 0; i < 4; i++) {
        res = iMovie().tag(PIP_PREV_PRED_MODE_TAG).scanBits(8);
        if (res.second) {
          fprintf(stderr, "failed to read prevPredMode!\n");
          rtd.iPrevIntra4x4PredMode[i] = 0;
        } else {
          rtd.iPrevIntra4x4PredMode[i] = res.first;
        }
      }
      for (int i = 0; i < 4; i++) {
        res = iMovie().tag(PIP_PRED_MODE_TAG).scanBits(8);
        if (res.second) {
          fprintf(stderr, "failed to read remPredMode!\n");
          rtd.iRemIntra4x4PredMode[i] = 0;
        } else {
          rtd.iRemIntra4x4PredMode[i] = res.first;
        }
      }
      for (int i = 0; i < 4; i++) {
        res = iMovie().tag(PIP_SUB_MB_TAG).scanBits(8);
        if (res.second) {
          fprintf(stderr, "failed to read uiSubMbType!\n");
          rtd.uiSubMbType[i] = 0;
        } else {
          rtd.uiSubMbType[i] = res.first;
        }
      }
      for (int i = 0; i < 4; i++) {
        res = iMovie().tag(PIP_REF_TAG).scanBits(8);
        if (res.second) {
          fprintf(stderr, "failed to read iRefIdx!\n");
          rtd.iRefIdx[i] = 0;
        } else {
          rtd.iRefIdx[i] = res.first;
        }
      }
    } else if (MB_TYPE_8x8 == rtd.uiMbType) {
      for (int i = 0; i < 4; i++) {
        res = iMovie().tag(PIP_SUB_MB_TAG).scanBits(8);
        if (res.second) {
          fprintf(stderr, "failed to read uiSubMbType!\n");
          rtd.uiSubMbType[i] = 0;
        } else {
          rtd.uiSubMbType[i] = res.first;
        }
      }
      for (int i = 0; i < 4; i++) {
        res = iMovie().tag(PIP_REF_TAG).scanBits(8);
        if (res.second) {
          fprintf(stderr, "failed to read iRefIdx!\n");
          rtd.iRefIdx[i] = 0;
        } else {
          rtd.iRefIdx[i] = res.first;
        }
      }
    } else if (MB_TYPE_8x8_REF0 == rtd.uiMbType) {
      for (int i = 0; i < 4; i++) {
        res = iMovie().tag(PIP_SUB_MB_TAG).scanBits(8);
        if (res.second) {
          fprintf(stderr, "failed to read uiSubMbType!\n");
          rtd.uiSubMbType[i] = 0;
        } else {
          rtd.uiSubMbType[i] = res.first;
        }
      }
    } else if (MB_TYPE_8x16 == rtd.uiMbType ||
               MB_TYPE_16x8 == rtd.uiMbType) {
      for (int i = 0; i < 2; i++) {
        res = iMovie().tag(PIP_REF_TAG).scanBits(8);
        if (res.second) {
          fprintf(stderr, "failed to read iRefIdx!\n");
          rtd.iRefIdx[i] = 0;
        } else {
          rtd.iRefIdx[i] = res.first;
        }
      }
    } else if (MB_TYPE_16x16 == rtd.uiMbType) {
      res = iMovie().tag(PIP_REF_TAG).scanBits(8);
      if (res.second) {
        fprintf(stderr, "failed to read iRefIdx!\n");
        rtd.iRefIdx[0] = 0;
      } else {
        rtd.iRefIdx[0] = res.first;
      }
    }
    if (MB_TYPE_INTRA16x16 != rtd.uiMbType &&
        MB_TYPE_INTRA8x8 != rtd.uiMbType &&
        MB_TYPE_INTRA4x4 != rtd.uiMbType) {
      for (int i = 0; i < 16; i++) {
        res = iMovie().tag(PIP_MVX_TAG).scanBits(16);
        if (res.second) {
          fprintf(stderr, "failed to read mv x!\n");
        }
        rtd.sMbMvp[i][0] = res.first;
        res = iMovie().tag(PIP_MVY_TAG).scanBits(16);
        if (res.second) {
          fprintf(stderr, "failed to read mv y!\n");
        }
        rtd.sMbMvp[i][1] = res.first;
      }
    }
    bool scanned_luma_dc = false;
    bool scanned_chroma_dc = false;
    if (MB_TYPE_INTRA16x16 == rtd.uiMbType) {
      scanned_luma_dc = true;
      for (int i = 0; i < 16; i++) {
        res = iMovie().tag(PIP_LDC_TAG).scanBits(16);
        if (res.second) {
          fprintf(stderr, "failed to read ldc!\n");
        }
        rtd.odata.lumaDC[i] = res.first;
      }
    }
    if (1 == rtd.uiCbpC || 2 == rtd.uiCbpC) {
      scanned_chroma_dc = true;
      for (int i = 0; i < 8; i++) {
        res = iMovie().tag(PIP_CRDC_TAG).scanBits(16);
        if (res.second) {
          fprintf(stderr, "failed to read crdc!\n");
        }
        rtd.odata.chromaDC[i] = res.first;
      }
    }
    if (rtd.uiCbpL) {
      for (int i = 0; i < 16; i++) {
        decode4x4(&rtd.odata.lumaAC[i * 16], i, !scanned_luma_dc, 0);
      }
    }
    if (rtd.uiCbpC == 2) {
      for (int i = 0; i < 8; i++) {
        decode4x4(&rtd.odata.chromaAC[i * 16], (i & 3), !scanned_chroma_dc, i < 4 ? 1 : 2);
      }
    }

#ifdef DEBUG_PRINTS
    fprintf(stderr, "block %d uiCbpC=%d, L=%d\n", which_block, (int)rtd.uiCbpC, (int)rtd.uiCbpL);
    fprintf(stderr, "all done!\n");
#endif
  }
#ifdef DEBUG_PRINTS
  fprintf(stderr, "READY skiprun=%d ; origSkip%d ; which_block=%d curSkip=%d\n", rtd.iMbSkipRun, origSkipped, (int)(uint16_t)which_block, (int)curSkipped);
#endif
  // Some state is duplicated in rtd an odata. Just set both for now.
#ifdef CABAC_HACK
  static EncoderState esCabac(100000000);
  static bool firstTime = true;
  if (firstTime) {
    firstTime = false;
    WelsEnc::WelsCabacInit (&esCabac.pEncCtx);
    WelsEnc::WelsInitSliceCabac (&esCabac.pEncCtx, &esCabac.pSlice);
  }
  esCabac.init(&rtd);
  esCabac.setupCoefficientsFromOdata(rtd.odata);
  esCabac.initNonZeroCount(pCurLayer, rtd.odata);
  WelsEnc::WelsSpatialWriteMbSynCabac (
                                       &esCabac.pEncCtx, &esCabac.pSlice, &esCabac.pCurMb);
  fprintf(stderr, "block %d: Cabac size: %ld\n", which_block,
          esCabac.pSlice.sCabacCtx.m_pBufCur - esCabac.pSlice.sCabacCtx.m_pBufStart);
#endif
  EncoderState es;
  rtd.iMbSkipRun = origSkipped;
  es.init(&rtd);
  es.setupCoefficientsFromOdata(rtd.odata);
  es.initNonZeroCount(pCurLayer, rtd.odata);
  WelsEnc::WelsSpatialWriteMbSyn (
                                  &es.pEncCtx, &es.pSlice, &es.pCurMb);

  // Copied from svc_encode_slive.cpp:1023 . Have my doubts about CABAC support here.
  if (curSkipped == 1 && endOfSlice) {
    // If we end a slice with a skip, we need to output the first part of
    // a macroblock containing the length of the skip.
#ifdef BILLING
    curBillTag = PIP_SKIP_TAG;
#endif
    BsWriteUE (&es.wrBs, origSkipped);
  }
  EmitDefBitsToOMovie emission;
  copySBitStringAux(es.wrBs, emission);

#ifdef ROUNDTRIP_TEST
  SBitStringAux copyOfFirstEncodeWrbs = es.wrBs;
#endif

  PBitStringAux pBsOrig = pCurLayer->pBitStringAux;
  // We always will need at least one stop bit.
  // This bit value is not actually read, just used to compare the
  // size of the input bitstring to determine the uiEosFlag.
#ifdef BILLING
  curBillTag = PIP_SKIP_END_TAG;
#endif
  BsWriteOneBit (&es.wrBs, 0);
  if (!endOfSlice) {
    // This extra bit tells the deoder that there is at least one more
    // bit before the stop bit, hence not end of slice.
#ifdef BILLING
    curBillTag = PIP_SKIP_END_TAG;
#endif
    BsWriteOneBit (&es.wrBs, 0);
  }
  size_t len = ((es.wrBs.pCurBuf - es.wrBs.pStartBuf) << 3) + (32 - es.wrBs.iLeftBits);
  es.wrBs.uiCurBits <<= es.wrBs.iLeftBits;
  for (int i = 24; i >= 0; i -= 8) {
    *es.wrBs.pCurBuf = (es.wrBs.uiCurBits >> i) & 0xff;
    es.wrBs.pCurBuf++;
  }
  // The encoder does not update some state, such as 'iBits' and leaves
  // some data in uiCurBits. We call DecInitBits to reinitialize all fields
  // but reuse the existing buffer.
  DecInitBits (&es.wrBs, es.wrBs.pStartBuf, len);
  pCurLayer->pBitStringAux = &es.wrBs;
  // We want the decoder to read at the same time the encoder reads.
  // By default, the encoder has 3 stats with skipping:
  // > 0: we have already decided to skip.
  // == 0: we are finished skipping but have already read skip.
  // == -1: we need to read the skip bits and then decide to skip.
  //
  // The decoder only reads the skip bit when iMbSkipRun==-1.
  // So, we only want to read the skip bits then.
  // We also force this to the correct value at the beginning
  // of a run so it never reads iMbSkipRun early.
  pSlice->iMbSkipRun = curSkipped > 0 ? curSkipped : -1;
  int32_t iRet = pDecMbFunc (pCtx,  pNalCur, uiEosFlag, &rtd);
#ifdef DEBUG_PRINTS
  fprintf(stderr, "EOSTEST which_block=%d origSkipped=%d skip=%d endofslice=%d uiEosFlag=%d\n", which_block, origSkipped, pSlice->iMbSkipRun, endOfSlice, uiEosFlag);
#endif
  pCurLayer->pBitStringAux = pBsOrig;
  pCurLayer->pMbRefConcealedFlag[iNextMbXyIndex] = pCtx->bMbRefConcealed;

#ifdef ROUNDTRIP_TEST
  {
    initRTDFromDecoderState(rtd, pCurLayer);
    rtd.iMbSkipRun = origSkipped;
    EncoderState es2;
    es2.init(&rtd);
    es2.setupCoefficientsFromOdata(rtd.odata);
    es2.initNonZeroCount(pCurLayer, rtd.odata);
    woffset = 0;
    WelsEnc::WelsSpatialWriteMbSyn (
                                    &es2.pEncCtx, &es2.pSlice, &es2.pCurMb);
    assert(stringBitCompare(&copyOfFirstEncodeWrbs, es2.wrBs, 0));
  }
#endif

  curSkipped--;
  if (iRet != ERR_NONE) {
    return iRet;
  }
}

int32_t WelsDecodeSlice (PWelsDecoderContext pCtx, bool bFirstSliceInLayer, PNalUnit pNalCur) {
  PDqLayer pCurLayer = pCtx->pCurDqLayer;
  PFmo pFmo = pCtx->pFmo;
  int32_t iRet;
  int32_t iNextMbXyIndex, iSliceIdc;

  PSlice pSlice = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeaderExt pSliceHeaderExt = &pSlice->sSliceHeaderExt;
  PSliceHeader pSliceHeader = &pSliceHeaderExt->sSliceHeader;
  int32_t iMbX, iMbY;
  const int32_t kiCountNumMb = pSliceHeader->pSps->uiTotalMbCount; //need to be correct when fmo or multi slice
  uint32_t uiEosFlag = 0;
  PWelsDecMbFunc pDecMbFunc;

  if (oMovie().isRecoding) {
    PBitStringAux pBs = pCurLayer->pBitStringAux;
    int iIndex = ((pBs->pCurBuf - pBs->pStartBuf) << 3) - (16 - pBs->iLeftBits);
    for (int i = 0; i < iIndex; i++) {
      int whichBit = i & 0x07;
      int x = (pBs->pStartBuf[i >> 3] >> (7 - whichBit)) & 0x01;
      oMovie().def().emitBit(x);
    }
  }

  pSlice->iTotalMbInCurSlice = 0; //initialize at the starting of slice decoding.

  if (pCtx->pPps->bEntropyCodingModeFlag) {
    if (pSlice->sSliceHeaderExt.bAdaptiveMotionPredFlag ||
        pSlice->sSliceHeaderExt.bAdaptiveBaseModeFlag ||
        pSlice->sSliceHeaderExt.bAdaptiveResidualPredFlag) {
      WelsLog (& (pCtx->sLogCtx), WELS_LOG_ERROR,
               "WelsDecodeSlice()::::ILP flag exist, not supported with CABAC enabled!");
      pCtx->iErrorCode |= dsBitstreamError;
      return dsBitstreamError;
    }
    if (P_SLICE == pSliceHeader->eSliceType)
      pDecMbFunc = WelsDecodeMbCabacPSlice;
    else //I_SLICE. B_SLICE not supported now
      pDecMbFunc = WelsDecodeMbCabacISlice;
  } else {
    if (P_SLICE == pSliceHeader->eSliceType) {
      pDecMbFunc = WelsDecodeMbCavlcPSlice;
    } else { //I_SLICE
      pDecMbFunc = WelsDecodeMbCavlcISlice;
    }
  }

  if (pSliceHeader->pPps->bConstainedIntraPredFlag) {
    pCtx->pFillInfoCacheIntraNxNFunc = WelsFillCacheConstrain1IntraNxN;
    pCtx->pMapNxNNeighToSampleFunc    = WelsMapNxNNeighToSampleConstrain1;
    pCtx->pMap16x16NeighToSampleFunc  = WelsMap16x16NeighToSampleConstrain1;
  } else {
    pCtx->pFillInfoCacheIntraNxNFunc = WelsFillCacheConstrain0IntraNxN;
    pCtx->pMapNxNNeighToSampleFunc    = WelsMapNxNNeighToSampleNormal;
    pCtx->pMap16x16NeighToSampleFunc  = WelsMap16x16NeighToSampleNormal;
  }

  pCtx->eSliceType = pSliceHeader->eSliceType;
  if (pCurLayer->sLayerInfo.pPps->bEntropyCodingModeFlag == 1) {
    int32_t iQp = pSlice->sSliceHeaderExt.sSliceHeader.iSliceQp;
    int32_t iCabacInitIdc = pSlice->sSliceHeaderExt.sSliceHeader.iCabacInitIdc;
    WelsCabacContextInit (pCtx, pSlice->eSliceType, iCabacInitIdc, iQp);
    //InitCabacCtx (pCtx->pCabacCtx, pSlice->eSliceType, iCabacInitIdc, iQp);
    pSlice->iLastDeltaQp = 0;
    WELS_READ_VERIFY (InitCabacDecEngineFromBS (pCtx->pCabacDecEngine, pCtx->pCurDqLayer->pBitStringAux));
  }
  //try to calculate  the dequant_coeff
  WelsCalcDeqCoeffScalingList (pCtx);

  iNextMbXyIndex = pSliceHeader->iFirstMbInSlice;
  iMbX = iNextMbXyIndex % pCurLayer->iMbWidth;
  iMbY = iNextMbXyIndex / pCurLayer->iMbWidth; // error is introduced by multiple slices case, 11/23/2009
  static FreqImage imageCache;
  imageCache.priorValid = true;
  imageCache.updateFrame(pCtx->iFrameNum);
  if ((int)imageCache.width != pCurLayer->iMbWidth
      ||(int)imageCache.height != pCurLayer->iMbHeight) {
      if (imageCache.width) {
          fprintf(stderr, "Warning suboptimal resolution switch %d %d -> %d %d, throwing out prior (maybe want to resize?)\n",
                  imageCache.width, imageCache.height, pCurLayer->iMbWidth, pCurLayer->iMbHeight);
      }
      imageCache.priorValid = false;// should we splay the data across and resize it?
      imageCache.width = pCurLayer->iMbWidth;
      imageCache.height = pCurLayer->iMbHeight;
      imageCache.frame[0].resize(imageCache.width * imageCache.height);
      imageCache.frame[1].resize(imageCache.width * imageCache.height);
  }
  pSlice->iMbSkipRun = -1;
  iSliceIdc = (pSliceHeader->iFirstMbInSlice << 7) + pCurLayer->uiLayerDqId;
  static int slice_group = 0;
  ++slice_group;
  pCurLayer->iMbX =  iMbX;
  pCurLayer->iMbY = iMbY;
  pCurLayer->iMbXyIndex = iNextMbXyIndex;
  int origSkipped = -1;
  int curSkipped = -1;
  do {
    if ((-1 == iNextMbXyIndex) || (iNextMbXyIndex >= kiCountNumMb)) { // slice group boundary or end of a frame
      break;
    }
    static int which_block = 0;
    //Neighbors neighbors(&imageCache, iMbX, iMbY);
    pCurLayer->pSliceIdc[iNextMbXyIndex] = iSliceIdc;
    pCtx->bMbRefConcealed = false;
    DecodedMacroblock rtd;
    rtd.preInit(&pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer);
    oMovie().model().initCurrentMacroblock(&rtd, pCtx, &imageCache, iMbX, iMbY);
    woffset = 0;
    if (oMovie().isRecoding) {
      iRet = WelsDecodeSliceForRecoding(pCtx, pNalCur, pSlice,
                                        iNextMbXyIndex, uiEosFlag,
                                        rtd, pDecMbFunc, pCurLayer,
                                        origSkipped, curSkipped);
    } else {
      iRet = WelsDecodeSliceForNonRecoding(pCtx, pNalCur, pSlice,
                                           iNextMbXyIndex, uiEosFlag, rtd,
                                           pDecMbFunc, pCurLayer,
                                           origSkipped);
    }
    if (iRet != ERR_NONE) {
      return iRet;
    }

    {
      // save out dequantized values instead
      initCoeffsFromCoefPtr(rtd, pCurLayer->pScaledTCoeff[pCurLayer->iMbXyIndex]);
      imageCache.at(iMbX, iMbY) = rtd;
    }
    ++which_block;
    ++pSlice->iTotalMbInCurSlice;
    if (uiEosFlag) { //end of slice
      break;
    }
    if (pSliceHeader->pPps->uiNumSliceGroups > 1) {
      iNextMbXyIndex = FmoNextMb (pFmo, iNextMbXyIndex);
    } else {
      ++iNextMbXyIndex;
    }
    iMbX = iNextMbXyIndex % pCurLayer->iMbWidth;
    iMbY = iNextMbXyIndex / pCurLayer->iMbWidth;
    pCurLayer->iMbX =  iMbX;
    pCurLayer->iMbY = iMbY;
    pCurLayer->iMbXyIndex = iNextMbXyIndex;
  } while (1);
  return ERR_NONE;
}

int32_t WelsActualDecodeMbCavlcISlice (PWelsDecoderContext pCtx, PNalUnit pNalCur, DecodedMacroblock *rtd) {
  SVlcTable* pVlcTable     = &pCtx->sVlcTable;
  PDqLayer pCurLayer             = pCtx->pCurDqLayer;
  PBitStringAux pBs              = pCurLayer->pBitStringAux;
  PSlice pSlice                  = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeader pSliceHeader      = &pSlice->sSliceHeaderExt.sSliceHeader;

  SWelsNeighAvail sNeighAvail;
  int32_t iMbResProperty;

  int32_t iScanIdxStart = pSlice->sSliceHeaderExt.uiScanIdxStart;
  int32_t iScanIdxEnd   = pSlice->sSliceHeaderExt.uiScanIdxEnd;

  int32_t iMbX = pCurLayer->iMbX;
  int32_t iMbY = pCurLayer->iMbY;
  const int32_t iMbXy = pCurLayer->iMbXyIndex;
  int8_t* pNzc = pCurLayer->pNzc[iMbXy];
  int32_t i;
  uint32_t uiMbType = 0, uiCbp = 0, uiCbpL = 0, uiCbpC = 0;
  uint32_t uiCode;
  int32_t iCode;

  ENFORCE_STACK_ALIGN_1D (uint8_t, pNonZeroCount, 48, 16);
  {
  GetNeighborAvailMbType (&sNeighAvail, pCurLayer);
  pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;
  pCurLayer->pResidualPredFlag[iMbXy] = pSlice->sSliceHeaderExt.bDefaultResidualPredFlag;

  pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
  pCurLayer->pTransformSize8x8Flag[iMbXy] = false;
#ifdef BILLING
  curBillTag = PIP_MB_TYPE_TAG;
#endif
  WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //uiMbType
  uiMbType = uiCode;
  if (uiMbType > 25)
    return ERR_INFO_INVALID_MB_TYPE;
  if (!pCtx->pSps->uiChromaFormatIdc && ((uiMbType >= 5 && uiMbType <= 12) || (uiMbType >= 17 && uiMbType <= 24)))
    return ERR_INFO_INVALID_MB_TYPE;

  if (25 == uiMbType) {
    int32_t iDecStrideL = pCurLayer->pDec->iLinesize[0];
    int32_t iDecStrideC = pCurLayer->pDec->iLinesize[1];

    int32_t iOffsetL = (iMbX + iMbY * iDecStrideL) << 4;
    int32_t iOffsetC = (iMbX + iMbY * iDecStrideC) << 3;

    uint8_t* pDecY = pCurLayer->pDec->pData[0] + iOffsetL;
    uint8_t* pDecU = pCurLayer->pDec->pData[1] + iOffsetC;
    uint8_t* pDecV = pCurLayer->pDec->pData[2] + iOffsetC;

    uint8_t* pTmpBsBuf;


    int32_t i;
    int32_t iCopySizeY  = (sizeof (uint8_t) << 4);
    int32_t iCopySizeUV = (sizeof (uint8_t) << 3);

    int32_t iIndex = ((-pBs->iLeftBits) >> 3) + 2;

    pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA_PCM;

    //step 1: locating bit-stream pointer [must align into integer byte]
    pBs->pCurBuf -= iIndex;

    //step 2: copy pixel from bit-stream into fdec [reconstruction]
    pTmpBsBuf = pBs->pCurBuf;
    for (i = 0; i < 16; i++) { //luma
      memcpy (pDecY , pTmpBsBuf, iCopySizeY);
      pDecY += iDecStrideL;
      pTmpBsBuf += 16;
    }
    for (i = 0; i < 8; i++) { //cb
      memcpy (pDecU, pTmpBsBuf, iCopySizeUV);
      pDecU += iDecStrideC;
      pTmpBsBuf += 8;
    }
    for (i = 0; i < 8; i++) { //cr
      memcpy (pDecV, pTmpBsBuf, iCopySizeUV);
      pDecV += iDecStrideC;
      pTmpBsBuf += 8;
    }

    pBs->pCurBuf += 384;

    //step 3: update QP and pNonZeroCount
    pCurLayer->pLumaQp[iMbXy] = 0;
    memset (pCurLayer->pChromaQp[iMbXy], 0, sizeof (pCurLayer->pChromaQp[iMbXy]));
    memset (pNzc, 16, sizeof (pCurLayer->pNzc[iMbXy]));   //Rec. 9.2.1 for PCM, nzc=16
    WELS_READ_VERIFY (InitReadBits (pBs, 0));
    return 0;
  } else if (0 == uiMbType) { //reference to JM
    ENFORCE_STACK_ALIGN_1D (int8_t, pIntraPredMode, 48, 16);
    pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA4x4;
    if (pCtx->pPps->bTransform8x8ModeFlag) {
#ifdef BILLING
      curBillTag = PIP_8x8_TAG;
#endif
      WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode)); //transform_size_8x8_flag
      pCurLayer->pTransformSize8x8Flag[iMbXy] = !!uiCode;
      if (pCurLayer->pTransformSize8x8Flag[iMbXy]) {
        uiMbType = pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA8x8;
      }
    }
    if (!pCurLayer->pTransformSize8x8Flag[iMbXy]) {
      pCtx->pFillInfoCacheIntraNxNFunc (&sNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
      WELS_READ_VERIFY (ParseIntra4x4Mode (pCtx, &sNeighAvail, pIntraPredMode, pBs, pCurLayer, rtd));
    } else {
      pCtx->pFillInfoCacheIntraNxNFunc (&sNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
      WELS_READ_VERIFY (ParseIntra8x8Mode (pCtx, &sNeighAvail, pIntraPredMode, pBs, pCurLayer, rtd));
    }

    //uiCbp
#ifdef BILLING
      curBillTag = PIP_16x16_TAG;
#endif
    WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //coded_block_pattern
    uiCbp = uiCode;
    //G.9.1 Alternative parsing process for coded pBlock pattern
    if (pCtx->pSps->uiChromaFormatIdc && (uiCbp > 47))
      return ERR_INFO_INVALID_CBP;
    if (!pCtx->pSps->uiChromaFormatIdc && (uiCbp > 15))
      return ERR_INFO_INVALID_CBP;

    if (pCtx->pSps->uiChromaFormatIdc)
      uiCbp = g_kuiIntra4x4CbpTable[uiCbp];
    else
      uiCbp = g_kuiIntra4x4CbpTable400[uiCbp];
    pCurLayer->pCbp[iMbXy] = uiCbp;
    uiCbpC = uiCbp >> 4;
    uiCbpL = uiCbp & 15;
    rtd->uiCbpC = uiCbpC;
    rtd->uiCbpL = uiCbpL;
  } else { //I_PCM exclude, we can ignore it
    pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA16x16;
    pCurLayer->pTransformSize8x8Flag[iMbXy] = false;
    pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
    pCurLayer->pIntraPredMode[iMbXy][7] = (uiMbType - 1) & 3;
    pCurLayer->pCbp[iMbXy] = g_kuiI16CbpTable[ (uiMbType - 1) >> 2];
    uiCbpC = pCtx->pSps->uiChromaFormatIdc ? pCurLayer->pCbp[iMbXy] >> 4 : 0;
    uiCbpL = pCurLayer->pCbp[iMbXy] & 15;
    rtd->uiCbpC = uiCbpC;
    rtd->uiCbpL = uiCbpL;
    WelsFillCacheNonZeroCount (&sNeighAvail, pNonZeroCount, pCurLayer);
    WELS_READ_VERIFY (ParseIntra16x16Mode (pCtx, &sNeighAvail, pBs, pCurLayer));
  }

  ST32A4 (&pNzc[0], 0);
  ST32A4 (&pNzc[4], 0);
  ST32A4 (&pNzc[8], 0);
  ST32A4 (&pNzc[12], 0);
  ST32A4 (&pNzc[16], 0);
  ST32A4 (&pNzc[20], 0);

  if (pCurLayer->pCbp[iMbXy] == 0 && IS_INTRA4x4 (pCurLayer->pMbType[iMbXy])) {
    pCurLayer->pLumaQp[iMbXy] = pSlice->iLastMbQp;
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pCurLayer->pLumaQp[iMbXy] +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i], 0, 51)];
    }

  }

  if (pCurLayer->pCbp[iMbXy] || MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
    memset (pCurLayer->pScaledTCoeff[iMbXy], 0, 384 * sizeof (pCurLayer->pScaledTCoeff[iMbXy][0]));
    memset (pCurLayer->pScaledTCoeffQuant[iMbXy], 0, 384 * sizeof (pCurLayer->pScaledTCoeffQuant[iMbXy][0]));
    int32_t iQpDelta, iId8x8, iId4x4;
#ifdef BILLING
      curBillTag = PIP_QPL_TAG;
#endif

    WELS_READ_VERIFY (BsGetSe (pBs, &iCode)); //mb_qp_delta
    iQpDelta = iCode;

    if (iQpDelta > 25 || iQpDelta < -26) { //out of iQpDelta range
      return ERR_INFO_INVALID_QP;
    }

    pCurLayer->pLumaQp[iMbXy] = (pSlice->iLastMbQp + iQpDelta + 52) % 52; //update last_mb_qp
    pSlice->iLastMbQp = pCurLayer->pLumaQp[iMbXy];
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pSlice->iLastMbQp +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i], 0,
                                       51)];
    }

    BsStartCavlc (pBs);

    if (MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
      //step1: Luma DC
      if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, 0, 16,
                                  g_kuiLumaDcZigzagScan, I16_LUMA_DC, pCurLayer->pScaledTCoeff[iMbXy], pCurLayer->pScaledTCoeffQuant[iMbXy], pCurLayer->pLumaQp[iMbXy], pCtx)) {
        return -1;//abnormal
      }
      //step2: Luma AC
      if (uiCbpL) {
        for (i = 0; i < 16; i++) {
          if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, i,
                                      iScanIdxEnd - WELS_MAX (iScanIdxStart, 1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1),
                                      I16_LUMA_AC, pCurLayer->pScaledTCoeff[iMbXy] + (i << 4), pCurLayer->pScaledTCoeffQuant[iMbXy] + (i << 4), pCurLayer->pLumaQp[iMbXy], pCtx)) {
            return -1;//abnormal
          }
        }
        ST32A4 (&pNzc[0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32A4 (&pNzc[4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32A4 (&pNzc[8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32A4 (&pNzc[12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      }
    } else { //non-MB_TYPE_INTRA16x16
      if (pCurLayer->pTransformSize8x8Flag[iMbXy]) {
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          iMbResProperty = (IS_INTRA (pCurLayer->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA : LUMA_DC_AC_INTER;
          if (uiCbpL & (1 << iId8x8)) {
            int32_t iIndex = (iId8x8 << 2);
            for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
              if (WelsResidualBlockCavlc8x8 (pVlcTable, pNonZeroCount, pBs, iIndex,
                                             iScanIdxEnd - iScanIdxStart + 1, g_kuiZigzagScan8x8 + iScanIdxStart, iMbResProperty,
                                             pCurLayer->pScaledTCoeff[iMbXy] + (iId8x8 << 6), pCurLayer->pScaledTCoeffQuant[iMbXy] + (iId8x8 << 6), iId4x4, pCurLayer->pLumaQp[iMbXy], pCtx)) {
                return -1;
              }
              iIndex++;
            }
          } else {
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[iId8x8 << 2]], 0);
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32A4 (&pNzc[0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32A4 (&pNzc[4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32A4 (&pNzc[8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32A4 (&pNzc[12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      } else {
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          if (uiCbpL & (1 << iId8x8)) {
            int32_t iIndex = (iId8x8 << 2);
            for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
              //Luma (DC and AC decoding together)
              if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, iIndex,
                                          iScanIdxEnd - iScanIdxStart + 1, g_kuiZigzagScan + iScanIdxStart,
                                          LUMA_DC_AC_INTRA, pCurLayer->pScaledTCoeff[iMbXy] + (iIndex << 4), pCurLayer->pScaledTCoeffQuant[iMbXy] + (iIndex << 4), pCurLayer->pLumaQp[iMbXy], pCtx)) {
                return -1;//abnormal
              }
              iIndex++;
            }
          } else {
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[ (iId8x8 << 2)]], 0);
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32A4 (&pNzc[0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32A4 (&pNzc[4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32A4 (&pNzc[8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32A4 (&pNzc[12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      }
    }

    //chroma
    //step1: DC
    if (1 == uiCbpC || 2 == uiCbpC) {
      for (i = 0; i < 2; i++) { //Cb Cr
        iMbResProperty = i ? CHROMA_DC_V : CHROMA_DC_U;
        if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs,
                                    16 + (i << 2), 4, g_kuiChromaDcScan, iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + 256 + (i << 6), pCurLayer->pScaledTCoeffQuant[iMbXy] + 256 + (i << 6),
                                    pCurLayer->pChromaQp[iMbXy][i], pCtx)) {
          return -1;//abnormal
        }
      }
    } else if (uiCbpC != 0) {
      // FIXME(patrick): Encoder does not check for this condition...
      fprintf(stderr, "uiCbpC is %d not 1 or 2. Encoder might expect DC!\n",
              uiCbpC);
    }

    //step2: AC
    if (2 == uiCbpC) {
      for (i = 0; i < 2; i++) { //Cb Cr
        iMbResProperty = i ? CHROMA_AC_V : CHROMA_AC_U;
        int32_t iIndex = 16 + (i << 2);
        for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
          if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, iIndex,
                                      iScanIdxEnd - WELS_MAX (iScanIdxStart, 1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1),
                                      iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + (iIndex << 4), pCurLayer->pScaledTCoeffQuant[iMbXy] + (iIndex << 4), pCurLayer->pChromaQp[iMbXy][i], pCtx)) {
            return -1;//abnormal
          }
          iIndex++;
        }
      }
      ST16A2 (&pNzc[16], LD16A2 (&pNonZeroCount[6 + 8 * 1]));
      ST16A2 (&pNzc[20], LD16A2 (&pNonZeroCount[6 + 8 * 2]));
      ST16A2 (&pNzc[18], LD16A2 (&pNonZeroCount[6 + 8 * 4]));
      ST16A2 (&pNzc[22], LD16A2 (&pNonZeroCount[6 + 8 * 5]));
    }
    BsEndCavlc (pBs);
  }

  } // fixme


  return 0;
}

int32_t WelsDecodeMbCavlcISlice (PWelsDecoderContext pCtx, PNalUnit pNalCur, uint32_t& uiEosFlag, DecodedMacroblock *rtd) {
  PDqLayer pCurLayer = pCtx->pCurDqLayer;
  PBitStringAux pBs = pCurLayer->pBitStringAux;
  PSliceHeaderExt pSliceHeaderExt = &pCurLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt;
  int32_t iBaseModeFlag;
  int32_t iRet = 0; //should have the return value to indicate decoding error or not, It's NECESSARY--2010.4.15
  uint32_t uiCode;
  intX_t iUsedBits;
  if (pSliceHeaderExt->bAdaptiveBaseModeFlag == 1) {
    WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode)); //base_mode_flag
    iBaseModeFlag = uiCode;
  } else {
    iBaseModeFlag = pSliceHeaderExt->bDefaultBaseModeFlag;
  }
  if (!iBaseModeFlag) {
      iRet = WelsActualDecodeMbCavlcISlice (pCtx, pNalCur, rtd);
  } else {
    WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING, "iBaseModeFlag (%d) != 0, inter-layer prediction not supported.",
             iBaseModeFlag);
    return GENERATE_ERROR_NO (ERR_LEVEL_SLICE_HEADER, ERR_INFO_UNSUPPORTED_ILP);
  }
  if (iRet) { //occur error when parsing, MUST STOP decoding
    return iRet;
  }

  // check whether there is left bits to read next time in case multiple slices
  iUsedBits = ((pBs->pCurBuf - pBs->pStartBuf) << 3) - (16 - pBs->iLeftBits);
  // sub 1, for stop bit
  if ((iUsedBits == (pBs->iBits - 1)) && (0 >= pCurLayer->sLayerInfo.sSliceInLayer.iMbSkipRun)) { // slice boundary
    uiEosFlag = 1;
  }
  if (iUsedBits > (pBs->iBits -
                   1)) { //When BS incomplete, as long as find it, SHOULD stop decoding to avoid mosaic or crash.
    WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING,
             "WelsDecodeMbCavlcISlice()::::pBs incomplete, iUsedBits:%" PRId64 " > pBs->iBits:%d, MUST stop decoding.",
             (int64_t) iUsedBits, pBs->iBits);
    return -1;
  }
  return 0;
}

int32_t WelsActualDecodeMbCavlcPSlice (PWelsDecoderContext pCtx, DecodedMacroblock *rtd) {
  SVlcTable* pVlcTable     = &pCtx->sVlcTable;
  PDqLayer pCurLayer             = pCtx->pCurDqLayer;
  PBitStringAux pBs              = pCurLayer->pBitStringAux;
  PSlice pSlice                  = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeader pSliceHeader      = &pSlice->sSliceHeaderExt.sSliceHeader;

  int32_t iScanIdxStart = pSlice->sSliceHeaderExt.uiScanIdxStart;
  int32_t iScanIdxEnd   = pSlice->sSliceHeaderExt.uiScanIdxEnd;

  SWelsNeighAvail sNeighAvail;
  int32_t iMbX = pCurLayer->iMbX;
  int32_t iMbY = pCurLayer->iMbY;
  const int32_t iMbXy = pCurLayer->iMbXyIndex;
  int8_t* pNzc = pCurLayer->pNzc[iMbXy];
  int32_t i;
  uint32_t uiMbType = 0, uiCbp = 0, uiCbpL = 0, uiCbpC = 0;
  uint32_t uiCode;
  int32_t iCode;
  int32_t iMbResProperty;

  GetNeighborAvailMbType (&sNeighAvail, pCurLayer);
  ENFORCE_STACK_ALIGN_1D (uint8_t, pNonZeroCount, 48, 16);
  {
  pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;//2009.10.23
#ifdef BILLING
      curBillTag = PIP_MB_TYPE_TAG;
#endif

  WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //uiMbType
  uiMbType = uiCode;
  if (uiMbType < 5) { //inter MB type
    int16_t iMotionVector[LIST_A][30][MV_A];
    int8_t  iRefIndex[LIST_A][30];
    pCurLayer->pMbType[iMbXy] = g_ksInterMbTypeInfo[uiMbType].iType;
    WelsFillCacheInter (&sNeighAvail, pNonZeroCount, iMotionVector, iRefIndex, pCurLayer);

    if (ParseInterInfo (pCtx, iMotionVector, iRefIndex, pBs, rtd)) {
      return -1;//abnormal
    }

    if (pSlice->sSliceHeaderExt.bAdaptiveResidualPredFlag == 1) {
#ifdef BILLING
      curBillTag = PIP_PRED_TAG;
#endif
      WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode)); //residual_prediction_flag
      pCurLayer->pResidualPredFlag[iMbXy] =  uiCode;
    } else {
      pCurLayer->pResidualPredFlag[iMbXy] = pSlice->sSliceHeaderExt.bDefaultResidualPredFlag;
    }

    if (pCurLayer->pResidualPredFlag[iMbXy] == 0) {
      pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;
    } else {
      WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING, "residual_pred_flag = 1 not supported.");
      return -1;
    }
  } else { //intra MB type
    uiMbType -= 5;
    if (uiMbType > 25)
      return ERR_INFO_INVALID_MB_TYPE;
    if (!pCtx->pSps->uiChromaFormatIdc && ((uiMbType >= 5 && uiMbType <= 12) || (uiMbType >= 17 && uiMbType <= 24)))
      return ERR_INFO_INVALID_MB_TYPE;

    if (25 == uiMbType) {
      int32_t iDecStrideL = pCurLayer->pDec->iLinesize[0];
      int32_t iDecStrideC = pCurLayer->pDec->iLinesize[1];

      int32_t iOffsetL = (iMbX + iMbY * iDecStrideL) << 4;
      int32_t iOffsetC = (iMbX + iMbY * iDecStrideC) << 3;

      uint8_t* pDecY = pCurLayer->pDec->pData[0] + iOffsetL;
      uint8_t* pDecU = pCurLayer->pDec->pData[1] + iOffsetC;
      uint8_t* pDecV = pCurLayer->pDec->pData[2] + iOffsetC;

      uint8_t* pTmpBsBuf;

      int32_t i;
      int32_t iCopySizeY  = (sizeof (uint8_t) << 4);
      int32_t iCopySizeUV = (sizeof (uint8_t) << 3);

      int32_t iIndex = ((-pBs->iLeftBits) >> 3) + 2;

      pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA_PCM;

      //step 1: locating bit-stream pointer [must align into integer byte]
      pBs->pCurBuf -= iIndex;

      //step 2: copy pixel from bit-stream into fdec [reconstruction]
      pTmpBsBuf = pBs->pCurBuf;
      for (i = 0; i < 16; i++) { //luma
        memcpy (pDecY , pTmpBsBuf, iCopySizeY);
        pDecY += iDecStrideL;
        pTmpBsBuf += 16;
      }

      for (i = 0; i < 8; i++) { //cb
        memcpy (pDecU, pTmpBsBuf, iCopySizeUV);
        pDecU += iDecStrideC;
        pTmpBsBuf += 8;
      }
      for (i = 0; i < 8; i++) { //cr
        memcpy (pDecV, pTmpBsBuf, iCopySizeUV);
        pDecV += iDecStrideC;
        pTmpBsBuf += 8;
      }

      pBs->pCurBuf += 384;

      //step 3: update QP and pNonZeroCount
      pCurLayer->pLumaQp[iMbXy] = 0;
      pCurLayer->pChromaQp[iMbXy][0] = pCurLayer->pChromaQp[iMbXy][1] = 0;
      //Rec. 9.2.1 for PCM, nzc=16
      ST32A4 (&pNzc[0], 0x10101010);
      ST32A4 (&pNzc[4], 0x10101010);
      ST32A4 (&pNzc[8], 0x10101010);
      ST32A4 (&pNzc[12], 0x10101010);
      ST32A4 (&pNzc[16], 0x10101010);
      ST32A4 (&pNzc[20], 0x10101010);
      WELS_READ_VERIFY (InitReadBits (pBs, 0));
      return 0;
    } else {
      if (0 == uiMbType) {
        ENFORCE_STACK_ALIGN_1D (int8_t, pIntraPredMode, 48, 16);
        pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA4x4;
        if (pCtx->pPps->bTransform8x8ModeFlag) {
#ifdef BILLING
            curBillTag = PIP_8x8_TAG;
#endif
          WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode)); //transform_size_8x8_flag
          pCurLayer->pTransformSize8x8Flag[iMbXy] = !!uiCode;
          if (pCurLayer->pTransformSize8x8Flag[iMbXy]) {
            uiMbType = pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA8x8;
          }
        }
        if (!pCurLayer->pTransformSize8x8Flag[iMbXy]) {
          pCtx->pFillInfoCacheIntraNxNFunc (&sNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
          WELS_READ_VERIFY (ParseIntra4x4Mode (pCtx, &sNeighAvail, pIntraPredMode, pBs, pCurLayer, rtd));
        } else {
          pCtx->pFillInfoCacheIntraNxNFunc (&sNeighAvail, pNonZeroCount, pIntraPredMode, pCurLayer);
          WELS_READ_VERIFY (ParseIntra8x8Mode (pCtx, &sNeighAvail, pIntraPredMode, pBs, pCurLayer, rtd));
        }
      } else { //I_PCM exclude, we can ignore it
        pCurLayer->pMbType[iMbXy] = MB_TYPE_INTRA16x16;
        pCurLayer->pTransformSize8x8Flag[iMbXy] = false;
        pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
        pCurLayer->pIntraPredMode[iMbXy][7] = (uiMbType - 1) & 3;
        pCurLayer->pCbp[iMbXy] = g_kuiI16CbpTable[ (uiMbType - 1) >> 2];
        uiCbpC = pCtx->pSps->uiChromaFormatIdc ? pCurLayer->pCbp[iMbXy] >> 4 : 0;
        uiCbpL = pCurLayer->pCbp[iMbXy] & 15;
        rtd->uiCbpC = uiCbpC;
        rtd->uiCbpL = uiCbpL;
        WelsFillCacheNonZeroCount (&sNeighAvail, pNonZeroCount, pCurLayer);
        if (ParseIntra16x16Mode (pCtx, &sNeighAvail, pBs, pCurLayer)) {
          return -1;
        }
      }
    }
  }

  if (MB_TYPE_INTRA16x16 != pCurLayer->pMbType[iMbXy]) {
#ifdef BILLING
      curBillTag = PIP_16x16_TAG;
#endif
    WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //coded_block_pattern
    uiCbp = uiCode;
    {
      if (pCtx->pSps->uiChromaFormatIdc && (uiCbp > 47))
        return ERR_INFO_INVALID_CBP;
      if (!pCtx->pSps->uiChromaFormatIdc && (uiCbp > 15))
        return ERR_INFO_INVALID_CBP;
      if (MB_TYPE_INTRA4x4 == pCurLayer->pMbType[iMbXy] || MB_TYPE_INTRA8x8 == pCurLayer->pMbType[iMbXy]) {

        uiCbp = pCtx->pSps->uiChromaFormatIdc ? g_kuiIntra4x4CbpTable[uiCbp] : g_kuiIntra4x4CbpTable400[uiCbp];
      } else //inter
        uiCbp = pCtx->pSps->uiChromaFormatIdc ?  g_kuiInterCbpTable[uiCbp] : g_kuiInterCbpTable400[uiCbp];
    }

    pCurLayer->pCbp[iMbXy] = uiCbp;
    uiCbpC = pCurLayer->pCbp[iMbXy] >> 4;
    uiCbpL = pCurLayer->pCbp[iMbXy] & 15;
    rtd->uiCbpC = uiCbpC;
    rtd->uiCbpL = uiCbpL;

    // Need modification when B picutre add in
    bool bNeedParseTransformSize8x8Flag =
      (((pCurLayer->pMbType[iMbXy] >= MB_TYPE_16x16 && pCurLayer->pMbType[iMbXy] <= MB_TYPE_8x16)
        || pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy])
       && (pCurLayer->pMbType[iMbXy] != MB_TYPE_INTRA8x8)
       && (pCurLayer->pMbType[iMbXy] != MB_TYPE_INTRA4x4)
       && (uiCbpL > 0)
       && (pCtx->pPps->bTransform8x8ModeFlag));

    if (bNeedParseTransformSize8x8Flag) {
#ifdef BILLING
      curBillTag = PIP_8x8_TAG;
#endif
      WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode)); //transform_size_8x8_flag
      pCurLayer->pTransformSize8x8Flag[iMbXy] = !!uiCode;
    }
  }

  ST32A4 (&pNzc[0], 0);
  ST32A4 (&pNzc[4], 0);
  ST32A4 (&pNzc[8], 0);
  ST32A4 (&pNzc[12], 0);
  ST32A4 (&pNzc[16], 0);
  ST32A4 (&pNzc[20], 0);
  if (pCurLayer->pCbp[iMbXy] == 0 && !IS_INTRA16x16 (pCurLayer->pMbType[iMbXy]) && !IS_I_BL (pCurLayer->pMbType[iMbXy])) {
    pCurLayer->pLumaQp[iMbXy] = pSlice->iLastMbQp;
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pCurLayer->pLumaQp[iMbXy] +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i], 0, 51)];
    }
  }

  if (pCurLayer->pCbp[iMbXy] || MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
    int32_t iQpDelta, iId8x8, iId4x4;
    memset (pCurLayer->pScaledTCoeff[iMbXy], 0, MB_COEFF_LIST_SIZE * sizeof (int16_t));
    memset (pCurLayer->pScaledTCoeffQuant[iMbXy], 0, MB_COEFF_LIST_SIZE * sizeof (int16_t));
#ifdef BILLING
      curBillTag = PIP_16x16_TAG;
#endif
    WELS_READ_VERIFY (BsGetSe (pBs, &iCode)); //mb_qp_delta
    iQpDelta = iCode;

    if (iQpDelta > 25 || iQpDelta < -26) { //out of iQpDelta range
      return ERR_INFO_INVALID_QP;
    }

    pCurLayer->pLumaQp[iMbXy] = (pSlice->iLastMbQp + iQpDelta + 52) % 52; //update last_mb_qp
    pSlice->iLastMbQp = pCurLayer->pLumaQp[iMbXy];
    for (i = 0; i < 2; i++) {
      pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pSlice->iLastMbQp +
                                       pSliceHeader->pPps->iChromaQpIndexOffset[i], 0,
                                       51)];
    }

    BsStartCavlc (pBs);

    if (MB_TYPE_INTRA16x16 == pCurLayer->pMbType[iMbXy]) {
      //step1: Luma DC
      if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, 0, 16, g_kuiLumaDcZigzagScan,
                                  I16_LUMA_DC, pCurLayer->pScaledTCoeff[iMbXy], pCurLayer->pScaledTCoeffQuant[iMbXy], pCurLayer->pLumaQp[iMbXy], pCtx)) {
        return -1;//abnormal
      }
      //step2: Luma AC
      if (uiCbpL) {
        for (i = 0; i < 16; i++) {
          if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, i,
                                      iScanIdxEnd - WELS_MAX (iScanIdxStart, 1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1),
                                      I16_LUMA_AC, pCurLayer->pScaledTCoeff[iMbXy] + (i << 4), pCurLayer->pScaledTCoeffQuant[iMbXy] + (i << 4), pCurLayer->pLumaQp[iMbXy], pCtx)) {
            return -1;//abnormal
          }
        }
        ST32A4 (&pNzc[0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32A4 (&pNzc[4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32A4 (&pNzc[8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32A4 (&pNzc[12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      }
    } else { //non-MB_TYPE_INTRA16x16
      if (pCurLayer->pTransformSize8x8Flag[iMbXy]) {
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          iMbResProperty = (IS_INTRA (pCurLayer->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA : LUMA_DC_AC_INTER;
          if (uiCbpL & (1 << iId8x8)) {
            int32_t iIndex = (iId8x8 << 2);
            for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
              if (WelsResidualBlockCavlc8x8 (pVlcTable, pNonZeroCount, pBs, iIndex,
                                             iScanIdxEnd - iScanIdxStart + 1, g_kuiZigzagScan8x8 + iScanIdxStart, iMbResProperty,
                                             pCurLayer->pScaledTCoeff[iMbXy] + (iId8x8 << 6),pCurLayer->pScaledTCoeffQuant[iMbXy] + (iId8x8 << 6), iId4x4, pCurLayer->pLumaQp[iMbXy], pCtx)) {
                return -1;
              }
              iIndex++;
            }
          } else {
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[iId8x8 << 2]], 0);
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32A4 (&pNzc[0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32A4 (&pNzc[4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32A4 (&pNzc[8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32A4 (&pNzc[12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      } else { // Normal T4x4
        for (iId8x8 = 0; iId8x8 < 4; iId8x8++) {
          iMbResProperty = (IS_INTRA (pCurLayer->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA : LUMA_DC_AC_INTER;
          if (uiCbpL & (1 << iId8x8)) {
            int32_t iIndex = (iId8x8 << 2);
            for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
              //Luma (DC and AC decoding together)
              if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, iIndex,
                                          iScanIdxEnd - iScanIdxStart + 1, g_kuiZigzagScan + iScanIdxStart, iMbResProperty,
                                          pCurLayer->pScaledTCoeff[iMbXy] + (iIndex << 4),pCurLayer->pScaledTCoeffQuant[iMbXy] + (iIndex << 4), pCurLayer->pLumaQp[iMbXy], pCtx)) {
                return -1;//abnormal
              }
              iIndex++;
            }
          } else {
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[iId8x8 << 2]], 0);
            ST16 (&pNonZeroCount[g_kuiCache48CountScan4Idx[ (iId8x8 << 2) + 2]], 0);
          }
        }
        ST32A4 (&pNzc[0], LD32 (&pNonZeroCount[1 + 8 * 1]));
        ST32A4 (&pNzc[4], LD32 (&pNonZeroCount[1 + 8 * 2]));
        ST32A4 (&pNzc[8], LD32 (&pNonZeroCount[1 + 8 * 3]));
        ST32A4 (&pNzc[12], LD32 (&pNonZeroCount[1 + 8 * 4]));
      }
    }


    //chroma
    //step1: DC
    if (1 == uiCbpC || 2 == uiCbpC) {
      for (i = 0; i < 2; i++) { //Cb Cr
        if (IS_INTRA (pCurLayer->pMbType[iMbXy]))
          iMbResProperty = i ? CHROMA_DC_V : CHROMA_DC_U;
        else
          iMbResProperty = i ? CHROMA_DC_V_INTER : CHROMA_DC_U_INTER;

        if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs,
                                    16 + (i << 2), 4, g_kuiChromaDcScan, iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + 256 + (i << 6), pCurLayer->pScaledTCoeffQuant[iMbXy] + 256 + (i << 6),
                                    pCurLayer->pChromaQp[iMbXy][i], pCtx)) {
          return -1;//abnormal
        }
      }
    } else if (uiCbpC != 0) {
      // FIXME(patrick): Encoder does not check for this condition...
      fprintf(stderr, "uiCbpC is %d not 1 or 2. Encoder might expect DC!\n",
              uiCbpC);
    }
    //step2: AC
    if (2 == uiCbpC) {
      for (i = 0; i < 2; i++) { //Cb Cr
        if (IS_INTRA (pCurLayer->pMbType[iMbXy]))
          iMbResProperty = i ? CHROMA_AC_V : CHROMA_AC_U;
        else
          iMbResProperty = i ? CHROMA_AC_V_INTER : CHROMA_AC_U_INTER;

        int32_t iIndex = 16 + (i << 2);
        for (iId4x4 = 0; iId4x4 < 4; iId4x4++) {
          if (WelsResidualBlockCavlc (pVlcTable, pNonZeroCount, pBs, iIndex,
                                      iScanIdxEnd - WELS_MAX (iScanIdxStart, 1) + 1, g_kuiZigzagScan + WELS_MAX (iScanIdxStart, 1),
                                      iMbResProperty, pCurLayer->pScaledTCoeff[iMbXy] + (iIndex << 4), pCurLayer->pScaledTCoeffQuant[iMbXy] + (iIndex << 4), pCurLayer->pChromaQp[iMbXy][i], pCtx)) {
            return -1;//abnormal
          }
          iIndex++;
        }
      }
      ST16A2 (&pNzc[16], LD16A2 (&pNonZeroCount[6 + 8 * 1]));
      ST16A2 (&pNzc[20], LD16A2 (&pNonZeroCount[6 + 8 * 2]));
      ST16A2 (&pNzc[18], LD16A2 (&pNonZeroCount[6 + 8 * 4]));
      ST16A2 (&pNzc[22], LD16A2 (&pNonZeroCount[6 + 8 * 5]));
    }
    BsEndCavlc (pBs);
  }

  } // FIXME

  return 0;
}

int32_t WelsDecodeMbCavlcPSlice (PWelsDecoderContext pCtx, PNalUnit pNalCur, uint32_t& uiEosFlag, DecodedMacroblock *rtd) {
  PDqLayer pCurLayer             = pCtx->pCurDqLayer;
  PBitStringAux pBs              = pCurLayer->pBitStringAux;
  PSlice pSlice                  = &pCurLayer->sLayerInfo.sSliceInLayer;
  PSliceHeader pSliceHeader      = &pSlice->sSliceHeaderExt.sSliceHeader;
  PPicture* ppRefPic = pCtx->sRefPic.pRefList[LIST_0];
  intX_t iUsedBits;
  const int32_t iMbXy = pCurLayer->iMbXyIndex;
  int8_t* pNzc = pCurLayer->pNzc[iMbXy];
  int32_t iBaseModeFlag, i;
  int32_t iRet = 0; //should have the return value to indicate decoding error or not, It's NECESSARY--2010.4.15
  uint32_t uiCode;

  pCurLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy] = true;
  pCurLayer->pTransformSize8x8Flag[iMbXy] = false;
  if (-1 == pSlice->iMbSkipRun) {
#ifdef BILLING
      curBillTag = PIP_SKIP_TAG;
#endif
    WELS_READ_VERIFY (BsGetUe (pBs, &uiCode)); //mb_skip_run
    pSlice->iMbSkipRun = uiCode;
    if (-1 == pSlice->iMbSkipRun) {
      return -1;
    }
    if (oMovie().isRecoding) {
      // While recoding, we want to make sure that the reads of
      // iMbSkipRun are synced with writes from the encoder.
      // To do this, two things must happen:
      // We tell the decoder when we are currently skipping, and
      // at the end of a skip, we wish to merely consume this value
      // while ignoring the skip we already performed
      pSlice->iMbSkipRun = 0;
    }
  }
  rtd->iMbSkipRun = pSlice->iMbSkipRun;
  if (pSlice->iMbSkipRun--) {
    int16_t iMv[2];

    pCurLayer->pMbType[iMbXy] = MB_TYPE_SKIP;
    ST32A4 (&pNzc[0], 0);
    ST32A4 (&pNzc[4], 0);
    ST32A4 (&pNzc[8], 0);
    ST32A4 (&pNzc[12], 0);
    ST32A4 (&pNzc[16], 0);
    ST32A4 (&pNzc[20], 0);

    pCurLayer->pInterPredictionDoneFlag[iMbXy] = 0;
    memset (pCurLayer->pRefIndex[0][iMbXy], 0, sizeof (int8_t) * 16);
    pCtx->bMbRefConcealed = pCtx->bRPLRError || pCtx->bMbRefConcealed || ! (ppRefPic[0] && ppRefPic[0]->bIsComplete);
    //predict iMv
    PredPSkipMvFromNeighbor (pCurLayer, iMv);
    for (i = 0; i < 16; i++) {
      ST32A2 (pCurLayer->pMv[0][iMbXy][i], * (uint32_t*)iMv);
    }

    //if (!pSlice->sSliceHeaderExt.bDefaultResidualPredFlag) {
    //  memset (pCurLayer->pScaledTCoeff[iMbXy], 0, 384 * sizeof (int16_t));
    //}

    //reset rS
    if (!pSlice->sSliceHeaderExt.bDefaultResidualPredFlag ||
        (pNalCur->sNalHeaderExt.uiQualityId == 0 && pNalCur->sNalHeaderExt.uiDependencyId == 0)) {
      pCurLayer->pLumaQp[iMbXy] = pSlice->iLastMbQp;
      for (i = 0; i < 2; i++) {
        pCurLayer->pChromaQp[iMbXy][i] = g_kuiChromaQpTable[WELS_CLIP3 (pCurLayer->pLumaQp[iMbXy] +
                                         pSliceHeader->pPps->iChromaQpIndexOffset[i], 0, 51)];
      }
    }

    pCurLayer->pCbp[iMbXy] = 0;
  } else {
    if (pSlice->sSliceHeaderExt.bAdaptiveBaseModeFlag == 1) {
#ifdef BILLING
      curBillTag = PIP_MB_TYPE_TAG;
#endif
      WELS_READ_VERIFY (BsGetOneBit (pBs, &uiCode)); //base_mode_flag
      iBaseModeFlag = uiCode;
    } else {
      iBaseModeFlag = pSlice->sSliceHeaderExt.bDefaultBaseModeFlag;
    }
    if (!iBaseModeFlag) {
      iRet = WelsActualDecodeMbCavlcPSlice (pCtx, rtd);
    } else {
      WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING, "iBaseModeFlag (%d) != 0, inter-layer prediction not supported.",
               iBaseModeFlag);
      return GENERATE_ERROR_NO (ERR_LEVEL_SLICE_HEADER, ERR_INFO_UNSUPPORTED_ILP);
    }
    if (iRet) { //occur error when parsing, MUST STOP decoding
      return iRet;
    }
  }
  // check whether there is left bits to read next time in case multiple slices
  iUsedBits = ((pBs->pCurBuf - pBs->pStartBuf) << 3) - (16 - pBs->iLeftBits);
  // sub 1, for stop bit
  if ((iUsedBits == (pBs->iBits - 1)) && (0 >= pCurLayer->sLayerInfo.sSliceInLayer.iMbSkipRun)) { // slice boundary
    uiEosFlag = 1;
  }
  if (iUsedBits > (pBs->iBits -
                   1)) { //When BS incomplete, as long as find it, SHOULD stop decoding to avoid mosaic or crash.
    WelsLog (& (pCtx->sLogCtx), WELS_LOG_WARNING,
             "WelsDecodeMbCavlcISlice()::::pBs incomplete, iUsedBits:%" PRId64 " > pBs->iBits:%d, MUST stop decoding.",
             (int64_t) iUsedBits, pBs->iBits);
    return -1;
  }
  return 0;
}

void WelsBlockFuncInit (SBlockFunc*   pFunc,  int32_t iCpu) {
  pFunc->pWelsSetNonZeroCountFunc   = WelsNonZeroCount_c;
  pFunc->pWelsBlockZero16x16Func    = WelsBlockZero16x16_c;
  pFunc->pWelsBlockZero8x8Func      = WelsBlockZero8x8_c;

#ifdef HAVE_NEON
  if (iCpu & WELS_CPU_NEON) {
    pFunc->pWelsSetNonZeroCountFunc = WelsNonZeroCount_neon;
    pFunc->pWelsBlockZero16x16Func  = WelsBlockZero16x16_neon;
    pFunc->pWelsBlockZero8x8Func    = WelsBlockZero8x8_neon;
  }
#endif

#ifdef HAVE_NEON_AARCH64
  if (iCpu & WELS_CPU_NEON) {
    pFunc->pWelsSetNonZeroCountFunc = WelsNonZeroCount_AArch64_neon;
    pFunc->pWelsBlockZero16x16Func  = WelsBlockZero16x16_AArch64_neon;
    pFunc->pWelsBlockZero8x8Func    = WelsBlockZero8x8_AArch64_neon;
  }
#endif

#if defined(X86_ASM)
  if (iCpu & WELS_CPU_SSE2) {
    pFunc->pWelsSetNonZeroCountFunc = WelsNonZeroCount_sse2;
    pFunc->pWelsBlockZero16x16Func  = WelsBlockZero16x16_sse2;
    pFunc->pWelsBlockZero8x8Func    = WelsBlockZero8x8_sse2;
  }
#endif

}

void WelsBlockInit (int16_t* pBlock, int iW, int iH, int iStride, uint8_t uiVal) {
  int32_t i;
  int16_t* pDst = pBlock;

  for (i = 0; i < iH; i++) {
    memset (pDst, uiVal, iW * sizeof (int16_t));
    pDst += iStride;
  }
}
void WelsBlockZero16x16_c (int16_t* pBlock, int32_t iStride) {
  WelsBlockInit (pBlock, 16, 16, iStride, 0);
}

void WelsBlockZero8x8_c (int16_t* pBlock, int32_t iStride) {
  WelsBlockInit (pBlock, 8, 8, iStride, 0);
}

} // namespace WelsDec
