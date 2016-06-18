#include <cstring>
#include <assert.h>
#include "array_nd.h"
#include "macroblock_model.h"
#include "decoder_context.h"
#include "wels_common_defs.h"
#include "decoded_macroblock.h"
#include "wels_common_basis.h"

void Neighbors::init(const FreqImage *f, int x, int y) {
    using namespace Nei;
    for (int i = 0; i < Nei::NUMNEIGHBORS; ++i) {
        n[i] = NULL;
    }
    if (x > 0) {
        n[LEFT] = &f->at(x-1, y);
        if (!n[LEFT]->odata.initialized) {
            n[LEFT] = NULL;
        }
        if (y > 0) {
            n[ABOVELEFT] = &f->at(x-1, y-1);
            if (!n[ABOVELEFT]->odata.initialized) {
                n[ABOVELEFT] = NULL;
            }
        }
    }
    if (y > 0) {
        n[ABOVE] = &f->at(x, y-1);
        if (!n[ABOVE]->odata.initialized) {
            n[ABOVE] = NULL;
        }
        if (x + 1 < (int)f->width) {
            n[ABOVERIGHT] = &f->at(x + 1, y - 1);
            if (!n[ABOVERIGHT]->odata.initialized) {
                n[ABOVERIGHT] = NULL;
            }
        }
    }
    if (f->priorValid) {
        n[PAST] = &f->last(x, y);
        if (!n[PAST]->odata.initialized) {
            n[PAST] = NULL;
        }
    }
}

int curBillTag = PIP_DEFAULT_TAG;
double bill[NUM_TOTAL_TAGS] = {0};
const char * billEnumToName(int en) {
    if (PIP_DEFAULT_TAG == en) return "boilerplate";
    if(PIP_SKIP_TAG == en) return "skip";
    if(PIP_SKIP_END_TAG == en) return "skip end";
    if(PIP_CBPC_TAG == en) return "cbpc";
    if(PIP_CBPL_TAG == en) return "cbpl";
    if(PIP_LAST_MB_TAG == en) return "last mb";
    if(PIP_QPL_TAG == en) return "qpl";
    if(PIP_MB_TYPE_TAG == en) return "mb type";
    if(PIP_REF_TAG == en) return "ref";
    if(PIP_TRANSFORM_8x8_TAG == en) return "transform 8x8";
    if(PIP_8x8_TAG == en) return "pred 8x8";
    if(PIP_16x16_TAG == en) return "pred 16x16";
    if(PIP_PRED_TAG == en) return "pred";
    if(PIP_PRED_MODE_TAG == en) return "pred mode";
    if(PIP_SUB_MB_TAG == en) return "sub mb";
    if(PIP_MVX_TAG == en) return "mv[0]";
    if(PIP_MVY_TAG == en) return "mv[1]";
    if(PIP_LDC_TAG == en) return "luma dc";
    if(PIP_CRDC_TAG == en) return "chroma dc";
    if(PIP_LAC_TAG0 == en) return "luma ac 0";
    if(PIP_LAC_TAG0 + 1 == en) return "luma ac 1";
    if(PIP_LAC_TAG0 + 2 == en) return "luma ac 2";
    if(PIP_LAC_TAG0 + 3 == en) return "luma ac 3";
    if(PIP_LAC_TAG0 + 4 == en) return "luma ac 4";
    if(PIP_LAC_TAG0 + 5 == en) return "luma ac 5";
    if(PIP_LAC_TAG0 + 6 == en) return "luma ac 6";
    if(PIP_LAC_TAG0 + 7 == en) return "luma ac 7";
    if(PIP_LAC_TAG0 + 8 == en) return "luma ac 8";
    if(PIP_LAC_TAG0 + 9 == en) return "luma ac 9";
    if(PIP_LAC_TAG0 + 10 == en) return "luma ac 10";
    if(PIP_LAC_TAG0 + 11 == en) return "luma ac 11";
    if(PIP_LAC_TAG0 + 12 == en) return "luma ac 12";
    if(PIP_LAC_TAG0 + 13 == en) return "luma ac 13";
    if(PIP_LAC_TAG0 + 14 == en) return "luma ac 14";
    if(PIP_LAC_TAG0 + 15 == en) return "luma ac 15";

    if(PIP_CRAC_TAG0 + 0 == en) return "chroma ac 0";
    if(PIP_CRAC_TAG0 + 1 == en) return "chroma ac 1";
    if(PIP_CRAC_TAG0 + 2 == en) return "chroma ac 2";
    if(PIP_CRAC_TAG0 + 3 == en) return "chroma ac 3";
    if(PIP_CRAC_TAG0 + 4 == en) return "chroma ac 4";
    if(PIP_CRAC_TAG0 + 5 == en) return "chroma ac 5";
    if(PIP_CRAC_TAG0 + 6 == en) return "chroma ac 6";
    if(PIP_CRAC_TAG0 + 7 == en) return "chroma ac 7";

    if(PIP_CRAC_TAG0 + 8 == en) return "chroma ac 8";
    if(PIP_CRAC_TAG0 + 9 == en) return "chroma ac 9";
    if(PIP_CRAC_TAG0 + 10 == en) return "chroma ac 10";
    if(PIP_CRAC_TAG0 + 11 == en) return "chroma ac 11";
    if(PIP_CRAC_TAG0 + 12 == en) return "chroma ac 12";
    if(PIP_CRAC_TAG0 + 13 == en) return "chroma ac 13";
    if(PIP_CRAC_TAG0 + 14 == en) return "chroma ac 14";
    if(PIP_CRAC_TAG0 + 15 == en) return "chroma ac 15";

    if (PIP_LAC_0_EOB == en) return "luma 0 eob";
    if (PIP_LAC_0_BITMASK == en) return "luma 0 bitmask";
    if (PIP_LAC_0_EXP == en) return "luma 0 exponent"; //must be contiguous
    if (PIP_LAC_0_RES == en) return "luma 0 significand"; //must be contiguous
    if (PIP_LAC_0_SIGN == en) return "luma 0 sign"; //must be contiguous
    if (PIP_LAC_N_EOB == en) return "luma Nth eob";
    if (PIP_LAC_N_BITMASK == en) return "luma Nth bitmask";
    if (PIP_LAC_N_EXP == en) return "luma Nth exponent"; //must be contiguous
    if (PIP_LAC_N_RES == en) return "luma Nth significand"; //must be contiguous
    if (PIP_LAC_N_SIGN == en) return "luma Nth sign"; //must be contiguous
    if (PIP_CRAC_EOB == en) return "chroma eob"; //must be contiguous
    if (PIP_CRAC_BITMASK == en) return "chroma bitmask"; //must be contiguous
    if (PIP_CRAC_EXP == en) return "chroma exp"; //must be contiguous
    if (PIP_CRAC_RES == en) return "chroma significand"; //must be contiguous
    if (PIP_CRAC_SIGN == en) return "chroma sign"; //must be contiguous
    if(PIP_PREV_PRED_TAG == en) return "prev pred";
    if(PIP_PREV_PRED_MODE_TAG == en) return "prev pred mode";
    if(PIP_NZC_TAG == en) return "nonzero count";
    if (PIP_PADBYTE_TAG == en) return "pad byte";
    return "unknown";
}
#ifdef BILLING
struct BillTally {
    ~BillTally() {
        double total = 0;
        double adjustment = 1;
        for (int i= 0;i  < NUM_TOTAL_TAGS;++i) {
            total += ceil(bill[i] / 8);
        }
        adjustment = iMovie().getOriginalFileSize()/total;
        total = 0;
        
        for (int i= 0;i  < NUM_TOTAL_TAGS;++i) {
            total += ceil(bill[i] * adjustment / 8);
        }
        double final_adjustment = iMovie().getOriginalFileSize() - total;
        total = 0;
        for (int i= 0;i  < NUM_TOTAL_TAGS;++i) {
            if (bill[i]) {
                double cur = ceil(bill[i] * adjustment / 8);
                if ( i == PIP_DEFAULT_TAG) {
                    cur += final_adjustment;
                }
                total += cur;
                fprintf(stdout, "%d :: %f   [%s] \n", i, cur, billEnumToName(i));
            }
        }
        assert(total == iMovie().getOriginalFileSize());
        fprintf(stdout,"TOTAL: %f\n", total);
    }
} tallyAtEnd;
#endif
void MacroblockModel::initCurrentMacroblock(
                                            DecodedMacroblock *curMb,
                                            WelsDec::PWelsDecoderContext pCtx,
                                            const FreqImage*f,
                                            int mbx, int mby) {
    this->mb = curMb;
    this->pCtx = pCtx;
    this->n.init(f, mbx, mby);
}

#include <iostream>

extern const int rasterTo8x8OrderLuma[16];

uint16_t MacroblockModel::getAndUpdateMacroblockLumaNumNonzeros() {
    uint16_t retval = 0;
    bool emit_dc = (MB_TYPE_INTRA16x16 != mb->uiMbType);
    memset(mb->numSubLumaNonzeros_, 0, sizeof(mb->numSubLumaNonzeros_));
    for (int i = 0; i < 256; ++i) {
        if ((i & 15) != 0 || emit_dc) {
            if (mb->odata.lumaAC[i]) {
                ++retval;
                ++mb->numSubLumaNonzeros_[i / 16];
            }
        }
    }
    for (int i = 0; i < 16; ++i) {
        if (!mb->numSubLumaNonzeros_[i]) {
            continue; //std::cout << "no nonzeros ";
        }
        SingleCoefNeighbors nzPriors = {};
        for (int coef = 0; coef < 15; ++coef) {
            SingleCoefNeighbors coefPriors = priorCoef<16>(i, coef, 0);
            if (coef == 0) {
                nzPriors.has_left = coefPriors.has_left;
                nzPriors.has_above = coefPriors.has_above;
                nzPriors.has_past = coefPriors.has_past;
            }
            nzPriors.left += !!coefPriors.left;
            nzPriors.above += !!coefPriors.above;
            nzPriors.past += !!coefPriors.past;
        }
        std::cout << "<{";
        if (nzPriors.has_left) {
            std::cout << (int)nzPriors.left << ",";
        } else {
            std::cout << "X,";
        }
        if (nzPriors.has_above) {
            std::cout << (int)nzPriors.above << ",";
        } else {
            std::cout << "X,";
        }
        if (nzPriors.has_past) {
            std::cout << (int)nzPriors.past;
        } else {
            std::cout << "X";
        }
        std::cout << "} " << (int)mb->numSubLumaNonzeros_[i];
        std::cout << "> ";

        int numCoef = 15;
        for (int j = 15; j > 0; --j) {
            int coef = WelsDec::g_kuiZigzagScan[j];
            int full_index = (i) * 16 + coef;
            if (mb->odata.lumaAC[full_index]) {
                break;
            }
            numCoef--;
        }
        for (int j = 0; j <= numCoef; ++j) {
            int coef = WelsDec::g_kuiZigzagScan[j];
            SingleCoefNeighbors priors = priorCoef<16>(i, coef, 0);
            std::cout << "[";
            if (priors.has_left) {
                std::cout << (int)!!priors.left << ",";
            } else {
                std::cout << "x,";
            }
            if (priors.has_above) {
                std::cout << (int)!!priors.above << ",";
            } else {
                std::cout << "x,";
            }
            if (priors.has_past) {
                std::cout << (int)!!priors.past;
            } else {
                std::cout << "x";
            }
            int full_index = (i) * 16 + coef;
            std::cout << "] " << !!mb->odata.lumaAC[full_index];
            if (j < numCoef) {
                std::cout << " ";
            }
        }
        std::cout << std::endl;
    }

    mb->numLumaNonzeros_ = retval;
    return retval;
}

const int rasterTo8x8OrderLuma[16] = {
    0, 1, 4, 5,
    2, 3, 6, 7,
    8, 9, 12, 13,
    10, 11, 14, 15
};

const int rasterTo8x8OrderChroma[16] = {
    0, 1,
    2, 3
};
template<unsigned int block_size>
MacroblockModel::SingleCoefNeighbors MacroblockModel::priorCoef(int index8x8, int coef, int color) {
    using namespace Nei;
    SingleCoefNeighbors retval = {};
    const int *rasterTo8x8Order = color ? rasterTo8x8OrderChroma : rasterTo8x8OrderLuma;
    int index = rasterTo8x8Order[index8x8];
    int w = 4, h = 4;
    if (color || block_size == 64) {
        w = 2;
        h = 2;
    }
    int ix = (index & (w - 1));
    int iy = (index / w);
    int coloroffset = 0;
    if (color == 2) {
        coloroffset = w * h * 16;
    }
    
    int rasterdelta = block_size == 64 ? 2 : 1;
    if (ix > 0) {
        int full_index = (rasterTo8x8Order[index - rasterdelta]) * 16 + coef + coloroffset;
        if (color) {
            retval.left = mb->odata.chromaAC[full_index];
        }else {
            retval.left = mb->odata.lumaAC[full_index];
        }
        retval.has_left = true;
    } else {
        const DecodedMacroblock *left = n[LEFT];
        if (left) {
            retval.has_left = true;
            int full_index = (rasterTo8x8Order[index + w - rasterdelta]) * 16 + coef + coloroffset;
            if (color) {
                retval.left = left->odata.chromaAC[full_index];
            } else {
                retval.left = left->odata.lumaAC[full_index];
            }
        }
    }
    if (iy > 0) {
        int full_index = (rasterTo8x8Order[index - w * rasterdelta]) * 16 + coef + coloroffset;
        if (color) {
            retval.above = mb->odata.chromaAC[full_index];
        }else {
            retval.above = mb->odata.lumaAC[full_index];
        }
        retval.has_above = true;
    } else {
        const DecodedMacroblock *above = n[ABOVE];
        if (above) {
            int full_index = (rasterTo8x8Order[index + w * (h - rasterdelta) * rasterdelta]) * 16 + coef + coloroffset;
            if (color) {
                retval.above = above->odata.chromaAC[full_index];
            } else {
                retval.above = above->odata.lumaAC[full_index];
            }
        }
    }

    const DecodedMacroblock *past = n[PAST];
    if (past) {
        retval.has_past = true;
        int full_index = rasterTo8x8Order[index] * 16 + coef + coloroffset;
        if (color) {
            retval.past = past->odata.chromaAC[full_index];
        } else {
            retval.past = past->odata.lumaAC[full_index];
        }
    }
    return retval;
}

void MacroblockModel::checkSerializedNonzeros(const bool *nonzeros, const int16_t *ac,
                                              int index, bool emit_dc, int color) {
    int nz = 0;
    if (color) {
        nz = mb->numSubChromaNonzeros_[index + (color > 1 ? 4 : 0)];
    }else {
        nz = mb->numSubLumaNonzeros_[index];
    }
    int tot_nonzeros = 0;
    for (int i = (emit_dc ? 0: 1); i< 16; ++i) {
        if (nonzeros ? nonzeros[i] : (ac[i] ? true : false)) {
            ++tot_nonzeros;
            assert(ac[i]);
        }else {
            assert(!ac[i]);
        }
    }
    assert(nz == tot_nonzeros);
}

DynProb* MacroblockModel::getAcSignificandPrior(const bool *nonzeros, const int16_t *ac,
                                                 int index, int coef,
                                                 bool emit_dc, int color,
                                                 int bit_len, int which_bit, int significand) {
    int nz = 0;
#ifdef USE_NONZEROS
    if (color) {
        nz = mb->numSubChromaNonzeros_[index + (color > 1 ? 4 : 0)];
    }else {
        nz = mb->numSubLumaNonzeros_[index];
    }
#endif
    return &acSignificandPriors.at(coef, nz,
                                   bit_len, which_bit);
}

DynProb* MacroblockModel::getAcSignPrior(const bool *nonzeros, const int16_t *ac,
                                         int index, int coef, int color) {
    int nz = 0;
#ifdef USE_NONZEROS
    if (color) {
        nz = mb->numSubChromaNonzeros_[index + (color > 1 ? 4 : 0)];
    }else {
        nz = mb->numSubLumaNonzeros_[index];
    }
#endif
    return &acSignPriors.at(coef, nz, color);

}
Sirikata::Array1d<DynProb, 15>::Slice MacroblockModel::getAcExpPrior(const bool *nonzeros, const int16_t *ac,
                                         int index, int coef,
                                         bool emit_dc, int color) {
    SingleCoefNeighbors priors = priorCoef<16>(index, coef, color);
    int past_prior = 2;
    int left_prior = 2;
    int above_prior = 2;
    if (priors.has_past) {
        past_prior = priors.past ? 1 : 0;
    }
    if (priors.has_left) {
        left_prior = priors.left ? 1 : 0;
    }
    if (priors.has_above) {
        above_prior = priors.above ? 1 : 0;
    }
    int coef_x = coef & 3;
    int coef_y = coef >> 2;
    int left_freq = 0;
    int above_freq = 0;
    if (coef_x > 0) {
        left_freq = !!nonzeros[coef - 1];
    }
    if (coef_y > 0) {
        above_freq = !!nonzeros[coef - 4];
    }
    int16_t below = 0;
    int16_t right = 0;
    if ((coef_x & 3) != 3) {
        right = ac[coef + 1];
    }
    if (coef_y < 12) {
        below = ac[coef + 4];
    }
    int nz = 0;
#ifdef USE_NONZEROS
    if (color) {
        nz = mb->numSubChromaNonzeros_[index + (color > 1 ? 4 : 0)];
    }else {
        nz = mb->numSubLumaNonzeros_[index];
    }
#endif
    return acExpPriors.at(coef, nz,
                           past_prior,
                           left_freq, above_freq);
}


uint8_t MacroblockModel::getAndUpdateMacroblockChromaNumNonzeros() {
    uint8_t retval = 0;
    memset(mb->numSubChromaNonzeros_, 0, sizeof(mb->numSubChromaNonzeros_));
    for (int i = 0; i < 128; ++i) {
        if (mb->odata.chromaAC[i] && (i & 15) != 0) {
            ++retval;
            ++mb->numSubChromaNonzeros_[i / 16];
        }
    }
    mb->numChromaNonzeros_ = retval;
    return retval;
}

Sirikata::Array1d<DynProb, 511>::Slice MacroblockModel::getSkipRunPrior() {
    using namespace Nei;
    int prior;
    if (n[PAST]) {
        prior = n[PAST]->cachedSkips / 8 + (n[PAST]->cachedSkips % 8 ? 1 : 0);
        //prior = 0;
        //prior = n[PAST]->numLumaNonzeros_ + n[PAST]->numChromaNonzeros_;
    } else {
        prior = 0; // if no past macro exists, can we say it has 0 skip runs?
    }
    //fprintf(stderr, "p=%d t=%d\n", prior, encodeMacroblockType(mb->uiMbType));

    return mbSkipRunPrior.at(prior, encodeMacroblockType(mb->uiMbType));
}
Sirikata::Array1d<DynProb, 128>::Slice MacroblockModel::getQPLPrior(bool isFirstMB, int signLastDelta) {
    //return mbQPLPrior.slice<0,128>();
  return mbQPLPrior.at(isFirstMB, signLastDelta<0?0:(signLastDelta==0?1:2));
}
Branch<4> MacroblockModel::getNumRefIdxL0ActivePrior() {
    using namespace Nei;
    int prior;
    if (n[PAST]) {
        prior = n[PAST]->uiNumRefIdxL0Active;
    } else {
        prior = 0;
    }
    return numRefIdxL0ActivePrior.at(prior, encodeMacroblockType(mb->uiMbType));
};

Branch<2> MacroblockModel::getCbpCPrior() {
    using namespace Nei;
    int prior;
    if (n[PAST]) {
        prior = n[PAST]->uiCbpC;
    } else {
        prior = 0;
    }
    return CbpCPrior.at(prior, encodeMacroblockType(mb->uiMbType));
}

Branch<4> MacroblockModel::getCbpLPrior() {
    using namespace Nei;
    int prior;
    if (n[PAST]) {
        prior = n[PAST]->uiCbpL;
    } else {
        prior = 0;
    }
    return CbpLPrior.at(prior, encodeMacroblockType(mb->uiMbType));
}
DynProb * MacroblockModel::getStopBitPrior(int numMacroblocksThisSlice) {
    return &stopBitPriors.at(numMacroblocksThisSlice < 2048 ? numMacroblocksThisSlice : 2047);
}
Branch<4> MacroblockModel::getMacroblockTypePrior() {
    using namespace Nei;
    int leftType = 15;
    int aboveType = 15;
    int prevType = 15;
    int prior = 15;
    if (n[ABOVE]) {
        aboveType = encodeMacroblockType(n[ABOVE]->uiMbType);
        prior = aboveType;
    }
    if (n[LEFT]) {
        leftType = encodeMacroblockType(n[LEFT]->uiMbType);
        prior = leftType;
    }
    if (n[PAST]) {
        prevType = encodeMacroblockType(n[PAST]->uiMbType);
    }

    prior += prevType;
    return mbTypePriors.at(prior,(uint32_t)(
        pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt
             .sSliceHeader.eSliceType == P_SLICE));
}

Branch<8> MacroblockModel::getSubMbPrior(int i) {

    return subMbPriors.at(encodeMacroblockType(mb->uiMbType));
}

DynProb *MacroblockModel::getTransformSize8x8FlagPrior(int mbType, int qpl) {
  return &transform8x8FlagPriors.at(encodeMacroblockType(mbType), qpl);
}

Branch<4> MacroblockModel::getPredictionModePrior(int predMode, int leftAvail, int topAvail, int leftTopAvail) {
  int availIdx = (leftAvail << 2) | (topAvail << 1) | (leftTopAvail);
  return predictionModePriors.at(encodeMacroblockType(mb->uiMbType), availIdx, predMode);
}

MacroblockModel::DCPrior* MacroblockModel::getLumaDCIntPrior(size_t index) {
  return &lumaDCIntPriors.at(index, mb->eSliceType, encodeMacroblockType(mb->uiMbType));
}

MacroblockModel::DCPrior* MacroblockModel::getChromaDCIntPrior(size_t index) {
  return &chromaDCIntPriors.at(index, mb->eSliceType, encodeMacroblockType(mb->uiMbType));
}

MacroblockModel::NonzerosPrior* MacroblockModel::getNonzerosPrior4x4(int color, int subblockIndex) {
  using namespace Nei;
  int past = 0, left = 0, above = 0;
  if (n[PAST]) {
    past = n[PAST]->countSubblockNonzeros(color, subblockIndex);
  }
  if (color == 0) {
    if ((subblockIndex & 3) == 0) {
      if (n[LEFT]) {
        left = n[LEFT]->countSubblockNonzeros(color, subblockIndex + 3);
      }
    } else {
      left = mb->countSubblockNonzeros(color, subblockIndex - 1);
    }
    if (subblockIndex < 4) {
      if (n[ABOVE]) {
        above = n[ABOVE]->countSubblockNonzeros(color, subblockIndex + 12);
      }
    } else {
      above = mb->countSubblockNonzeros(color, subblockIndex - 4);
    }
  } else {
    // color == 1 (0..3) or color == 2 (4..7)
    if ((subblockIndex & 1) == 0) { // 0,2,4,6
      if (n[LEFT]) {
        left = n[LEFT]->countSubblockNonzeros(color, subblockIndex + 1);
      }
    } else {
      left = mb->countSubblockNonzeros(color, subblockIndex - 1);
    }
    if ((subblockIndex & 2) == 0) { // needs to match 0,1 and 4,5
      if (n[ABOVE]) {
        above = n[ABOVE]->countSubblockNonzeros(color, subblockIndex + 2);
      }
    } else {
      above = mb->countSubblockNonzeros(color, subblockIndex - 2);
    }
  }
  return &nonzerosPriors.at(
      mb->eSliceType,                       // makes very little difference
      encodeMacroblockType(mb->uiMbType),   // small but significant
      color,                                // makes very little difference
      std::min(2, past),                    // past + left + above priors give a pretty decent boost!
      std::min(2, left),
      std::min(2, above));
}

MacroblockModel::NonzerosPrior* MacroblockModel::getNonzerosPrior8x8(int color, int subblockIndex) {
  assert(color == 0);
  assert((subblockIndex & 3) == 0);
  using namespace Nei;
  int past = 0, left = 0, above = 0;
  if (n[PAST]) {
    past = n[PAST]->countSubblockNonzeros8x8(color, subblockIndex);
  }
  subblockIndex /= 4;
  if ((subblockIndex & 1) == 0) { // 0,2
    if (n[LEFT]) {
        left = n[LEFT]->countSubblockNonzeros8x8(color, (subblockIndex + 1) * 4);
    }
  } else {
    left = mb->countSubblockNonzeros8x8(color, (subblockIndex - 1) * 4);
  }
  if ((subblockIndex & 2) == 0) { // needs to match 0,1
    if (n[ABOVE]) {
        above = n[ABOVE]->countSubblockNonzeros8x8(color, (subblockIndex + 2) * 4);
    }
  } else {
    above = mb->countSubblockNonzeros8x8(color, (subblockIndex - 2) * 4);
  }
  return &nonzerosPriors8x8.at(
      mb->eSliceType,                       // makes very little difference
      encodeMacroblockType(mb->uiMbType),   // small but significant
      color,                                // makes very little difference
      std::min(2, past),                    // past + left + above priors give a pretty decent boost!
      std::min(2, left),
      std::min(2, above)); // FIXME: should this be a bit higher than a max of 2?
}

MacroblockModel::ACPrior* MacroblockModel::getACPrior8x8(int index, int coef, int color, const std::vector<int>& emitted, int nonzeros) {
  assert(!color && "Only use 8x8 for luma");
  int nonzeros_left = nonzeros;
  for (int coefficient : emitted) {
    if (coefficient != 0) nonzeros_left--;
  }
  int prev = emitted.empty() ? 0 : emitted.back();
  int prev2 = emitted.size() <= 1 ? 0 : emitted[emitted.size()-2];
  auto neighbors = priorCoef<64>(index, coef, 0);

  return &acPriors8x8.at(
      mb->eSliceType,
      encodeMacroblockType(mb->uiMbType),
      color,
      emitted.size()).at(
          std::min(4, nonzeros_left),
          std::max(0, std::min(4, prev+2)),
          std::max(0, std::min(4, prev2+2)),
          std::max(0, std::min(4, neighbors.left+2)),
          std::max(0, std::min(4, neighbors.above+2)));
}

MacroblockModel::ACPrior* MacroblockModel::getACPrior4x4(int index, int coef, int color, const std::vector<int>& emitted, int nonzeros) {
  int nonzeros_left = nonzeros;
  for (int coefficient : emitted) {
    if (coefficient != 0) nonzeros_left--;
  }
  int prev = emitted.empty() ? 0 : emitted.back();
  int prev2 = emitted.size() <= 1 ? 0 : emitted[emitted.size()-2];
  auto neighbors = priorCoef<16>(index % (color ? 4 : 16), coef, color);

  return &acPriors.at(
      mb->eSliceType,
      encodeMacroblockType(mb->uiMbType),
      color,
      emitted.size()).at(
          std::min(4, nonzeros_left),
          std::max(0, std::min(4, prev+2)),
          std::max(0, std::min(4, prev2+2)),
          std::max(0, std::min(4, neighbors.left+2)),
          std::max(0, std::min(4, neighbors.above+2)));
}

std::pair<MacroblockModel::MotionVectorDifferencePrior*, int>
MacroblockModel::getMotionVectorDifferencePrior(int subblockIndex, int xyIndex) {
  using namespace Nei;
  int prev = 0;
  if (n[PAST]) {
//    prev = n[PAST]->sMbMvp[subblockIndex][xyIndex];
  } else if (n[LEFT]) {
//    prev = n[LEFT]->sMbMvp[subblockIndex][xyIndex];
  } else if (n[ABOVE]) {
//    prev = n[ABOVE]->sMbMvp[subblockIndex][xyIndex];
  }
  return std::make_pair(&motionVectorDifferencePriors.at(mb->uiMbType, subblockIndex), prev);
}

std::pair<Sirikata::Array1d<DynProb, 8>::Slice, uint32_t> MacroblockModel::getLumaI16x16ModePrior() {
  using namespace Nei;
  int prior = 0;
  // NOTE(jongmin): uiLumaI16x16Mode is mostly zero (80%~?; for some streams all zero.)
  // When nonzero, it correlates very well with the value from n[PAST].
  // As such, we'll branch on the value from n[PAST].
  if (n[PAST]) {
    prior = n[PAST]->uiLumaI16x16Mode;
    prior = (prior >= 6) ? 6 : prior;
    /*
    static std::vector<int> histogram(64);
    static int sum = 0;
    histogram[prior * 8 + mb->uiLumaI16x16Mode]++;
    if (++sum % 1000 == 0) {
      printf("Hmm\n");
      for (int i = 0; i < 64; i++) {
        printf("%d%c", histogram[i], i % 8 == 7 ? '\n' : ' ');
      }
      }*/
  } else {
    prior = 7;
  }
  return std::make_pair(chromaI8x8ModePriors.at(prior), (uint32_t)prior);
}

std::pair<Sirikata::Array1d<DynProb, 8>::Slice, uint32_t> MacroblockModel::getChromaI8x8ModePrior() {
  using namespace Nei;
  int prior = 0;
  if (n[PAST]) {
    prior = n[PAST]->uiChmaI8x8Mode;
    prior = (prior >= 6) ? 6 : prior;
  } else {
    prior = 7;
  }
  return std::make_pair(chromaI8x8ModePriors.at(prior), (uint32_t)prior);
}

int MacroblockModel::encodeMacroblockType(int welsType) {
    switch (welsType) {
        case MB_TYPE_INTRA4x4:
            return 0;
        case MB_TYPE_INTRA16x16:
            return 1;
        case MB_TYPE_INTRA8x8:
            return 2;
        case MB_TYPE_16x16:
            return 3;
        case MB_TYPE_16x8:
            return 4;
        case MB_TYPE_8x16:
            return 5;
        case MB_TYPE_8x8:
            return 6;
        case MB_TYPE_8x8_REF0:
            return 7;
        case MB_TYPE_INTRA_PCM:
            return 8;
        case MB_TYPE_INTRA_BL:
            return 9;
        case MB_TYPE_DIRECT2:
            return 10;
        case MB_TYPE_SKIP:
        case 0: // this doesn't get set in the roundtrip
            return 11;
        default:
            fprintf(stderr, "Invalid macroblock type %d\n", welsType);
            assert(false && "Invalid macroblock type");
            return 0;
    }
}
int MacroblockModel::decodeMacroblockType(int storedType) {
    static const uint32_t MB_TYPES[16] = {
        MB_TYPE_INTRA4x4, MB_TYPE_INTRA16x16, MB_TYPE_INTRA8x8,
        MB_TYPE_16x16, MB_TYPE_16x8, MB_TYPE_8x16, MB_TYPE_8x8,
        MB_TYPE_8x8_REF0, MB_TYPE_INTRA_PCM, MB_TYPE_INTRA_BL,
        MB_TYPE_DIRECT2, 0, 0, 0, 0, 0
    };
    int ret = MB_TYPES[storedType];
    if (!ret) {
        fprintf(stderr, "Invalid decoded macroblock type %d\n", storedType);
        assert (ret && "Invalid decoded macroblock type");
    }
    return ret;
}


uint8_t MacroblockModel::get4x4NumNonzeros(uint8_t index, uint8_t color) const {
    if (color ==0) {
        return mb->numSubLumaNonzeros_[index];
    }
    if (color == 2 && index < 4) {
        index += 4;
    }
    return mb->numSubChromaNonzeros_[index];
}

uint8_t ilog2(uint16_t v) {
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
    uint16_t ret = ilog2(value) + 1;
    return (uint8_t)ret;
}

uint16_t swizzle_sign(int16_t v) {
  if (v >= 0) {
    return ((uint16_t)v) << 1;
  } else {
    return (((uint16_t)(-v-1)) << 1) | 0x1;
  }
}

int16_t unswizzle_sign(uint16_t v) {
  if (v & 0x1) {
    return -(int16_t)(v >> 1) - 1;
  } else {
    return (int16_t)(v >> 1);
  }
}
//  QuantizationTargets getTargetForQuantization(int32_t iResidualProperty);
int32_t DecodedMacroblock::getiResidualProperty(uint32_t uiMbType, bool dc, int color) {
    switch(uiMbType) {
    case MB_TYPE_INTRA16x16:
    case MB_TYPE_INTRA4x4:
    case MB_TYPE_INTRA8x8:
    case MB_TYPE_INTRA_PCM:
    case MB_TYPE_INTRA_BL:
        if (color == 0) {
            if (dc && uiMbType == MB_TYPE_INTRA16x16) {
                return I16_LUMA_DC;
            }
            return LUMA_DC_AC_INTRA;
        } else if (color == 1) {
            if (dc) {
                return CHROMA_DC_U;
            } else {
                return CHROMA_AC_U;
            }
        } else {
            if (dc) {
                return CHROMA_DC_V;
            } else {
                return CHROMA_AC_V;
            }
        }
        break;
    case 0:
    case MB_TYPE_SKIP:
    case MB_TYPE_16x16:
    case MB_TYPE_16x8:
    case MB_TYPE_8x16:
    case MB_TYPE_8x8:
    case MB_TYPE_8x8_REF0:
        if (color == 0) {
            return LUMA_DC_AC_INTER;
        } else if (color == 1) {
            if (dc) {
                return CHROMA_DC_U_INTER;
            } else {
                return CHROMA_AC_U;
            }
        } else {
            if (dc) {
                return CHROMA_DC_V_INTER;
            } else {
                return CHROMA_AC_V;
            }
        }
        break;
    default:
        assert(false && "Can't handle quantization for this kind of macroblock");
    }
    return LUMA_DC_AC_INTRA;
}

const bool MacroblockModel::isPastCorrespondingACNonzero(int index, int coef, int color) {
  if (n[Nei::PAST]) {
    const int16_t* last;
    switch (color) {
    case 0: last = n[Nei::PAST]->odata.lumaAC + (index * 16); break;
    case 1: last = n[Nei::PAST]->odata.chromaAC + (index * 16); break;
    case 2: last = n[Nei::PAST]->odata.chromaAC + ((index + 4) * 16); break;
    }
    return !!last[coef];
  }
  return false;
}
const int MacroblockModel::getPastNonzeroACCount(int index, int color) {
  if (n[Nei::PAST]) {
    // TODO: some code duplication here as the above helper.
    int nonzero_past = 0;
    const int16_t* last;
    switch (color) {
    case 0: last = n[Nei::PAST]->odata.lumaAC + (index * 16); break;
    case 1: last = n[Nei::PAST]->odata.chromaAC + (index * 16); break;
    case 2: last = n[Nei::PAST]->odata.chromaAC + ((index + 4) * 16); break;
    }
    for (int i = 0; i < 16; i++) {
      nonzero_past += !!last[i];
    }
    return nonzero_past;
  }
  return -1;
}
