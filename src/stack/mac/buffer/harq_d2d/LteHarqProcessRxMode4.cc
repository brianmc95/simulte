//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#include "stack/mac/buffer/harq_d2d/LteHarqProcessRxMode4.h"
#include "stack/mac/layer/LteMacBase.h"
#include "common/LteControlInfo.h"
#include "stack/mac/packet/LteHarqFeedback_m.h"
#include "stack/mac/packet/LteMacPdu.h"

LteHarqProcessRxMode4::LteHarqProcessRxMode4(unsigned char acid, LteMacBase *owner)
    : LteHarqProcessRx(acid, owner)
{
}

LteHarqProcessRxMode4::~LteHarqProcessRxMode4()
{
}

void LteHarqProcessRxMode4::insertPdu(Codeword cw, LteMacPdu *pdu)
{
    UserControlInfo *lteInfo = check_and_cast<UserControlInfo *>(pdu->getControlInfo());
    bool ndi = lteInfo->getNdi();

    EV << "LteHarqProcessRx::insertPdu - ndi is " << ndi << endl;
    if (ndi && !(status_.at(cw) == RXHARQ_PDU_EMPTY))
        throw cRuntimeError("New data arriving in busy harq process -- this should not happen");

    if (!ndi && !(status_.at(cw) == RXHARQ_PDU_EMPTY) && !(status_.at(cw) == RXHARQ_PDU_CORRUPTED))
        throw cRuntimeError(
                "Trying to insert macPdu in non-empty rx harq process: Node %d acid %d, codeword %d, ndi %d, status %d",
                macOwner_->getMacNodeId(), acid_, cw, ndi, status_.at(cw));

    // deallocate corrupted pdu received in previous transmissions
    if (pdu_.at(cw) != NULL){
        macOwner_->dropObj(pdu_.at(cw));
        delete pdu_.at(cw);
    }

    // store new received pdu
    pdu_.at(cw) = pdu;
    result_.at(cw) = lteInfo->getDeciderResult();
    // No feedback is possible in the Mode 4 standard as such accept result from PHY.
    if (lteInfo->getDeciderResult()){
        status_.at(cw) = RXHARQ_PDU_CORRECT;
    } else {
        status_.at(cw) = RXHARQ_PDU_CORRUPTED;
    }
    rxTime_.at(cw) = NOW;

    transmissions_++;
}
