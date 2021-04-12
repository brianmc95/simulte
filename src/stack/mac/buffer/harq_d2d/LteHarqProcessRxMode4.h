//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#ifndef _LTE_LTEHARQPROCESSRXMODE4_H_
#define _LTE_LTEHARQPROCESSRXMODE4_H_

#include "stack/mac/buffer/harq/LteHarqProcessRx.h"

/**
 * H-ARQ RX processes contain pdus received from phy layer for which
 * H-ARQ feedback must be sent.
 * These pdus must be evaluated using H-ARQ correction functions.
 * H-ARQ RX process state machine has three states:
 * RXHARQ_PDU_EMPTY
 * RXHARQ_PDU_EVALUATING
 * RXHARQ_PDU_CORRECT
 */
class LteHarqProcessRxMode4 : public LteHarqProcessRx
{
  public:

    /**
     * Constructor.
     *
     * @param acid process identifier
     * @param
     */
    LteHarqProcessRxMode4(unsigned char acid, LteMacBase *owner);

    /**
     * Inserts a pdu into the process and evaluates it (corrupted or correct).
     *
     * @param pdu pdu to be inserted
     */
    virtual void insertPdu(Codeword cw, LteMacPdu *pdu);

    virtual ~LteHarqProcessRxMode4();
};

#endif
