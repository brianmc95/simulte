/*
 * SpsCandidateResources.h
 *
 *  Created on: Oct 26, 2018
 *      Author: Brian McCarthy
 *       Email: b.mccarthy@cs.ucc.ie
 */

//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#include "stack/phy/packet/SpsCandidateResources_m.h"
#include "stack/mac/packet/LteSchedulingGrant.h"
#include "common/LteCommon.h"
#include "stack/phy/layer/Subchannel.h"

class SpsCandidateResources: public SpsCandidateResources_Base
{
  protected:

    std::vector<std::tuple<double, int, int>> CSRs;

  public:

    SpsCandidateResources(const char *name = NULL, int kind = 0) :
        SpsCandidateResources_Base(name, kind)
    {
    }

    ~SpsCandidateResources()
    {
    }

    SpsCandidateResources(const SpsCandidateResources& other)
    {
        operator=(other);
    }

    SpsCandidateResources& operator=(const SpsCandidateResources& other)
    {
        CSRs = other.CSRs;
        SpsCandidateResources_Base::operator=(other);
        return *this;
    }

    virtual SpsCandidateResources *dup() const
    {
        return new SpsCandidateResources(*this);
    }

    virtual void setCSRs(const std::vector<std::tuple<double, int, int>> CSRs )
    {
        this->CSRs = CSRs;
    }

    virtual std::vector<std::tuple<double, int, int>> getCSRs()
    {
        return CSRs;
    }
};
