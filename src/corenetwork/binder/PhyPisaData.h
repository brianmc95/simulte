//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#ifndef _LTE_PHYPISADATA_H_
#define _LTE_PHYPISADATA_H_

#include <string.h>
#include <vector>

using namespace omnetpp;

const uint16_t XTABLE_SIZE = 3;   //!<number of element in the X tables
const uint16_t PUSCH_AWGN_SIZE = 33;   //!<number of BLER values per HARQ transmission
const uint16_t PSDCH_AWGN_SIZE = 23;   //!<number of BLER values per HARQ transmission
const uint16_t PSCCH_AWGN_SIZE = 38;   //!<number of BLER values
const uint16_t PSBCH_AWGN_SIZE = 43;   //!<number of BLER values

/**
 * Structure to report transport block error rate value and computed SINR
 */
struct TbErrorStats_t
{
  double tbler;   //!< block error rate
  double sinr;   //!< SINR value
};


class PhyPisaData
{
    double lambdaTable_[10000][3];
    double blerCurves_[3][15][49];
    std::vector<double> channel_;
    public:
    PhyPisaData();
    virtual ~PhyPisaData();
    double getBler(int i, int j, int k){if (j==0) return 1; else return blerCurves_[i][j][k-1];}
    double getLambda(int i, int j){return lambdaTable_[i][j];}
    int nTxMode(){return 3;}
    int nMcs(){return 15;}
    int maxSnr(){return 49;}
    int maxChannel(){return 10000;}
    int maxChannel2(){return 1000;}
    double getChannel(unsigned int i);

    /**
     * List of possible channels
     */
     enum LtePhyChannel
     {
         PDCCH,
         PDSCH,
         PUCCH,
         PUSCH,
         PSCCH,
         PSSCH,
         PSDCH
     };

    /**
     * List of channel models. Note that not all models are available for each physical channel
     */
     enum LteFadingModel
     {
         AWGN
     };

    /**
     * List of transmission mode. Note that not all transmission modes are available for each physical channel or fading model
     */
     enum LteTxMode
     {
         SISO,
         SIMO,
         TxDiversity,
         SpatMultplex
     };

    /**
     * \brief Lookup the BLER for the given SINR
     * \param fadingChannel The channel to use
     * \param txmode The Transmission mode used
     * \param mcs The MCS of the TB
     * \param sinr The mean sinr of the TB
     * \param harqHistory The HARQ information
     * \return A Struct of type TbErrorStats_t containing the TB error rate and the SINR
     */
     static double GetPsschBler (LteFadingModel fadingChannel, LteTxMode txmode, uint16_t mcs, double sinr);

    /**
     * \brief Lookup the BLER for the given SINR
     * \param fadingChannel The channel to use
     * \param txmode The Transmission mode used
     * \param sinr The mean sinr of the TB
     * \param harqHistory The HARQ information
     * \return A Struct of type TbErrorStats_t containing the TB error rate and the SINR
     */
//     static double GetPsdchBler (LteFadingModel fadingChannel, LteTxMode txmode, double sinr);

     /**
      * \brief Lookup the BLER for the given SINR
      * \param fadingChannel The channel to use
      * \param txmode The Transmission mode used
      * \param sinr The mean sinr of the TB
      * \return A Struct of type TbErrorStats_t containing the TB error rate and the SINR
      */
      static double GetPscchBler (LteFadingModel fadingChannel, LteTxMode txmode, double sinr);

     /**
      * \brief Lookup the BLER for the given SINR
      * \param fadingChannel The channel to use
      * \param txmode The Transmission mode used
      * \param sinr The mean sinr of the TB
      * \return A Struct of type TbErrorStats_t containing the TB error rate and the SINR
      */

      static double GetBlerAnalytical(uint16_t mcs, double sinr);

     /**
      * \brief Lookup the BLER for the given SINR
      * \param fadingChannel The channel to use
      * \param txmode The Transmission mode used
      * \param mcs The MCS of the TB
      * \param sinr The mean sinr of the TB
      * \param harqHistory The HARQ information
      * \return A Struct of type TbErrorStats_t containing the TB error rate and the SINR
      */
//      static double GetPuschBler (LteFadingModel fadingChannel, LteTxMode txmode, uint16_t mcs, double sinr);

     /**
      * \brief Lookup the BLER for the given SINR
      * \param fadingChannel The channel to use
      * \param txmode The Transmission mode used
      * \param sinr The mean sinr of the TB
      * \return A Struct of type TbErrorStats_t containing the TB error rate and the SINR
      */
//      static double GetPsbchBler (LteFadingModel fadingChannel, LteTxMode txmode, double sinr);

      static double GetBlerAnayltical(uint16_t mcs, double sinr);

      private:

     /**
      * \brief Find the index of the data. Returns -1 if out of range.
      * \param mcs The MCS of the TB
      * \param harq The transmission number
      * \return The row index
      */
      static int16_t GetRowIndex (uint16_t mcs, uint8_t harq);

     /**
      * \brief Find the index of the column where the BLER is located. Returns -1 if out of range.
      * \param val The BLER value
      * \param min The minimum BLER value
      * \param max The maximum BLER value
      * \param step The step size
      * \return The index of the column
      */
      static int16_t GetColIndex (double val, double min, double max, double step);

     /**
      * \brief Get BLER value function
      * \param *xtable Pointer to the x-axis table
      * \param *ytable Pointer to the y-axis table
      * \param ysize The number of columns of the table containing y-axis values (BLER)
      * \param mcs The MCS
      * \param harq The HARQ index
      * \param sinr The SINR
      * \return The BLER value
      */
      static double GetBlerValue (const double (*xtable)[XTABLE_SIZE], const double *ytable, const uint16_t ysize, uint16_t mcs, uint8_t harq, double sinr);

     /**
      * \brief Get BLER value function
      * \param *xtable Pointer to the x-axis table
      * \param *ytable Pointer to the y-axis table
      * \param ysize The number of columns of the table containing y-axis values (BLER)
      * \param mcs The MCS
      * \param harq The HARQ index
      * \param bler The BLER
      * \return The SINR value
      */
      static double GetSinrValue (const double (*xtable)[XTABLE_SIZE], const double *ytable, const uint16_t ysize, uint16_t mcs, uint8_t harq, double bler);

     /**
      * \brief Compute the SINR value given the index on the table
      * \param index The index
      * \param min The minimum value
      * \param max The maximum value
      * \param step The step size
      * \return The SINR value
      */
      static double GetValueForIndex (uint16_t index, double min, double max, double step);

     /**
      * \brief Compute the index of the last column of the table given the BLER
      * \param bler The BLER
      * \param row The row number
      * \param *ytable Pointer to the y-axis table
      * \param ysize The number of columns of the table containing y-axis values
      * \return The index of the last column where BLER is above the target one
      */
      static uint16_t GetBlerIndex (double bler, uint16_t row, const double *ytable, const uint16_t ysize);

     /**
      * \brief Generic function to compute the effective BLER and SINR
      * \param *xtable Pointer to the x-axis table
      * \param *ytable Pointer to the y-axis table
      * \param ysize The number of columns of the table containing y-axis values
      * \param mcs The MCS
      * \param harq The HARQ index
      * \param prevSinr The previous SINR value in linear scale
      * \param newSinr The new SINR value in linear scale
      * \return A Struct of type TbErrorStats_t containing the TB error rate and the SINR
      */
      static TbErrorStats_t GetBler (const double (*xtable)[XTABLE_SIZE], const double *ytable, const uint16_t ysize, uint16_t mcs, uint8_t harq, double prevSinr, double newSinr);

};

#endif
