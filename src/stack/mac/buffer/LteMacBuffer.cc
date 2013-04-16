// 
//                           SimuLTE
// Copyright (C) 2012 Antonio Virdis, Daniele Migliorini, Giovanni
// Accongiagioco, Generoso Pagano, Vincenzo Pii.
// 
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself, 
// and cannot be removed from it.
// 


#include "LteMacBuffer.h"

LteMacBuffer::LteMacBuffer() {
	queueOccupancy_ = 0;
	queueLength_ = 0;
	processed_ = 0;
	Queue_.clear();
}

LteMacBuffer::LteMacBuffer(const LteMacQueue& queue) {
	operator=(queue);
}

LteMacBuffer::~LteMacBuffer() {
	Queue_.clear();
}

LteMacBuffer& LteMacBuffer::operator=(const LteMacBuffer& queue) {
	queueOccupancy_ = queue.queueOccupancy_;
	queueLength_ = queue.queueLength_;
	Queue_ = queue.Queue_;
	return *this;
}

LteMacBuffer* LteMacBuffer::dup () const {
	return new LteMacBuffer(*this);
}

void LteMacBuffer::pushBack (PacketInfo pkt) {
	queueLength_++;
	queueOccupancy_ += pkt.first;
	Queue_.push_back(pkt);
}

void LteMacBuffer::pushFront (PacketInfo pkt) {
	queueLength_++;
	queueOccupancy_ += pkt.first;
	Queue_.push_front(pkt);
}

PacketInfo LteMacBuffer::popFront () {
	if (queueLength_ > 0) {
		PacketInfo pkt = Queue_.front();
		Queue_.pop_front();
		processed_++;
		queueLength_--;
		queueOccupancy_ -= pkt.first;
		return pkt;
	} else {	// Packet queue empty
		opp_error("Packet queue empty");
	}
}

PacketInfo LteMacBuffer::popBack () {
	if (queueLength_ > 0) {
		PacketInfo pkt = Queue_.back();
		Queue_.pop_back();
		queueLength_--;
		queueOccupancy_ -= pkt.first;
		return pkt;
	} else {	// Packet queue empty
		opp_error("Packet queue empty");
	}
}

PacketInfo& LteMacBuffer::front ()
{
	if (queueLength_ > 0) {
		return Queue_.front();
	} else {	// Packet queue empty
		opp_error("Packet queue empty");
	}
}

PacketInfo LteMacBuffer::back () const {
	if (queueLength_ > 0) {
		return Queue_.back();
	} else {	// Packet queue empty
		opp_error("Packet queue empty");
	}
}

void LteMacBuffer::setProcessed(unsigned int i) {
	processed_ = i;
}

simtime_t LteMacBuffer::getHolTimestamp() const {
	if (queueLength_ > 0) {
		return Queue_.front().second;
	} else {	// Packet queue empty
		opp_error("Packet queue empty");
	}
}

unsigned int LteMacBuffer::getProcessed() const {
	return processed_;
}

const std::list<PacketInfo>*
LteMacBuffer::getPacketlist() const
{
	return &Queue_;
}

unsigned int LteMacBuffer::getQueueOccupancy() const {
	return queueOccupancy_;
}

int LteMacBuffer::getQueueLength() const {
	return queueLength_;
}

bool LteMacBuffer::isEmpty() const {
	return (queueLength_==0);
}

std::ostream &operator<<(std::ostream &stream, const LteMacBuffer* queue) {
	stream << "LteMacBuffer-> Length: " << queue->getQueueLength() <<
			" Occupancy: " << queue->getQueueOccupancy() <<
			" HolTimestamp: " << queue->getHolTimestamp() <<
			" Processed: " << queue->getProcessed();
	return stream;
}