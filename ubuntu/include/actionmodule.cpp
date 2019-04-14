#include "actionmodule.h"

namespace {
	const int TRANSMIT_PACKET_SIZE = 25;
	const int TRANSMIT_START_PACKET_SIZE = 6;
	const QString radioSendAddress = QString("10.12.225.78");
	const int PORT = 1030;
}

ActionModule::ActionModule() {
	tx.resize(TRANSMIT_PACKET_SIZE);
	tx[0] = 0x40;
	sendStartPacket();
}

void ActionModule::sendStartPacket() {
	QByteArray startPacketSend(TRANSMIT_START_PACKET_SIZE, 0);
	startPacketSend[0] = (char)0xf0;
	startPacketSend[1] = (char)0x5a;
	startPacketSend[2] = (char)0x5a;
	startPacketSend[3] = (char)0x01;
	startPacketSend[4] = (char)0x01;
	startPacketSend[5] = (char)0xa6;
	sendSocket.writeDatagram(startPacketSend, TRANSMIT_START_PACKET_SIZE, QHostAddress(radioSendAddress), PORT);
}

void ActionModule::sendPacket(quint8 id, qint16 vx, qint16 vy, qint16 vr, bool is_dribble) {
	int num = 0;
	qint16 abs_vx = std::abs(vx);
	qint16 abs_vy = std::abs(vy);
	qint16 abs_vr = std::abs(vr);

	// kick   1 : chip   0 : flat`
	bool kick = 0;
	quint8 power = 0;
	// dribble -1 ~ +1 -> -3 ~ +3
	qint8 dribble = is_dribble ? 3:0;
	tx[0] = (tx[0]) | (1 << (3 - num));
	tx[num * 4 + 1] = ((quint8)kick << 6) | dribble << 4 | id;
	tx[num * 4 + 2] = (vx >> 8 & 0x80) | (abs_vx & 0x7f);
	tx[num * 4 + 3] = (vy >> 8 & 0x80) | (abs_vy & 0x7f);
	tx[num * 4 + 4] = (vr >> 8 & 0x80) | (abs_vr & 0x7f);
	tx[num + 17] = (abs_vx >> 1 & 0xc0) | (abs_vy >> 3 & 0x30) | (abs_vr >> 7 & 0x0f);
	tx[num + 21] = power;
	sendSocket.writeDatagram(tx.data(), TRANSMIT_PACKET_SIZE, QHostAddress(radioSendAddress), PORT);
}