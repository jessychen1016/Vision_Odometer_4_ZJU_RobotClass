#include "actionmodule.h"
#include <QDebug>


namespace {
	const int TRANSMIT_PACKET_SIZE = 25;
	const int TRANS_FEEDBACK_SIZE = 26;
	const int PORT = 1030;
	const int TRANSMIT_START_PACKET_SIZE = 6;
	const QString radioSendAddress = QString("10.12.225.78");
	const QString radioReceiveAddress = QString("10.12.225.79");
}

ActionModule::ActionModule() {
    tx.resize(TRANSMIT_PACKET_SIZE);
    tx[0] = 0x40;
    receiveSocket.bind(QHostAddress::AnyIPv4, PORT, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
	sendStartPacket();
}

ActionModule::~ActionModule() {
   //sendSocket.disconnectFromHost();
    //receiveSocket.disconnectFromHost();
}


void ActionModule::sendStartPacket() {
	QByteArray startPacketSend(TRANSMIT_START_PACKET_SIZE, 0);
	QByteArray startPacketReceive(TRANSMIT_START_PACKET_SIZE, 0);
	startPacketSend[0] = (char)0xf0;
	startPacketSend[1] = (char)0x5a;
	startPacketSend[2] = (char)0x5a;
	startPacketSend[3] = (char)0x01;
	startPacketSend[4] = (char)0x01;
	startPacketSend[5] = (char)0xa6;
	sendSocket.writeDatagram(startPacketSend, TRANSMIT_START_PACKET_SIZE, QHostAddress(radioSendAddress), PORT);
	receiveSocket.writeDatagram(startPacketReceive, TRANSMIT_START_PACKET_SIZE, QHostAddress(radioReceiveAddress), PORT);
}

void ActionModule::readData() {
	qDebug() << "bangbingbangbing" << bool(receiveSocket.state() == QUdpSocket::BoundState);
	qDebug() << "shujushujushujushuju" << bool(receiveSocket.hasPendingDatagrams());
    while (receiveSocket.state() == QUdpSocket::BoundState && receiveSocket.hasPendingDatagrams()) {
//        msgInfo->setInfo(newInfo);
		qDebug() << "hahahahahahahahahaha";
        rx.resize(receiveSocket.pendingDatagramSize());
        receiveSocket.readDatagram(rx.data(), rx.size());
        auto& data = rx;
        short wheelVel[4] = {0};

		if(data[0] == (char)0xff && data[1] == (char)0x02  ) {
			qDebug() << "hohohohohohoho"<< (quint8)data[2];
			if(id == (quint8)data[2]){
				qDebug() << "hihihihihihihi";
				infrared = bool((quint8)data[3] & 0x40);
				wheelVel[0] = (quint16)(data[6] << 8) + data[7];
				wheelVel[1] = 1 + (short)~(data[8] << 8) + data[9];
				wheelVel[2] = 1 + (short)~(data[10] << 8) + data[11];
				wheelVel[3] = (quint16)(data[12] << 8) + data[13];
			}
            

        }
    }
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