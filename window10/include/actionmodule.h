#ifndef __ACTIONMODULE_H__
#define __ACTIONMODULE_H__

#include <QUdpSocket>
#include "singleton.hpp"
#include <QObject>

class ActionModule{
public:
	ActionModule();
	~ActionModule();
	void sendPacket(quint8 id, qint16 vx=0, qint16 vy=0, qint16 vr=0, bool is_dribble = false);
	bool getInfrared(){
		return infrared;
	}
	void readData();
private:
	void sendStartPacket();
	quint8 id = 3;
	QByteArray tx;
	QByteArray rx;
	QUdpSocket sendSocket;
	QUdpSocket receiveSocket;
	bool infrared = false;

};

typedef Singleton<ActionModule> ZActionModule;


#endif // __ACTIONMODULE_H__