#ifndef __ACTIONMODULE_H__
#define __ACTIONMODULE_H__

#include <QUdpSocket>
#include "singleton.hpp"

class ActionModule{
public:
	ActionModule();
	void sendPacket(quint8 id, qint16 vx=0, qint16 vy=0, qint16 vr=0, bool is_dribble = false);
private:
	void sendStartPacket();
	QByteArray tx;
	QUdpSocket sendSocket;
};

typedef Singleton<ActionModule> ZActionModule;


#endif // __ACTIONMODULE_H__