#ifndef _BUCKETCONTROL_H_
#define _BUCKETCONTROL_H_

#define CONTROL_OUTPUT_ROLL			11
#define CONTROL_OUTPUT_PITCH		12
#define CONTROL_OUTPUT_THROTTLE_0	21
#define CONTROL_OUTPUT_THROTTLE_1	22
#define CONTROL_OUTPUT_RPM_0		31
#define CONTROL_OUTPUT_RPM_1		32

#include <thread>
#include <seasocks/WebSocket.h>
#include <seasocks/Server.h>
#include <seasocks/PrintfLogger.h>

using namespace std;
using namespace seasocks;

struct ControlHandler : WebSocket::Handler {
	set<WebSocket *> connections;
	function<void(const char* data)> onDataHandlerFunc;

  	void onConnect(WebSocket *socket) override {
		connections.insert(socket);
	}

	void setOnDataHandlerFunc(function<void(const char* data)> func) {
		onDataHandlerFunc = func;
	}

  	void onData(WebSocket *socket, const char *data) override {
		if (onDataHandlerFunc != NULL)
			onDataHandlerFunc(data);
	}

  	void onDisconnect(WebSocket *socket) override {
		connections.erase(socket);
	}

    void send(const char* data) {
        for (auto c: connections)
            c->send(data);
    }
};

class BucketRunnable : public Server::Runnable {
	function<void()> func;

	public:
		BucketRunnable(function<void()> func) {
			this->func = func;
		}

		virtual void run() {
			this->func();
		}
};

class BucketControl {
	public:
		BucketControl();
		void start();
		void run();
		void setOnDataHandlerFunc(function<void(const char* data)> func);
		void send(const char field, const double value);
		void kill();

	private:
		Server server;
		shared_ptr<ControlHandler> handler;
		thread serverThread;
};

#endif /* _BUCKETCONTROL_H_ */
