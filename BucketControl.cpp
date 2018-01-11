#include "BucketControl.h"
#include <seasocks/WebSocket.h>
#include <seasocks/Server.h>
#include <seasocks/PrintfLogger.h>
#include <thread>

using namespace std;
using namespace seasocks;

BucketControl::BucketControl() : server(make_shared<PrintfLogger>(Logger::Level::INFO)) { }

void BucketControl::start() {
	this->serverThread = thread(bind(&BucketControl::run, this));
}

void BucketControl::run() {
	handler = make_shared<ControlHandler>();
	server.addWebSocketHandler("/control", handler);
	server.serve("/home/pi/www", 9090);
}

void BucketControl::setOnDataHandlerFunc(function<void(const char* data)> func) {
	if (handler != NULL)
		handler->setOnDataHandlerFunc(func);
	else
		printf("WebSockets input handler set before BucketControl was started.\n");
}

void BucketControl::kill() {
	server.terminate();
}

void BucketControl::send(const char field, const double value) {
	shared_ptr<Server::Runnable> sendRunner = make_shared<BucketRunnable>([this, field, value]()
	{
		char x[50];
		sprintf(x, "%d %f", field, value);
		handler->send(x);
	});
	server.execute(sendRunner);
}

