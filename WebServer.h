#pragma once
#include <drogon/drogon.h>
#include <memory>
#include <string>

class WebServer{
    public:

WebServer();
void startWebServer();

std::string mainPage;

std::shared_ptr<drogon::orm::DbClient> db;

//buttons
std::string buttonSetPatrol;

std::string webServerIpAddr = "192.168.1.15";
bool isPatrolOn; 
};