#pragma once
#include <drogon/drogon.h>
#include <memory>
#include <string>
#include <drogon/HttpController.h>
#include <drogon/HttpClient.h>
using namespace drogon;
class WebServer : public HttpController<WebServer>
{
public:
    METHOD_LIST_BEGIN
    METHOD_ADD(WebServer::controlPage, "/", {Get});
    METHOD_ADD(WebServer::guidePage, "/guide", {Get});
    METHOD_ADD(WebServer::togglePatrol, "/?togglePatrol={1}", {Post});
    METHOD_ADD(WebServer::getPatrol, "/getPatrol", {Get});

    METHOD_LIST_END

    WebServer();
    void startWebServer();
    bool shouldPatrolBeRunning;

    void getPatrol(const HttpRequestPtr &req,
                      std::function<void(const HttpResponsePtr &)> &&callback
)
    //   const std::string &password)
    {
        LOG_DEBUG << "getPatrol: " << " \n";
        std::cout << "getPatrol: " << " \n";
         Json::Value patrolVal;
         patrolVal["togglePatrol"] = shouldPatrolBeRunning;

        // Authentication algorithm, read database, verify, identify, etc...
        //...
        //  Json::Value ret;
        //  ret["result"]="ok";
        //  ret["token"]=drogon::utils::getUuid();
        auto resp = HttpResponse::newHttpJsonResponse(patrolVal);
        resp->setStatusCode(k200OK);
        callback(resp);
    }

    void togglePatrol(const HttpRequestPtr &req,
                      std::function<void(const HttpResponsePtr &)> &&callback,
                      std::string &&togglePatrol)
    //   const std::string &password)
    {
        LOG_DEBUG << "shouldPatrolBeRunning: " << togglePatrol << " \n";
        std::cout << "shouldPatrolBeRunning: " << togglePatrol << " \n";

        if (togglePatrol == "1")
            shouldPatrolBeRunning = true;
        else if (togglePatrol == "0")
            shouldPatrolBeRunning = false;
        // Authentication algorithm, read database, verify, identify, etc...
        //...
         Json::Value ret;
        //  ret["result"]="ok";
        //  ret["token"]=drogon::utils::getUuid();
        auto resp = HttpResponse::newHttpResponse();
        resp->setStatusCode(k200OK);
        callback(resp);
    }
    void controlPage(const HttpRequestPtr &req, std::function<void(const HttpResponsePtr &)> &&callback)
    {
        auto resp = HttpResponse::newHttpViewResponse("controlPage");
        callback(resp);
    }

    void guidePage(const HttpRequestPtr &req, std::function<void(const HttpResponsePtr &)> &&callback)
    {
        auto resp = HttpResponse::newHttpViewResponse("guidePage");
        callback(resp);
    }

    // std::string mainPage;

    std::shared_ptr<drogon::orm::DbClient> db;

    // buttons
    std::string buttonSetPatrol;

    std::string webServerIpAddr = "192.168.1.15";
    bool isPatrolOn;
};