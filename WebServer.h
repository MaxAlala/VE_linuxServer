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
    ADD_METHOD_TO(WebServer::controlPage, "/", {Get});
    ADD_METHOD_TO(WebServer::guidePage, "/guide", {Get});
    ADD_METHOD_TO(WebServer::togglePatrol, "/togglePatrol?togglePatrol={1}", {Post});
    ADD_METHOD_TO(WebServer::getPatrol, "/getPatrol", {Get});
    ADD_METHOD_TO(WebServer::getTimeOfStart, "/getTimeOfStart", {Get});
    ADD_METHOD_TO(WebServer::getTimeOfBeingOnline, "/getTimeOfBeingOnline", {Get});
    ADD_METHOD_TO(WebServer::toggleSpotPatrol, "/toggleSpotPatrol?toggleSpotPatrol={1}", {Post});
    ADD_METHOD_TO(WebServer::getSpotPatrol, "/getSpotPatrol", {Get});

    METHOD_LIST_END

    WebServer();
    static void startWebServer();
    static bool isPatrolOn;
    static bool isSpotPatrolOn;
    static int timeOfbeingOnline;
    static std::string timeOfStartUTC;
    static std::string webServerIpAddr;

static void setTimeOfbeingOnline(int timeOfbeingOnlineToSet) {
    timeOfbeingOnline = timeOfbeingOnlineToSet;
    std::cout << "setTimeOfbeingOnline" << timeOfbeingOnline  << std::endl; 
}

static int& getTimeOfbeingOnline() {
    return timeOfbeingOnline;
    // std::cout << "setTimeOfbeingOnline" << timeOfbeingOnline  << std::endl; 
}

    void getTimeOfStart(const HttpRequestPtr &req,
                        std::function<void(const HttpResponsePtr &)> &&callback)
    //   const std::string &password)
    {
        // std::cout << "getTimeOfStart() "<< timeOfStartUTC << "\n";
        // std::cout << "getPatrol: "
        //           << " \n";
        Json::Value timeOfStartUTC_js;
        timeOfStartUTC_js["timeOfStartUtc"] = timeOfStartUTC;

        // Authentication algorithm, read database, verify, identify, etc...
        //...
        //  Json::Value ret;
        //  ret["result"]="ok";
        //  ret["token"]=drogon::utils::getUuid();
        auto resp = HttpResponse::newHttpJsonResponse(timeOfStartUTC_js);
        resp->setStatusCode(k200OK);
        callback(resp);
    }

    void getTimeOfBeingOnline(const HttpRequestPtr &req,
                              std::function<void(const HttpResponsePtr &)> &&callback)
    //   const std::string &password)
    {
        LOG_DEBUG << "getTimeOfbeingOnline() \n";
        // std::cout << "getTimeOfbeingOnline()  "<< timeOfbeingOnline<< "\n";
        Json::Value timeOfbeingOnline_js;
        timeOfbeingOnline_js["timeOfBeingOnline"] = timeOfbeingOnline;
        // std::cout << "this=" << this << "\n";
        // Authentication algorithm, read database, verify, identify, etc...
        //...
        //  Json::Value ret;
        //  ret["result"]="ok";
        //  ret["token"]=drogon::utils::getUuid();
        auto resp = HttpResponse::newHttpJsonResponse(timeOfbeingOnline_js);
        resp->setStatusCode(k200OK);
        callback(resp);
    }

    void getSpotPatrol(const HttpRequestPtr &req,
                   std::function<void(const HttpResponsePtr &)> &&callback)
    //   const std::string &password)
    {
        LOG_DEBUG << "getSpotPatrol() "
                  << " \n";
        // std::cout << "getPatrol: "
        //           << " \n";
        Json::Value patrolVal;
        patrolVal["isSpotPatrolOn"] = isSpotPatrolOn;

        // Authentication algorithm, read database, verify, identify, etc...
        //...
        //  Json::Value ret;
        //  ret["result"]="ok";
        //  ret["token"]=drogon::utils::getUuid();
        auto resp = HttpResponse::newHttpJsonResponse(patrolVal);
        resp->setStatusCode(k200OK);
        callback(resp);
    }

    void toggleSpotPatrol(const HttpRequestPtr &req,
                      std::function<void(const HttpResponsePtr &)> &&callback,
                      std::string &&toggleSpotPatrol);

    void getPatrol(const HttpRequestPtr &req,
                   std::function<void(const HttpResponsePtr &)> &&callback)
    //   const std::string &password)
    {
        LOG_DEBUG << "getPatrol() "
                  << " \n";
        // std::cout << "getPatrol: "
        //           << " \n";
        Json::Value patrolVal;
        patrolVal["isPatrolOn"] = isPatrolOn;

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
                      std::string &&togglePatrol);
    //   const std::string &password)=

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

    // buttons
    // static std::string buttonSetPatrol;

    // bool isPatrolOn;
};