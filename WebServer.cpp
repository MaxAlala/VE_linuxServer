#include <drogon/drogon.h>
using namespace drogon;

#include "WebServer.h"
#include <string>

WebServer::WebServer()
{
shouldPatrolBeRunning = false;
    // host=localhost port=5432 dbname=SolarBeam connect_timeout=10 user='pi' password=''
    //  std::string connInfo = "host=localhost port=5432 dbname=solarbeam connect_timeout=10 user=pi password=\'1qaz2ws3E4\'";
    //  db = drogon::orm::DbClient::newPgClient(connInfo, 1);
    //  void execSqlAsync(const std::string &sql,
    //                    FUNCTION1 &&rCallback,
    //                    FUNCTION2 &&exceptCallback,
    //                    Arguments &&...args) noexcept
    // auto clientPtr = drogon::app().getDbClient();

    // buttonSetPatrol = "patrol is off";
    // isPatrolOn = false;

    // mainPage = ""
    //            "<!DOCTYPE html>\n"
    //            "<html>\n"
    //            "<head>\n"
    //            "<!-- <link rel=\"stylesheet\" href=\"style.css\"> -->\n"
    //            "<title>Beam control</title>\n"
    //            "\n"
    //            "<style>\n"
    //            "    body {\n"
    //            "    background-color: lightblue;\n"
    //            "  }\n"
    //            "  \n"
    //            "  h1 {\n"
    //            "    color: navy;\n"
    //            "    margin-left: 20px;\n"
    //            "  }\n"
    //            "\n"
    //            "/* CSS */\n"
    //            ".button-9 {\n"
    //            "  appearance: button;\n"
    //            "  backface-visibility: hidden;\n"
    //            "  background-color: #405cf5;\n"
    //            "  border-radius: 6px;\n"
    //            "  border-width: 0;\n"
    //            "  box-shadow: rgba(50, 50, 93, .1) 0 0 0 1px inset,rgba(50, 50, 93, .1) 0 2px 5px 0,rgba(0, 0, 0, .07) 0 1px 1px 0;\n"
    //            "  box-sizing: border-box;\n"
    //            "  color: #fff;\n"
    //            "  cursor: pointer;\n"
    //            "  font-family: -apple-system,system-ui,\"Segoe UI\",Roboto,\"Helvetica Neue\",Ubuntu,sans-serif;\n"
    //            "  font-size: 100%;\n"
    //            "  height: 44px;\n"
    //            "  line-height: 1.15;\n"
    //            "  margin: 12px 0 0;\n"
    //            "  outline: none;\n"
    //            "  overflow: hidden;\n"
    //            "  padding: 0 25px;\n"
    //            "  position: relative;\n"
    //            "  text-align: center;\n"
    //            "  text-transform: none;\n"
    //            "  transform: translateZ(0);\n"
    //            "  transition: all .2s,box-shadow .08s ease-in;\n"
    //            "  user-select: none;\n"
    //            "  -webkit-user-select: none;\n"
    //            "  touch-action: manipulation;\n"
    //            "  width: 100%;\n"
    //            "}\n"
    //            "\n"
    //            ".button-9:disabled {\n"
    //            "  cursor: default;\n"
    //            "}\n"
    //            "\n"
    //            ".button-9:focus {\n"
    //            "  box-shadow: rgba(50, 50, 93, .1) 0 0 0 1px inset, rgba(50, 50, 93, .2) 0 6px 15px 0, rgba(0, 0, 0, .1) 0 2px 2px 0, rgba(50, 151, 211, .3) 0 0 0 4px;\n"
    //            "}\n"
    //            "\n"
    //            "</style>\n"
    //            "</head>\n"
    //            "<body>\n"
    //            "<form action=\"\" method=\"get\">"
    //            "    <button type=\"submit\" name=\"setPatrol\" value=\"toggle\">" +
    //            buttonSetPatrol + "</> \n"
    //                              "</form>"
    //                              "    <button name=\"but1\" value=\"hihih\" class=\"button-9\" role=\"button\">Button 9</button>\n"
    //                              "\n"
    //                              "<h1>This is a Heading</h1>\n"
    //                              "<p>This is a paragraph.</p>\n"
    //                              "\n"
    //                              "</body>\n"
    //                              "</html>";
}

void WebServer::startWebServer()
{
    // `registerHandler()` adds a handler to the desired path. The handler is
    // responsible for generating a HTTP response upon an HTTP request being
    // sent to Drogon
    // app().registerHandler(
    //     "/",
    //     [this](const HttpRequestPtr &,
    //            std::function<void(const HttpResponsePtr &)> &&callback)
    //     {
    //         auto resp = HttpResponse::newHttpResponse();
    //         resp->setBody(this->mainPage);
    //         callback(resp);
    //     },
    //     {Get});

    // app().registerHandler(
    //     "/?setPatrol={user-name}",
    //     [this](const HttpRequestPtr &,
    //            std::function<void(const HttpResponsePtr &)> &&callback,
    //            const std::string &name)
    //     {
    //         auto resp = HttpResponse::newHttpResponse();
    //         resp->setBody(this->mainPage);
    //         std::cout << "server.setPatrol=" << name << "\n";

    //         auto db = app().getDbClient();
    //         db->execSqlAsync(
    //             "select * from SolarBeam",
    //             [](const drogon::orm::Result &result)
    //             {
    //                 std::cout << result.size() << " rows selected!" << std::endl;
    //                 int i = 0;
    //                 for (auto row : result)
    //                 {
    //                     std::cout << i++ << ": timeoflifeinminutes is " << row["timeoflifeinminutes"].as<std::string>() << std::endl;
    //                 }
    //             },
    //             [](const drogon::orm::DrogonDbException &e)
    //             {
    //                 std::cerr << "error:" << e.base().what() << std::endl;
    //             });

    //         callback(resp);
    //     },
    //     {Get});

    // // `registerHandler()` also supports parsing and passing the path as
    // // parameters to the handler. Parameters are specified using {}. The text
    // // inside the {} does not correspond to the index of parameter passed to the
    // // handler (nor it has any meaning). Instead, it is only to make it easier
    // // for users to recognize the function of each parameter.
    // app().registerHandler(
    //     "/user/{user-name}",
    //     [this](const HttpRequestPtr &,
    //            std::function<void(const HttpResponsePtr &)> &&callback,
    //            const std::string &name)
    //     {
    //         auto resp = HttpResponse::newHttpResponse();

    //         resp->setBody(this->mainPage);
    //         callback(resp);
    //     },
    //     {Get});

    // // You can also specify that the parameter is in the query section of the
    // // URL!
    // app().registerHandler(
    //     "/hello?user={user-name}",
    //     [](const HttpRequestPtr &,
    //        std::function<void(const HttpResponsePtr &)> &&callback,
    //        const std::string &name)
    //     {
    //         auto resp = HttpResponse::newHttpResponse();
    //         resp->setBody("Hello, " + name + "!");
    //         callback(resp);
    //     },
    //     {Get});

    // // Or, if you want to, instead of asking drogon to parse it for you. You can
    // // parse the request yourselves.
    // app().registerHandler(
    //     "/hello_user",
    //     [](const HttpRequestPtr &req,
    //        std::function<void(const HttpResponsePtr &)> &&callback)
    //     {
    //         auto resp = HttpResponse::newHttpResponse();
    //         auto name = req->getOptionalParameter<std::string>("user");
    //         if (!name)
    //             resp->setBody("Please tell me your name");
    //         else
    //             resp->setBody("Hello, " + name.value() + "!");
    //         callback(resp);
    //     },
    //     {Get});

    // Ask Drogon to listen on 127.0.0.1 port 8848. Drogon supports listening
    // on multiple IP addresses by adding multiple listeners. For example, if
    // you want the server also listen on 127.0.0.1 port 5555. Just add another
    // line of addListener("127.0.0.1", 5555)
    LOG_INFO << "Server running on " << webServerIpAddr << ":8848 \n";
    // drogon::app().loadConfigFile("../configs/webConfig.json");
    // app().addListener(webServerIpAddr, 8848).run();
    app().loadConfigFile("../configs/webConfig.json").run();

}