#pragma once
#include <ctime>
#include <string>
#include <deque>
#include <iostream>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <thread>
#include <mutex>
#include <algorithm>
#include <iomanip>
#include <array>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include "protocol.hpp"
#include "MotorController.h"

using boost::asio::ip::tcp;

//    std::string message = "C M " + std::to_string(steps1) + " " + std::to_string(steps2) + " " + std::to_string(commandID) + "\n";
//    std::string message = "C H " + std::to_string(commandID) + " " + std::to_string(Axis1Direction) + "\n";

namespace
{
	std::string getTimestamp()
	{
		time_t t = time(0);   // get time now
		struct tm* now = localtime(&t);
		std::stringstream ss;
		ss << '[' << (now->tm_year + 1900) << '-' << std::setfill('0')
			<< std::setw(2) << (now->tm_mon + 1) << '-' << std::setfill('0')
			<< std::setw(2) << now->tm_mday << ' ' << std::setfill('0')
			<< std::setw(2) << now->tm_hour << ":" << std::setfill('0')
			<< std::setw(2) << now->tm_min << ":" << std::setfill('0')
			<< std::setw(2) << now->tm_sec << "] ";

		return ss.str();
	}

	class workerThread
	{
	public:
		static void run(std::shared_ptr<boost::asio::io_service> io_service)
		{
			{
				std::lock_guard < std::mutex > lock(m);
				std::cout << "[" << std::this_thread::get_id() << "] Thread starts" << std::endl;
			}

			io_service->run();

			{
				std::lock_guard < std::mutex > lock(m);
				std::cout << "[" << std::this_thread::get_id() << "] Thread ends" << std::endl;
			}

		}
	private:
		static std::mutex m;
	};

	std::mutex workerThread::m;
}


class participant
{
public:
	virtual ~participant() {}
	virtual void onMessage(std::array<char, MAX_IP_PACK_SIZE>& msg) = 0;
};

class chatRoom {
public:
	void enter(std::shared_ptr<participant> participant, const std::string& nickname);
	void leave(std::shared_ptr<participant> participant);
	void broadcast(std::array<char, MAX_IP_PACK_SIZE>& msg, std::shared_ptr<participant> participant);

	std::string getNickname(std::shared_ptr<participant> participant);
	std::unordered_map<std::string, int> name_to_id[NUMBER_OF_ANGLES];
private:
	enum { max_recent_msgs = 100 };
	std::unordered_set<std::shared_ptr<participant>> participants_;
	std::unordered_map<std::shared_ptr<participant>, std::string> name_table_;

	std::deque<std::array<char, MAX_IP_PACK_SIZE>> recent_msgs_;
};



class personInRoom : public participant,
	public std::enable_shared_from_this<personInRoom>
{
public:
	personInRoom(boost::asio::io_service& io_service,
		boost::asio::io_service::strand& strand, chatRoom& room,
		MotorController* motorController);

	tcp::socket& socket() { return socket_; }

	void start();
	void onMessage(std::array<char, MAX_IP_PACK_SIZE>& msg);
	chatRoom& room_;
		std::array<char, MAX_NICKNAME> nickname_;
		std::string nicknameStr;
	//std::vector<int> angles;
private:
	void nicknameHandler(const boost::system::error_code& error);

	void readHandler(const boost::system::error_code& error);
	void writeHandler(const boost::system::error_code& error);

	tcp::socket socket_;
	boost::asio::io_service::strand& strand_;



	std::array<char, MAX_IP_PACK_SIZE> read_msg_;
	std::deque<std::array<char, MAX_IP_PACK_SIZE> > write_msgs_;
	MotorController* motorController_;
};

class stringParser
{
public:

	enum class CommandTypes
	{
		NOT_DETECTED_COMMAND = 0,
		AUTOHOMING,
		MOVE,
		REQUEST_ID,
	};

	//CommandTypes color = CommandTypes::GRAY;
	CommandTypes detectCommandType(std::array<char, MAX_IP_PACK_SIZE>& read_msg_);
	bool parseCommandMove(std::array<char, MAX_IP_PACK_SIZE> &read_msg_, std::array<int, NUMBER_OF_ANGLES> &angles, int & axis, int& ID);
	bool parseCommandMoveID(std::array<char, MAX_IP_PACK_SIZE> &read_msg_, int &axis);
bool removeSomeTrash(std::array<char, MAX_IP_PACK_SIZE> &read_msg_);
};


class ServerController {
public:
	ServerController() {
		serverPort = 8888;
	}

	bool startServer();
private:
	int serverPort;
};

class server
{
public:
	server(boost::asio::io_service& io_service,
		boost::asio::io_service::strand& strand,
		const tcp::endpoint& endpoint, MotorController* motorController);


	private:

	void run();
	void onAccept(std::shared_ptr<personInRoom> new_participant, const boost::system::error_code& error);

	boost::asio::io_service& io_service_;
	boost::asio::io_service::strand& strand_;
	tcp::acceptor acceptor_;
	MotorController* motorController_;
	chatRoom room_;
};

