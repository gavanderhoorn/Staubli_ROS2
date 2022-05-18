/*
 *  Copyright (c) 2022 Ivo Dekker ACRO Diepenbeek KULeuven

 Permission is hereby granted, free of charge, to any person
 obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without
 restriction, including without limitation the rights to use,
 copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the
 Software is furnished to do so, subject to the following
 conditions:

 The above copyright notice and this permission notice shall be
 included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 OTHER DEALINGS IN THE SOFTWARE.
 */

#include "robot_middleware/connection_manager/simple_socket_manager.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdio>

using namespace industrial::simple_message;

namespace robot_middleware
{
namespace connection_manager
{

SimpleSocketManager::SimpleSocketManager(const std::string &name, int port)
    : name_(name), port_(port), connected_once_(false), friend_connection_(nullptr)
{
}

SimpleSocketManager::~SimpleSocketManager()
{
    #ifndef NDEBUG
    printf("DEBUG: Destructing SimpleSocketManager '%s'\n", getName());
    #endif

    // wake up connection thread to avoid blocking
    connection_lost_cv_.notify_one();
}

const char *SimpleSocketManager::getName()
{
    return name_.c_str();
}

void SimpleSocketManager::setFriendConnection(const std::shared_ptr<SimpleSocketManager> &conn)
{
    friend_connection_ = conn;
}

bool SimpleSocketManager::isConnected()
{
    return conn_->isConnected();
}

bool SimpleSocketManager::isReadyReceive(int timeout)
{
    return conn_->isReadyReceive(timeout);
}

bool SimpleSocketManager::sendMsg(SimpleMessage &msg)
{
    if (conn_->sendMsg(msg))
    {
        return true;
    }
    else
    {
        connection_lost_cv_.notify_one();
        return false;
    }
}

bool SimpleSocketManager::receiveMsg(SimpleMessage &msg)
{
    if (conn_->receiveMsg(msg))
    {
        return true;
    }
    else
    {
        connection_lost_cv_.notify_one();
        return false;
    }
}

bool SimpleSocketManager::sendAndReceiveMsg(SimpleMessage &send, SimpleMessage &recv)
{
    if (conn_->sendAndReceiveMsg(send, recv))
    {
        return true;
    }
    else
    {
        connection_lost_cv_.notify_one();
        return false;
    }
}

void SimpleSocketManager::disconnect()
{
    RCLCPP_DEBUG(rclcpp::get_logger("simple_socket_manager"), "[%s] Disconnecting", getName());
    conn_->setDisconnected();
    connection_lost_cv_.notify_one();
}

void SimpleSocketManager::startConnectionTask()
{
    std::thread t(&SimpleSocketManager::connectionTask, this);
    t.detach();
}

void SimpleSocketManager::connectionTask()
{
    // store local copy of the connection name string in case it is destroyed
    // before this detached thread is finished
    std::string name = name_; // NOLINT(performance-unnecessary-copy-initialization)

    while (rclcpp::ok())
    {
        if (isConnected())
        {
            std::unique_lock<std::mutex> lock(connection_mtx_);
            connection_lost_cv_.wait(lock);
            RCLCPP_WARN(rclcpp::get_logger("simple_socket_manager"), "[%s] Received 'connection lost' signal while connected!", name.c_str());
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("simple_socket_manager"), "[%s] Connection lost", name.c_str());

            // disconnect friend connection forcing it to reconnect
            if (friend_connection_ && friend_connection_->isConnected())
                friend_connection_->disconnect();

            connect();
        }
    }

    #ifndef NDEBUG
    printf("DEBUG: [%s] Exiting connection task\n", name.c_str());
    #endif
}

} // namespace connection_manager
} // namespace robot_middleware