/**
 * @file SocketServer.cpp
 * @brief 接続を待ち、クライアントからのリクエストを処理するクラス
 * @author okuyama0528, sadomiya-sousi
 */

#include "SocketServer.h"
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define PORT 27015
#define DEFAULT_BUFLEN 512

SocketServer::SocketServer(ColorRegionDetecorHandler& _colorRegionDetecorHandler)
  : listenSocket(-1), isRunning(false), colorRegionDetecorHandler(_colorRegionDetectorHandler),
{
}

bool SocketServer::init()
{
  listenSocket = socket(AF_INET, SOCK_STREAM, 0);
  if(listenSocket < 0) {
    perror("socket failed");
    return false;
  }

  int opt = 1;
  if(setsockopt(listenSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    perror("setsockopt failed");
    close(listenSocket);
    return false;
  }

  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  serv_addr.sin_port = htons(PORT);

  if(bind(listenSocket, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    perror("bind failed");
    close(listenSocket);
    return false;
  }

  if(listen(listenSocket, 3) < 0) {
    perror("listen failed");
    close(listenSocket);
    return false;
  }

  std::cout << "Socket server initialized and listening on port " << PORT << std::endl;
  return true;
}

void SocketServer::run()
{
  isRunning = true;
  while(isRunning) {
    int clientSocket = accept(listenSocket, (struct sockaddr*)NULL, NULL);
    if(clientSocket < 0) {
      if(!isRunning) break;  // Shutdown was called
      perror("accept failed");
      continue;
    }
    std::cout << "Client connected." << std::endl;
    handle_connection(clientSocket);
    close(clientSocket);
    std::cout << "Client disconnected." << std::endl;
  }
}

void SocketServer::shutdown()
{
  isRunning = false;
  if(listenSocket != -1) {
    close(listenSocket);
    listenSocket = -1;
  }
  std::cout << "Socket server shutting down." << std::endl;
}

void SocketServer::handle_connection(int clientSocket)
{
  char recvbuf[DEFAULT_BUFLEN];
  ssize_t iResult;

  do {
    iResult = recv(clientSocket, recvbuf, DEFAULT_BUFLEN, 0);
    if(iResult > 0) {
      if(static_cast<size_t>(iResult) >= sizeof(CameraServer::Command)) {
        CameraServer::Command cmd = *reinterpret_cast<CameraServer::Command*>(recvbuf);

        switch(cmd) {
          case CameraServer::Command::COLOR_REGION_DETECTION:
            if(static_cast<size_t>(iResult) == sizeof(CameraServer::ColorRegionDetectionRequest)) {
              auto* request = reinterpret_cast<CameraServer::ColorRegionDetectionRequest*>(recvbuf);

              std::cout << "Executing COLOR_REGION_DETECTION" << std::endl;

              CameraServer::ColorRegionDetectionResponse response;

              colorRegionDetecorHandler.execute(*request, response);

              send(clientSocket, reinterpret_cast<const char*>(&response), sizeof(response), 0);

            } else {
              std::cerr << "Invalid request size for COLOR_REGION_DETECTION." << std::endl;
            }
            break;
          case CameraServer::Command::SHUTDOWN:
            std::cout << "Received SHUTDOWN command." << std::endl;
            shutdown();
            break;
          default:
            std::cerr << "Received unknown command." << std::endl;
            break;
        }
      } else {
        std::cerr << "Received unexpected data size: " << iResult << std::endl;
      }
    } else if(iResult == 0) {
      std::cout << "Connection closing..." << std::endl;
    } else {
      if(isRunning) perror("recv failed");
    }
  } while(iResult > 0);
}