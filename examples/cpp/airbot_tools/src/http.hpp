#ifndef HTTP_HPP
#define HTTP_HPP
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <sstream>
#include <string>

std::string post(const std::string& host, int port, const std::string& path, const std::string& body);

class HTTPException : public std::exception {
 public:
  explicit HTTPException(const std::string& message) : message_(message) {}

  const char* what() const noexcept override { return message_.c_str(); }

 private:
  std::string message_;
};
#endif