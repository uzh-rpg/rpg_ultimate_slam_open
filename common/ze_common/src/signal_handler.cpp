// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <ze/common/signal_handler.hpp>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <ze/common/logging.hpp>

namespace ze {

// enforce local linkage
namespace {

volatile bool* s_simple_flag = nullptr;

void handleSignalSimple(int signal)
{
  LOG(WARNING) << "Signal handler was called with signal " << signal;
  *s_simple_flag = false;
}

void installSignalHandlerSimple(int sig)
{
  struct sigaction handler;
  handler.sa_handler = handleSignalSimple;
  sigfillset(&handler.sa_mask); // block all pending signals
  handler.sa_flags = 0;
  if (sigaction(sig, &handler, NULL) < 0)
  {
    LOG(FATAL) << "cannot install the signal handler for signal " << sig
               << " : " << std::strerror(errno);
  }
}

void clearSignalHandlerSimple(int sig)
{
  struct sigaction handler;
  handler.sa_handler = SIG_DFL;
  sigemptyset(&handler.sa_mask);
  handler.sa_flags = 0;
  if (sigaction(sig, &handler, NULL) < 0)
  {
    LOG(ERROR) << "failed to uninstall the signal handler for signal "
               << sig << ": " << std::strerror(errno);
  }
}

} // unnamed namespace

SimpleSigtermHandler::SimpleSigtermHandler(volatile bool &flag)
{
  CHECK(s_simple_flag == nullptr) << "Signal handler already installed";
  s_simple_flag = &flag;

  // note: if one of the functions below throws,
  // s_simple_flag will remain set since the destructor will not be called
  // however, an error here will lead to process termination anyway
  installSignalHandlerSimple(SIGHUP);
  installSignalHandlerSimple(SIGTERM);
  installSignalHandlerSimple(SIGINT);
}

SimpleSigtermHandler::~SimpleSigtermHandler()
{
  clearSignalHandlerSimple(SIGINT);
  clearSignalHandlerSimple(SIGTERM);
  clearSignalHandlerSimple(SIGHUP);

  s_simple_flag = nullptr;
}

} // namespace ze
