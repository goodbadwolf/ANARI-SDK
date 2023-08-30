// Copyright 2023 The Khronos Group
// SPDX-License-Identifier: Apache-2.0

#include "KhilariGlobalState.h"
// #include "frame/Frame.h"

namespace khilari {

KhilariGlobalState::KhilariGlobalState(ANARIDevice d)
    : helium::BaseGlobalDeviceState(d)
{}

void KhilariGlobalState::waitOnCurrentFrame() const
{
  printf("TODO: Impl KhilariGlobalState::waitOnCurrentFrame\n");
  /*
  if (currentFrame)
    currentFrame->wait();
  */
}

} // namespace khilari