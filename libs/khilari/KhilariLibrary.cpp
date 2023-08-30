// Copyright 2023 The Khronos Group
// SPDX-License-Identifier: Apache-2.0

#include "KhilariDevice.h"
#include "anari/backend/LibraryImpl.h"
#include "anari_library_khilari_export.h"

namespace khilari {

const char **query_extensions();

struct KhilariLibrary : public anari::LibraryImpl
{
  KhilariLibrary(
      void *lib, ANARIStatusCallback defaultStatusCB, const void *statusCBPtr);

  ANARIDevice newDevice(const char *subtype) override;
  const char **getDeviceExtensions(const char *deviceType) override;
};

// Definitions ////////////////////////////////////////////////////////////////

KhilariLibrary::KhilariLibrary(
    void *lib, ANARIStatusCallback defaultStatusCB, const void *statusCBPtr)
    : anari::LibraryImpl(lib, defaultStatusCB, statusCBPtr)
{}

ANARIDevice KhilariLibrary::newDevice(const char * /*subtype*/)
{
  return (ANARIDevice) new KhilariDevice(this_library());
}

const char **KhilariLibrary::getDeviceExtensions(const char * /*deviceType*/)
{
  // return query_extensions();
  return nullptr;
}

} // namespace khilari

// Define library entrypoint //////////////////////////////////////////////////

extern "C" KHILARI_DEVICE_INTERFACE ANARI_DEFINE_LIBRARY_ENTRYPOINT(
    khilari, handle, scb, scbPtr)
{
  return (ANARILibrary) new khilari::KhilariLibrary(handle, scb, scbPtr);
}

extern "C" KHILARI_DEVICE_INTERFACE ANARIDevice anariNewKhilariDevice(
    ANARIStatusCallback defaultCallback, const void *userPtr)
{
  return (ANARIDevice) new khilari::KhilariDevice(defaultCallback, userPtr);
}
