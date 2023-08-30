// Copyright 2022 The Khronos Group
// SPDX-License-Identifier: Apache-2.0

#include "KhilariDevice.h"

#include <vtkm/cont/Initialize.h>

#include "anari/anari_cpp.hpp"
#include "helium/BaseObject.h"
/*
#include "array/Array1D.h"
#include "array/Array2D.h"
#include "array/Array3D.h"
#include "array/ObjectArray.h"
#include "frame/Frame.h"
#include "scene/volume/spatial_field/SpatialField.h"
*/

namespace khilari {

///////////////////////////////////////////////////////////////////////////////
// Generated function declarations ////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

const char **query_object_types(ANARIDataType type);

const void *query_object_info(ANARIDataType type,
    const char *subtype,
    const char *infoName,
    ANARIDataType infoType);

const void *query_param_info(ANARIDataType type,
    const char *subtype,
    const char *paramName,
    ANARIDataType paramType,
    const char *infoName,
    ANARIDataType infoType);

const char **query_extensions();

void managed_deleter(const void *, const void *memory)
{
  delete[] static_cast<char *>(const_cast<void *>(memory));
}

///////////////////////////////////////////////////////////////////////////////
// Helper functions ///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename HANDLE_T, typename OBJECT_T>
inline HANDLE_T getHandleForAPI(OBJECT_T *object)
{
  return (HANDLE_T)object;
}

template <typename OBJECT_T, typename HANDLE_T, typename... Args>
inline HANDLE_T createObjectForAPI(KhilariGlobalState *s, Args &&...args)
{
  return getHandleForAPI<HANDLE_T>(
      new OBJECT_T(s, std::forward<Args>(args)...));
}

///////////////////////////////////////////////////////////////////////////////
// KhilariDevice definitions ///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Data Arrays ////////////////////////////////////////////////////////////////

ANARIArray1D KhilariDevice::newArray1D(const void *appMemory,
    ANARIMemoryDeleter deleter,
    const void *userData,
    ANARIDataType type,
    uint64_t numItems)
{
  ANARIArray1D handle = nextHandle<ANARIArray1D>();
  if (auto obj = getObject(handle)) {
    if (appMemory == nullptr) {
      obj->userdata = nullptr;
      obj->memory = new char[anari::sizeOf(type) * numItems];
      obj->deleter = managed_deleter;
    } else {
      obj->userdata = userData;
      obj->memory = appMemory;
      obj->deleter = deleter;
    }
  }
  return handle;
  /*
  Array1DMemoryDescriptor md;
  md.appMemory = appMemory;
  md.deleter = deleter;
  md.deleterPtr = userData;
  md.elementType = type;
  md.numItems = numItems;

  if (anari::isObject(type))
    return createObjectForAPI<ObjectArray, ANARIArray1D>(deviceState(), md);
  else
    return createObjectForAPI<Array1D, ANARIArray1D>(deviceState(), md);
  */
}

ANARIArray2D KhilariDevice::newArray2D(const void *appMemory,
    ANARIMemoryDeleter deleter,
    const void *userData,
    ANARIDataType type,
    uint64_t numItems1,
    uint64_t numItems2)
{
  ANARIArray2D handle = nextHandle<ANARIArray2D>();
  if (auto obj = getObject(handle)) {
    if (appMemory == nullptr) {
      obj->userdata = nullptr;
      obj->memory = new char[anari::sizeOf(type) * numItems1 * numItems2];
      obj->deleter = managed_deleter;
    } else {
      obj->userdata = userData;
      obj->memory = appMemory;
      obj->deleter = deleter;
    }
  }
  return handle;
  /*
  Array2DMemoryDescriptor md;
  md.appMemory = appMemory;
  md.deleter = deleter;
  md.deleterPtr = userData;
  md.elementType = type;
  md.numItems1 = numItems1;
  md.numItems2 = numItems2;

  return createObjectForAPI<Array2D, ANARIArray2D>(deviceState(), md);
  */
}

ANARIArray3D KhilariDevice::newArray3D(const void *appMemory,
    ANARIMemoryDeleter deleter,
    const void *userData,
    ANARIDataType type,
    uint64_t numItems1,
    uint64_t numItems2,
    uint64_t numItems3)
{
  ANARIArray3D handle = nextHandle<ANARIArray3D>();
  if (auto obj = getObject(handle)) {
    if (appMemory == nullptr) {
      obj->userdata = nullptr;
      obj->memory =
          new char[anari::sizeOf(type) * numItems1 * numItems2 * numItems3];
      obj->deleter = managed_deleter;
    } else {
      obj->userdata = userData;
      obj->memory = appMemory;
      obj->deleter = deleter;
    }
  }
  return handle;
  // Array3DMemoryDescriptor md;
  // md.appMemory = appMemory;
  // md.deleter = deleter;
  // md.deleterPtr = userData;
  // md.elementType = type;
  // md.numItems1 = numItems1;
  // md.numItems2 = numItems2;
  // md.numItems3 = numItems3;

  // return createObjectForAPI<Array3D, ANARIArray3D>(deviceState(), md);
}

// Renderable Objects /////////////////////////////////////////////////////////

ANARILight KhilariDevice::newLight(const char *subtype)
{
  return nextHandle<ANARILight>();
  // return getHandleForAPI<ANARILight>(
  //     Light::createInstance(subtype, deviceState()));
}

ANARICamera KhilariDevice::newCamera(const char *subtype)
{
  return nextHandle<ANARICamera>();
  // return getHandleForAPI<ANARICamera>(
  //     Camera::createInstance(subtype, deviceState()));
}

ANARIGeometry KhilariDevice::newGeometry(const char *subtype)
{
  return nextHandle<ANARIGeometry>();
  // return getHandleForAPI<ANARIGeometry>(
  //     Geometry::createInstance(subtype, deviceState()));
}

ANARISpatialField KhilariDevice::newSpatialField(const char *subtype)
{
  return nextHandle<ANARISpatialField>();
  // return getHandleForAPI<ANARISpatialField>(
  //     SpatialField::createInstance(subtype, deviceState()));
}

ANARISurface KhilariDevice::newSurface()
{
  return nextHandle<ANARISurface>();
  // return createObjectForAPI<Surface, ANARISurface>(deviceState());
}

ANARIVolume KhilariDevice::newVolume(const char *subtype)
{
  return nextHandle<ANARIVolume>();
  // return getHandleForAPI<ANARIVolume>(
  //     Volume::createInstance(subtype, deviceState()));
}

// Surface Meta-Data //////////////////////////////////////////////////////////

ANARIMaterial KhilariDevice::newMaterial(const char *subtype)
{
  return nextHandle<ANARIMaterial>();
  // return getHandleForAPI<ANARIMaterial>(
  //     Material::createInstance(subtype, deviceState()));
}

ANARISampler KhilariDevice::newSampler(const char *subtype)
{
  return nextHandle<ANARISampler>();
  // return getHandleForAPI<ANARISampler>(
  //     Sampler::createInstance(subtype, deviceState()));
}

// Instancing /////////////////////////////////////////////////////////////////

ANARIGroup KhilariDevice::newGroup()
{
  return nextHandle<ANARIGroup>();
  // return createObjectForAPI<Group, ANARIGroup>(deviceState());
}

ANARIInstance KhilariDevice::newInstance(const char * /*subtype*/)
{
  return nextHandle<ANARIInstance>();
  // return createObjectForAPI<Instance, ANARIInstance>(deviceState());
}

// Top-level Worlds ///////////////////////////////////////////////////////////

ANARIWorld KhilariDevice::newWorld()
{
  return nextHandle<ANARIWorld>();
  // return createObjectForAPI<World, ANARIWorld>(deviceState());
}

// Query functions ////////////////////////////////////////////////////////////

const char **KhilariDevice::getObjectSubtypes(ANARIDataType objectType)
{
  return khilari::query_object_types(objectType);
}

const void *KhilariDevice::getObjectInfo(ANARIDataType objectType,
    const char *objectSubtype,
    const char *infoName,
    ANARIDataType infoType)
{
  return khilari::query_object_info(
      objectType, objectSubtype, infoName, infoType);
}

const void *KhilariDevice::getParameterInfo(ANARIDataType objectType,
    const char *objectSubtype,
    const char *parameterName,
    ANARIDataType parameterType,
    const char *infoName,
    ANARIDataType infoType)
{
  return khilari::query_param_info(objectType,
      objectSubtype,
      parameterName,
      parameterType,
      infoName,
      infoType);
}

// Object + Parameter Lifetime Management /////////////////////////////////////

int KhilariDevice::getProperty(ANARIObject object,
    const char *name,
    ANARIDataType type,
    void *mem,
    uint64_t size,
    uint32_t mask)
{
  return 0;
}

struct FrameData
{
  uint32_t width = 1;
  uint32_t height = 1;
};

void frame_deleter(const void *userdata, const void *memory)
{
  delete[] static_cast<char *>(const_cast<void *>(memory));
  delete static_cast<FrameData *>(const_cast<void *>(userdata));
}

void KhilariDevice::setParameter(
    ANARIObject object, const char *name, ANARIDataType type, const void *mem)
{
  if (auto obj = getObject(object)) {
    if (obj->type == ANARI_FRAME) {
      FrameData *data =
          static_cast<FrameData *>(const_cast<void *>(obj->userdata));
      if (type == ANARI_UINT32_VEC2 && std::strncmp("size", name, 4) == 0) {
        const uint32_t *size = static_cast<const uint32_t *>(mem);
        data->width = size[0];
        data->height = size[1];
        delete[] static_cast<char *>(const_cast<void *>(obj->memory));
        obj->memory = nullptr;
      }
    }
  }
}

void KhilariDevice::unsetParameter(ANARIObject, const char *) {}

void KhilariDevice::unsetAllParameters(ANARIObject) {}

void *KhilariDevice::mapParameterArray1D(ANARIObject object,
    const char *name,
    ANARIDataType dataType,
    uint64_t numElements1,
    uint64_t *elementStride)
{
  if (auto obj = getObject(object)) {
    if (elementStride) {
      *elementStride = 0;
    }
    return obj->mapArray(name, anari::sizeOf(dataType) * numElements1);
  } else {
    return nullptr;
  }
}

void *KhilariDevice::mapParameterArray2D(ANARIObject object,
    const char *name,
    ANARIDataType dataType,
    uint64_t numElements1,
    uint64_t numElements2,
    uint64_t *elementStride)
{
  if (auto obj = getObject(object)) {
    if (elementStride) {
      *elementStride = 0;
    }
    return obj->mapArray(
        name, anari::sizeOf(dataType) * numElements1 * numElements2);
  } else {
    return nullptr;
  }
}

void *KhilariDevice::mapParameterArray3D(ANARIObject object,
    const char *name,
    ANARIDataType dataType,
    uint64_t numElements1,
    uint64_t numElements2,
    uint64_t numElements3,
    uint64_t *elementStride)
{
  if (auto obj = getObject(object)) {
    if (elementStride) {
      *elementStride = 0;
    }
    return obj->mapArray(name,
        anari::sizeOf(dataType) * numElements1 * numElements2 * numElements3);
  } else {
    return nullptr;
  }
}

void KhilariDevice::unmapParameterArray(ANARIObject object, const char *name) {}

void KhilariDevice::commitParameters(ANARIObject) {}

void KhilariDevice::release(ANARIObject object)
{
  if (auto obj = getObject(object)) {
    obj->release();
  }
}

void KhilariDevice::retain(ANARIObject object)
{
  if (auto obj = getObject(object)) {
    obj->retain();
  }
}

// Frame Manipulation /////////////////////////////////////////////////////////

ANARIFrame KhilariDevice::newFrame()
{
  ANARIFrame frame = nextHandle<ANARIFrame>();
  if (auto obj = getObject(frame)) {
    obj->userdata = new FrameData();
    obj->deleter = frame_deleter;
  }
  return frame;
  // initDevice();
  // return createObjectForAPI<Frame, ANARIFrame>(deviceState());
}

const void *KhilariDevice::frameBufferMap(ANARIFrame fb,
    const char *,
    uint32_t *width,
    uint32_t *height,
    ANARIDataType *pixelType)
{
  if (auto obj = getObject(fb)) {
    if (obj->type == ANARI_FRAME) {
      const FrameData *data = static_cast<const FrameData *>(obj->userdata);
      if (obj->memory == nullptr) {
        obj->memory = new char[data->width * data->height * 4 * sizeof(float)];
      }
      *width = data->width;
      *height = data->height;
      *pixelType = ANARI_FLOAT32;
      return obj->memory;
    }
  }
  return nullptr;
}

void KhilariDevice::frameBufferUnmap(ANARIFrame, const char *) {}

// Frame Rendering ////////////////////////////////////////////////////////////

ANARIRenderer KhilariDevice::newRenderer(const char *subtype)
{
  return nextHandle<ANARIRenderer>();
  // return getHandleForAPI<ANARIRenderer>(
  //     Renderer::createInstance(subtype, deviceState()));
}

void KhilariDevice::renderFrame(ANARIFrame) {}

int KhilariDevice::frameReady(ANARIFrame, ANARIWaitMask)
{
  return 1;
}

void KhilariDevice::discardFrame(ANARIFrame) {}

// Other KhilariDevice definitions /////////////////////////////////////////////

KhilariDevice::KhilariDevice(ANARIStatusCallback cb, const void *ptr)
    : helium::BaseDevice(cb, ptr)
{
  m_state = std::make_unique<KhilariGlobalState>(this_device());
  deviceCommitParameters();
  initDevice();
}

KhilariDevice::KhilariDevice(ANARILibrary l) : helium::BaseDevice(l)
{
  m_state = std::make_unique<KhilariGlobalState>(this_device());
  deviceCommitParameters();
  initDevice();
}

KhilariDevice::~KhilariDevice()
{
  auto &state = *deviceState();

  state.commitBuffer.clear();

  reportMessage(ANARI_SEVERITY_DEBUG, "destroying khilari device (%p)", this);

  // NOTE: These object leak warnings are not required to be done by
  //       implementations as the debug layer in the SDK is far more
  //       comprehensive and designed for detecting bugs like this. However
  //       these simple checks are very straightforward to implement and do not
  //       really add substantial code complexity, so they are provided out of
  //       convenience.

  auto reportLeaks = [&](size_t &count, const char *handleType) {
    if (count != 0) {
      reportMessage(ANARI_SEVERITY_WARNING,
          "detected %zu leaked %s objects",
          count,
          handleType);
    }
  };

  reportLeaks(state.objectCounts.frames, "ANARIFrame");
  reportLeaks(state.objectCounts.cameras, "ANARICamera");
  reportLeaks(state.objectCounts.renderers, "ANARIRenderer");
  reportLeaks(state.objectCounts.worlds, "ANARIWorld");
  reportLeaks(state.objectCounts.instances, "ANARIInstance");
  reportLeaks(state.objectCounts.groups, "ANARIGroup");
  reportLeaks(state.objectCounts.surfaces, "ANARISurface");
  reportLeaks(state.objectCounts.geometries, "ANARIGeometry");
  reportLeaks(state.objectCounts.materials, "ANARIMaterial");
  reportLeaks(state.objectCounts.samplers, "ANARISampler");
  reportLeaks(state.objectCounts.volumes, "ANARIVolume");
  reportLeaks(state.objectCounts.spatialFields, "ANARISpatialField");
  reportLeaks(state.objectCounts.arrays, "ANARIArray");

  if (state.objectCounts.unknown != 0) {
    reportMessage(ANARI_SEVERITY_WARNING,
        "detected %zu leaked ANARIObject objects created by unknown subtypes",
        state.objectCounts.unknown);
  }
}

void KhilariDevice::initDevice()
{
  if (m_initialized)
    return;

  reportMessage(ANARI_SEVERITY_DEBUG, "initializing khilari device (%p)", this);

  // TODO: Read the device to use from device paramters and then
  // construct argc and argv accordingly.
  auto initResult = vtkm::cont::Initialize();

  reportMessage(ANARI_SEVERITY_DEBUG,
      "VTKm initialized with device \"%s\"",
      initResult.Device.GetName().c_str());

  m_initialized = true;
}

void KhilariDevice::deviceCommitParameters()
{
  helium::BaseDevice::deviceCommitParameters();
}

KhilariGlobalState *KhilariDevice::deviceState() const
{
  return (KhilariGlobalState *)helium::BaseDevice::m_state.get();
}

} // namespace khilari
