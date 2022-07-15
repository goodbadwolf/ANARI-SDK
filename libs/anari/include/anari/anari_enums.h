// Copyright 2021 The Khronos Group
// SPDX-License-Identifier: Apache-2.0

// This file was generated by generate_headers.py
// Don't make changes to this directly


#pragma once
#ifdef __cplusplus
struct ANARIDataType
{
  ANARIDataType() = default;
  constexpr ANARIDataType(int v) noexcept : value(v) {}
  constexpr operator int() const noexcept { return value; }
  constexpr bool operator==(int v) const noexcept { return v == value; }
 private:
  int value;
};
constexpr bool operator<(ANARIDataType v1, ANARIDataType v2)
{
  return static_cast<int>(v1) < static_cast<int>(v2);
}
#define ANARI_DATA_TYPE_DEFINE(v) ANARIDataType(v)
#else
typedef int ANARIDataType;
#define ANARI_DATA_TYPE_DEFINE(v) v
#endif
#define ANARI_UNKNOWN ANARI_DATA_TYPE_DEFINE(0)
#define ANARI_DATA_TYPE ANARI_DATA_TYPE_DEFINE(100)
#define ANARI_STRING ANARI_DATA_TYPE_DEFINE(101)
#define ANARI_VOID_POINTER ANARI_DATA_TYPE_DEFINE(102)
#define ANARI_BOOL ANARI_DATA_TYPE_DEFINE(103)
#define ANARI_STRING_LIST ANARI_DATA_TYPE_DEFINE(150)
#define ANARI_TYPE_LIST ANARI_DATA_TYPE_DEFINE(151)
#define ANARI_PARAMETER_LIST ANARI_DATA_TYPE_DEFINE(152)
#define ANARI_FUNCTION_POINTER ANARI_DATA_TYPE_DEFINE(200)
#define ANARI_MEMORY_DELETER ANARI_DATA_TYPE_DEFINE(201)
#define ANARI_STATUS_CALLBACK ANARI_DATA_TYPE_DEFINE(202)
#define ANARI_LIBRARY ANARI_DATA_TYPE_DEFINE(500)
#define ANARI_DEVICE ANARI_DATA_TYPE_DEFINE(501)
#define ANARI_OBJECT ANARI_DATA_TYPE_DEFINE(502)
#define ANARI_ARRAY ANARI_DATA_TYPE_DEFINE(503)
#define ANARI_ARRAY1D ANARI_DATA_TYPE_DEFINE(504)
#define ANARI_ARRAY2D ANARI_DATA_TYPE_DEFINE(505)
#define ANARI_ARRAY3D ANARI_DATA_TYPE_DEFINE(506)
#define ANARI_CAMERA ANARI_DATA_TYPE_DEFINE(507)
#define ANARI_FRAME ANARI_DATA_TYPE_DEFINE(508)
#define ANARI_GEOMETRY ANARI_DATA_TYPE_DEFINE(509)
#define ANARI_GROUP ANARI_DATA_TYPE_DEFINE(510)
#define ANARI_INSTANCE ANARI_DATA_TYPE_DEFINE(511)
#define ANARI_LIGHT ANARI_DATA_TYPE_DEFINE(512)
#define ANARI_MATERIAL ANARI_DATA_TYPE_DEFINE(513)
#define ANARI_RENDERER ANARI_DATA_TYPE_DEFINE(514)
#define ANARI_SURFACE ANARI_DATA_TYPE_DEFINE(515)
#define ANARI_SAMPLER ANARI_DATA_TYPE_DEFINE(516)
#define ANARI_SPATIAL_FIELD ANARI_DATA_TYPE_DEFINE(517)
#define ANARI_VOLUME ANARI_DATA_TYPE_DEFINE(518)
#define ANARI_WORLD ANARI_DATA_TYPE_DEFINE(519)
#define ANARI_INT8 ANARI_DATA_TYPE_DEFINE(1000)
#define ANARI_INT8_VEC2 ANARI_DATA_TYPE_DEFINE(1001)
#define ANARI_INT8_VEC3 ANARI_DATA_TYPE_DEFINE(1002)
#define ANARI_INT8_VEC4 ANARI_DATA_TYPE_DEFINE(1003)
#define ANARI_UINT8 ANARI_DATA_TYPE_DEFINE(1004)
#define ANARI_UINT8_VEC2 ANARI_DATA_TYPE_DEFINE(1005)
#define ANARI_UINT8_VEC3 ANARI_DATA_TYPE_DEFINE(1006)
#define ANARI_UINT8_VEC4 ANARI_DATA_TYPE_DEFINE(1007)
#define ANARI_INT16 ANARI_DATA_TYPE_DEFINE(1008)
#define ANARI_INT16_VEC2 ANARI_DATA_TYPE_DEFINE(1009)
#define ANARI_INT16_VEC3 ANARI_DATA_TYPE_DEFINE(1010)
#define ANARI_INT16_VEC4 ANARI_DATA_TYPE_DEFINE(1011)
#define ANARI_UINT16 ANARI_DATA_TYPE_DEFINE(1012)
#define ANARI_UINT16_VEC2 ANARI_DATA_TYPE_DEFINE(1013)
#define ANARI_UINT16_VEC3 ANARI_DATA_TYPE_DEFINE(1014)
#define ANARI_UINT16_VEC4 ANARI_DATA_TYPE_DEFINE(1015)
#define ANARI_INT32 ANARI_DATA_TYPE_DEFINE(1016)
#define ANARI_INT32_VEC2 ANARI_DATA_TYPE_DEFINE(1017)
#define ANARI_INT32_VEC3 ANARI_DATA_TYPE_DEFINE(1018)
#define ANARI_INT32_VEC4 ANARI_DATA_TYPE_DEFINE(1019)
#define ANARI_UINT32 ANARI_DATA_TYPE_DEFINE(1020)
#define ANARI_UINT32_VEC2 ANARI_DATA_TYPE_DEFINE(1021)
#define ANARI_UINT32_VEC3 ANARI_DATA_TYPE_DEFINE(1022)
#define ANARI_UINT32_VEC4 ANARI_DATA_TYPE_DEFINE(1023)
#define ANARI_INT64 ANARI_DATA_TYPE_DEFINE(1024)
#define ANARI_INT64_VEC2 ANARI_DATA_TYPE_DEFINE(1025)
#define ANARI_INT64_VEC3 ANARI_DATA_TYPE_DEFINE(1026)
#define ANARI_INT64_VEC4 ANARI_DATA_TYPE_DEFINE(1027)
#define ANARI_UINT64 ANARI_DATA_TYPE_DEFINE(1028)
#define ANARI_UINT64_VEC2 ANARI_DATA_TYPE_DEFINE(1029)
#define ANARI_UINT64_VEC3 ANARI_DATA_TYPE_DEFINE(1030)
#define ANARI_UINT64_VEC4 ANARI_DATA_TYPE_DEFINE(1031)
#define ANARI_FIXED8 ANARI_DATA_TYPE_DEFINE(1032)
#define ANARI_FIXED8_VEC2 ANARI_DATA_TYPE_DEFINE(1033)
#define ANARI_FIXED8_VEC3 ANARI_DATA_TYPE_DEFINE(1034)
#define ANARI_FIXED8_VEC4 ANARI_DATA_TYPE_DEFINE(1035)
#define ANARI_UFIXED8 ANARI_DATA_TYPE_DEFINE(1036)
#define ANARI_UFIXED8_VEC2 ANARI_DATA_TYPE_DEFINE(1037)
#define ANARI_UFIXED8_VEC3 ANARI_DATA_TYPE_DEFINE(1038)
#define ANARI_UFIXED8_VEC4 ANARI_DATA_TYPE_DEFINE(1039)
#define ANARI_FIXED16 ANARI_DATA_TYPE_DEFINE(1040)
#define ANARI_FIXED16_VEC2 ANARI_DATA_TYPE_DEFINE(1041)
#define ANARI_FIXED16_VEC3 ANARI_DATA_TYPE_DEFINE(1042)
#define ANARI_FIXED16_VEC4 ANARI_DATA_TYPE_DEFINE(1043)
#define ANARI_UFIXED16 ANARI_DATA_TYPE_DEFINE(1044)
#define ANARI_UFIXED16_VEC2 ANARI_DATA_TYPE_DEFINE(1045)
#define ANARI_UFIXED16_VEC3 ANARI_DATA_TYPE_DEFINE(1046)
#define ANARI_UFIXED16_VEC4 ANARI_DATA_TYPE_DEFINE(1047)
#define ANARI_FIXED32 ANARI_DATA_TYPE_DEFINE(1048)
#define ANARI_FIXED32_VEC2 ANARI_DATA_TYPE_DEFINE(1049)
#define ANARI_FIXED32_VEC3 ANARI_DATA_TYPE_DEFINE(1050)
#define ANARI_FIXED32_VEC4 ANARI_DATA_TYPE_DEFINE(1051)
#define ANARI_UFIXED32 ANARI_DATA_TYPE_DEFINE(1052)
#define ANARI_UFIXED32_VEC2 ANARI_DATA_TYPE_DEFINE(1053)
#define ANARI_UFIXED32_VEC3 ANARI_DATA_TYPE_DEFINE(1054)
#define ANARI_UFIXED32_VEC4 ANARI_DATA_TYPE_DEFINE(1055)
#define ANARI_FIXED64 ANARI_DATA_TYPE_DEFINE(1056)
#define ANARI_FIXED64_VEC2 ANARI_DATA_TYPE_DEFINE(1057)
#define ANARI_FIXED64_VEC3 ANARI_DATA_TYPE_DEFINE(1058)
#define ANARI_FIXED64_VEC4 ANARI_DATA_TYPE_DEFINE(1059)
#define ANARI_UFIXED64 ANARI_DATA_TYPE_DEFINE(1060)
#define ANARI_UFIXED64_VEC2 ANARI_DATA_TYPE_DEFINE(1061)
#define ANARI_UFIXED64_VEC3 ANARI_DATA_TYPE_DEFINE(1062)
#define ANARI_UFIXED64_VEC4 ANARI_DATA_TYPE_DEFINE(1063)
#define ANARI_FLOAT16 ANARI_DATA_TYPE_DEFINE(1064)
#define ANARI_FLOAT16_VEC2 ANARI_DATA_TYPE_DEFINE(1065)
#define ANARI_FLOAT16_VEC3 ANARI_DATA_TYPE_DEFINE(1066)
#define ANARI_FLOAT16_VEC4 ANARI_DATA_TYPE_DEFINE(1067)
#define ANARI_FLOAT32 ANARI_DATA_TYPE_DEFINE(1068)
#define ANARI_FLOAT32_VEC2 ANARI_DATA_TYPE_DEFINE(1069)
#define ANARI_FLOAT32_VEC3 ANARI_DATA_TYPE_DEFINE(1070)
#define ANARI_FLOAT32_VEC4 ANARI_DATA_TYPE_DEFINE(1071)
#define ANARI_FLOAT64 ANARI_DATA_TYPE_DEFINE(1072)
#define ANARI_FLOAT64_VEC2 ANARI_DATA_TYPE_DEFINE(1073)
#define ANARI_FLOAT64_VEC3 ANARI_DATA_TYPE_DEFINE(1074)
#define ANARI_FLOAT64_VEC4 ANARI_DATA_TYPE_DEFINE(1075)
#define ANARI_UFIXED8_RGBA_SRGB ANARI_DATA_TYPE_DEFINE(2003)
#define ANARI_UFIXED8_RGB_SRGB ANARI_DATA_TYPE_DEFINE(2002)
#define ANARI_UFIXED8_RA_SRGB ANARI_DATA_TYPE_DEFINE(2001)
#define ANARI_UFIXED8_R_SRGB ANARI_DATA_TYPE_DEFINE(2000)
#define ANARI_INT32_BOX1 ANARI_DATA_TYPE_DEFINE(2004)
#define ANARI_INT32_BOX2 ANARI_DATA_TYPE_DEFINE(2005)
#define ANARI_INT32_BOX3 ANARI_DATA_TYPE_DEFINE(2006)
#define ANARI_INT32_BOX4 ANARI_DATA_TYPE_DEFINE(2007)
#define ANARI_FLOAT32_BOX1 ANARI_DATA_TYPE_DEFINE(2008)
#define ANARI_FLOAT32_BOX2 ANARI_DATA_TYPE_DEFINE(2009)
#define ANARI_FLOAT32_BOX3 ANARI_DATA_TYPE_DEFINE(2010)
#define ANARI_FLOAT32_BOX4 ANARI_DATA_TYPE_DEFINE(2011)
#define ANARI_UINT64_REGION ANARI_DATA_TYPE_DEFINE(2104)
#define ANARI_FLOAT32_MAT2 ANARI_DATA_TYPE_DEFINE(2012)
#define ANARI_FLOAT32_MAT3 ANARI_DATA_TYPE_DEFINE(2013)
#define ANARI_FLOAT32_MAT4 ANARI_DATA_TYPE_DEFINE(2014)
#define ANARI_FLOAT32_MAT2x3 ANARI_DATA_TYPE_DEFINE(2015)
#define ANARI_FLOAT32_MAT3x4 ANARI_DATA_TYPE_DEFINE(2016)
#define ANARI_FLOAT32_QUAT_IJKW ANARI_DATA_TYPE_DEFINE(2017)
#define ANARI_FRAME_COMPLETION_CALLBACK ANARI_DATA_TYPE_DEFINE(203)

typedef int ANARILogLevel;
#define ANARI_LOG_DEBUG 1
#define ANARI_LOG_INFO 2
#define ANARI_LOG_WARNING 3
#define ANARI_LOG_ERROR 4
#define ANARI_LOG_NONE 5

typedef unsigned ANARIWaitMask;
#define ANARI_NO_WAIT 0
#define ANARI_WAIT 1

typedef int ANARIStatusCode;
#define ANARI_STATUS_NO_ERROR 0
#define ANARI_STATUS_UNKNOWN_ERROR 1
#define ANARI_STATUS_INVALID_ARGUMENT 2
#define ANARI_STATUS_INVALID_OPERATION 3
#define ANARI_STATUS_OUT_OF_MEMORY 4
#define ANARI_STATUS_UNSUPPORTED_DEVICE 5
#define ANARI_STATUS_VERSION_MISMATCH 6

typedef int ANARIStatusSeverity;
#define ANARI_SEVERITY_FATAL_ERROR 1
#define ANARI_SEVERITY_ERROR 2
#define ANARI_SEVERITY_WARNING 3
#define ANARI_SEVERITY_PERFORMANCE_WARNING 4
#define ANARI_SEVERITY_INFO 5
#define ANARI_SEVERITY_DEBUG 6
