/*********************************************************************************************************************
  Copyright 2025 RoboSense Technology Co., Ltd

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*********************************************************************************************************************/
#ifndef DATA_TYPE_H
#define DATA_TYPE_H

#pragma pack(push)
#pragma pack(4)

typedef struct Tag_Position
{
	float x;
	float y;
	float z;
	unsigned long long timeStamp; //ns
}Position;

typedef struct Tag_Point
{
  float x;
  float y;
  float z;
  float nx;
  float ny;
  float nz;
  float intensity;
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned char a;
  unsigned char label;
  unsigned long long timeStamp; //ns
}Point;

typedef struct Tag_PointCloud
{
  Point* data;
  unsigned int size;
}PointCloud;

typedef struct Tag_Triangle
{
	unsigned int index0;
	unsigned int index1;
	unsigned int index2;
}Triangle;

typedef struct Tag_TriangleFacet
{
  Triangle* data;
  unsigned int size;
}TriangleFacet;

typedef struct Tag_RgbImage
{
	unsigned int width;
	unsigned int height;
	unsigned long long timeStamp; //ns
	unsigned char* data; //r g b 24bit bytes_length = width*height*3
}RgbImage;

typedef struct Tag_DepthImage
{
	unsigned int width;
	unsigned int height;
	unsigned long long timeStamp; //ns
	float* data; // 32bit   bytes_length = width*height
}DepthImage;

typedef struct Tag_DeviceEvent
{
  unsigned char event_type;
  unsigned int uuid_size;
  char uuid[128];
}DeviceEvent;

typedef struct Tag_CString
{
	unsigned int length;
	char data[128];
}CString;

typedef struct Tag_TConfig
{
	CString topic;
	CString type;
	CString process_types[32];
	unsigned int process_types_size;
}TConfig;

typedef struct Tag_TSetting
{
	CString topic;
	CString compressedType;
}TSetting;

#pragma pack(pop)

#endif //DATA_TYPE_H