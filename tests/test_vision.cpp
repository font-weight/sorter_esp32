#include <ColorDetector.h>
#include <Homography.h>
#include "../firmware/camera_node/CameraSession.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <limits>

namespace {
const sorter::ColorRange RANGES[] = {
  {1,120,255,0,110,0,110,60}, {2,0,120,100,255,0,130,45}, {3,0,110,0,150,110,255,50}
};
void pixel(std::vector<uint8_t>& image, int width, int x, int y, uint16_t color) {
  const size_t index = size_t(y*width+x)*2;
  image[index] = uint8_t(color >> 8); image[index+1] = uint8_t(color);
}
void rect(std::vector<uint8_t>& image, int width, int x, int y, int w, int h, uint16_t color) {
  for (int iy = y; iy < y+h; ++iy) for (int ix = x; ix < x+w; ++ix) pixel(image,width,ix,iy,color);
}
bool near(double a, double b, double eps=1e-8) { return fabs(a-b) < eps; }
void testRgb() {
  const uint8_t red[] = {0xF8,0x00}, green[] = {0x07,0xE0}, blue[] = {0x00,0x1F}, white[] = {0xFF,0xFF};
  const sorter::Rgb r = sorter::decodeRgb565BE(red);
  const sorter::Rgb g = sorter::decodeRgb565BE(green);
  const sorter::Rgb b = sorter::decodeRgb565BE(blue);
  const sorter::Rgb w = sorter::decodeRgb565BE(white);
  assert(r.r==255 && r.g==0 && r.b==0);
  assert(g.r==0 && g.g==255 && g.b==0);
  assert(b.r==0 && b.g==0 && b.b==255);
  assert(w.r==255 && w.g==255 && w.b==255);
  assert(sorter::classifyRgb(r,RANGES,3)==1);
  assert(sorter::classifyRgb(g,RANGES,3)==2);
  assert(sorter::classifyRgb(b,RANGES,3)==3);
  assert(sorter::classifyRgb(w,RANGES,3)==0);
  assert(sorter::classifyRgb({50,50,50},RANGES,3)==0);
  sorter::ColorRange overlap[] = {{1,0,255,0,255,0,255,0},{2,0,255,0,255,0,255,0}};
  bool ambiguous = false;
  assert(sorter::classifyRgb(r,overlap,2,&ambiguous)==0 && ambiguous);
  assert(sorter::validColorRanges(overlap,2));
  overlap[1].classId=1;
  assert(!sorter::validColorRanges(overlap,2));
  overlap[1].classId=4;
  assert(!sorter::validColorRanges(overlap,2));
  overlap[1].classId=2; overlap[1].rMin=255; overlap[1].rMax=0;
  assert(!sorter::validColorRanges(overlap,2));
}
void testDetector() {
  const int width=32,height=24;
  std::vector<uint8_t> image(width*height*2,0), labels(width*height);
  std::vector<uint32_t> queue(width*height);
  sorter::DetectorWorkspace work={labels.data(),labels.size(),queue.data(),queue.size()};
  sorter::DetectorConfig cfg={RANGES,3,{0,0,width,height},4,100,true};
  sorter::PixelBlob blobs[8];
  // Two disconnected red parts MUST yield two centres, not their average.
  rect(image,width,2,2,4,3,0xF800);
  rect(image,width,20,3,3,4,0xF800);
  rect(image,width,8,12,5,3,0x07E0);
  rect(image,width,23,16,3,3,0x001F);
  auto result=sorter::detectRgb565BE(image.data(),image.size(),width,height,cfg,work,blobs,8);
  assert(result.status==sorter::DetectionStatus::Ok && result.count==4);
  assert(blobs[0].classId==1 && near(blobs[0].centerX,3.5) && near(blobs[0].centerY,3));
  assert(blobs[1].classId==1 && near(blobs[1].centerX,21) && near(blobs[1].centerY,4.5));
  assert(blobs[2].classId==2 && blobs[2].pixels==15);
  assert(blobs[3].classId==3 && blobs[3].pixels==9);
  assert(result.stats.components==4 && result.stats.matchedPixels==48);
  result=sorter::detectRgb565BE(image.data(),image.size(),width,height,cfg,work,blobs,2);
  assert(result.status==sorter::DetectionStatus::TooManyObjects && result.count==0);
  // Tiny speck, oversized merged region and clipped edge object are excluded.
  image.assign(image.size(),0);
  rect(image,width,0,1,2,4,0xF800);
  rect(image,width,5,3,12,10,0x07E0);
  pixel(image,width,24,4,0x001F);
  rect(image,width,24,15,3,3,0x001F);
  result=sorter::detectRgb565BE(image.data(),image.size(),width,height,cfg,work,blobs,8);
  assert(result.status==sorter::DetectionStatus::Ok && result.count==1);
  assert(result.stats.rejectedEdge==1 && result.stats.rejectedSmall==1 && result.stats.rejectedLarge==1);
  assert(blobs[0].classId==3 && blobs[0].pixels==9);
  // ROI edge rejection uses ROI boundary, not just the full image boundary.
  cfg.roi={2,2,30,22};
  image.assign(image.size(),0); rect(image,width,2,5,3,3,0xF800);
  result=sorter::detectRgb565BE(image.data(),image.size(),width,height,cfg,work,blobs,8);
  assert(result.count==0 && result.stats.rejectedEdge==1);
  cfg.rejectRoiEdge=false;
  result=sorter::detectRgb565BE(image.data(),image.size(),width,height,cfg,work,blobs,8);
  assert(result.count==1);
  // Diagonal contacts belong to one eight-connected component.
  image.assign(image.size(),0);
  for(int i=4;i<8;++i) pixel(image,width,i,i,0xF800);
  result=sorter::detectRgb565BE(image.data(),image.size(),width,height,cfg,work,blobs,8);
  assert(result.count==1 && blobs[0].pixels==4 && near(blobs[0].centerX,5.5));
  // Adjacent different colors remain separate.
  image.assign(image.size(),0); rect(image,width,4,4,2,3,0xF800); rect(image,width,6,4,2,3,0x07E0);
  result=sorter::detectRgb565BE(image.data(),image.size(),width,height,cfg,work,blobs,8);
  assert(result.count==2);
  sorter::DetectorWorkspace small=work; small.queueCapacity=3;
  result=sorter::detectRgb565BE(image.data(),image.size(),width,height,cfg,small,blobs,8);
  assert(result.status==sorter::DetectionStatus::WorkspaceTooSmall && result.count==0);
  result=sorter::detectRgb565BE(image.data(),image.size()-1,width,height,cfg,work,blobs,8);
  assert(result.status==sorter::DetectionStatus::BadImage);
  result=sorter::detectRgb565BE(NULL,image.size(),width,height,cfg,work,blobs,8);
  assert(result.status==sorter::DetectionStatus::BadImage);
  result=sorter::detectRgb565BE(image.data(),image.size(),321,height,cfg,work,blobs,8);
  assert(result.status==sorter::DetectionStatus::BadImage);
  cfg.roi.right=33;
  result=sorter::detectRgb565BE(image.data(),image.size(),width,height,cfg,work,blobs,8);
  assert(result.status==sorter::DetectionStatus::BadConfig);
  cfg.roi.right=30; cfg.minPixels=0;
  result=sorter::detectRgb565BE(image.data(),image.size(),width,height,cfg,work,blobs,8);
  assert(result.status==sorter::DetectionStatus::BadConfig);
}
void testFullFrameAndNoStaleResults() {
  const int width=320,height=240;
  std::vector<uint8_t> image(width*height*2,0), labels(width*height);
  std::vector<uint32_t> queue(width*height);
  sorter::DetectorWorkspace work={labels.data(),labels.size(),queue.data(),queue.size()};
  sorter::DetectorConfig cfg={RANGES,3,{0,0,width,height},1,width*height,false};
  sorter::PixelBlob blobs[8];
  rect(image,width,0,0,width,height,0xF800);
  auto result=sorter::detectRgb565BE(image.data(),image.size(),width,height,cfg,work,blobs,8);
  assert(result.status==sorter::DetectionStatus::Ok && result.count==1);
  assert(blobs[0].pixels==76800 && near(blobs[0].centerX,159.5) && near(blobs[0].centerY,119.5));
  image.assign(image.size(),0);
  result=sorter::detectRgb565BE(image.data(),image.size(),width,height,cfg,work,blobs,8);
  assert(result.status==sorter::DetectionStatus::Ok && result.count==0 && result.stats.components==0);
}
void testHomography() {
  sorter::Homography map;
  const sorter::Point2 quad[]={{0,0},{100,0},{100,100},{0,100}};
  const double affine[]={0.5,0,10,0,-0.25,40,0,0,1};
  double x=0,y=0;
  assert(!map.map(50,50,x,y));
  assert(map.configure(affine,quad,4));
  assert(map.map(20,40,x,y) && near(x,20) && near(y,30));
  assert(map.contains(0,0) && map.contains(100,100));
  assert(!map.contains(-0.001,50) && !map.contains(50,100.001));
  x=123; y=456;
  assert(!map.map(101,50,x,y) && x==123 && y==456);
  int32_t x10=0,y10=0;
  assert(map.mapTenths(20,40,x10,y10) && x10==200 && y10==300);
  assert(!map.map(NAN,0,x,y) && !map.contains(INFINITY,0));
  const sorter::Point2 reverse[]={{0,100},{100,100},{100,0},{0,0}};
  assert(map.configure(affine,reverse,4) && map.map(50,50,x,y));
  const double perspective[]={1,0,0,0,1,0,0.001,0.002,1};
  assert(map.configure(perspective,quad,4));
  assert(map.map(50,50,x,y) && near(x,50.0/1.15) && near(y,50.0/1.15));
  const double singular[]={1,0,0,2,0,0,0,0,1};
  assert(!map.configure(singular,quad,4) && !map.valid());
  const double horizon[]={1,0,0,0,1,0,0.02,0,-1};
  assert(!map.configure(horizon,quad,4)); // Horizon crosses support.
  const double bad[]={1,0,0,0,1,NAN,0,0,1};
  assert(!map.configure(bad,quad,4));
  const sorter::Point2 concave[]={{0,0},{100,0},{50,40},{0,100}};
  assert(!map.configure(affine,concave,4));
  const sorter::Point2 crossed[]={{0,0},{100,100},{100,0},{0,100}};
  assert(!map.configure(affine,crossed,4));
  const sorter::Point2 star[]={{0,100},{59,-81},{-95,31},{95,31},{-59,-81}};
  assert(!map.configure(affine,star,5));
  assert(!map.configure(affine,quad,2));
  // Affine mapping may be valid mathematically yet too large for wire int32.
  const double huge[]={10000000,0,0,0,10000000,0,0,0,1};
  assert(map.configure(huge,quad,4));
  assert(!map.mapTenths(100,100,x10,y10));
}
void testSession() {
  sorter::CameraSession session;
  assert(session.accept(1,1,10)==sorter::RequestDecision::New);
  assert(session.remember("@cached\n"));
  assert(session.accept(1,1,10)==sorter::RequestDecision::Duplicate);
  assert(!strcmp(session.response(),"@cached\n"));
  assert(session.accept(1,1,11)==sorter::RequestDecision::Conflict);
  assert(!strcmp(session.response(),"@cached\n"));
  assert(session.accept(1,2,10)==sorter::RequestDecision::New && !session.response()[0]);
  assert(session.remember("@next\n"));
  assert(session.accept(1,1,10)==sorter::RequestDecision::Stale);
  assert(!strcmp(session.response(),"@next\n"));
  assert(session.accept(2,1,10)==sorter::RequestDecision::New);
  assert(session.accept(0,2,10)==sorter::RequestDecision::Invalid);
  assert(session.accept(2,0,10)==sorter::RequestDecision::Invalid);
  assert(session.accept(2,2,0)==sorter::RequestDecision::Invalid);
  assert(session.accept(2,UINT32_MAX,10)==sorter::RequestDecision::New);
  assert(session.accept(2,1,10)==sorter::RequestDecision::Stale);
  assert(session.accept(3,1,10)==sorter::RequestDecision::New);
  std::vector<char> oversized(sorter::kMaxFrame+1,'x'); oversized.back()='\0';
  assert(!session.remember(oversized.data()) && !session.remember(NULL) && !session.remember(""));
}
}

int main() {
  testRgb(); testDetector(); testFullFrameAndNoStaleResults(); testHomography(); testSession();
  puts("vision: RGB565BE, separated objects, ROI/size/overflow, full-frame queue, homography and request cache passed");
  return 0;
}
