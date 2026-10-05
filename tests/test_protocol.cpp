#include <Protocol.h>
#include <assert.h>
#include <stdio.h>
#include <string>
#include <string.h>

using namespace sorter;
static std::string framed(const std::string& body) {
    char tail[16]; snprintf(tail, sizeof(tail), "*%04X\n", crc16(reinterpret_cast<const uint8_t*>(body.data()), body.size()));
    return "@" + body + tail;
}
int main() {
    assert(crc16(reinterpret_cast<const uint8_t*>("123456789"),9)==0x29B1);
    uint32_t u=123; int32_t i=123;
    assert(parseU32("4294967295",u) && u==UINT32_MAX);
    assert(!parseU32("4294967296",u));
    assert(!parseU32("-1",u)); assert(!parseU32("1.0",u)); assert(!parseU32("1x",u));
    assert(!parseU32("",u)); assert(!parseU32(" 1",u));
    assert(parseI32("-2147483648",i) && i==INT32_MIN);
    assert(parseI32("2147483647",i) && i==INT32_MAX);
    assert(!parseI32("2147483648",i)); assert(!parseI32("-2147483649",i));

    Packet q; q.type='Q';q.session=17;q.seq=2;strcpy(q.payload,"123");
    char encoded[kMaxFrame];assert(encodePacket(q,encoded,sizeof(encoded)));
    Packet decoded;assert(decodePacket(encoded,decoded));
    assert(decoded.type=='Q' && decoded.session==17 && decoded.seq==2 && strcmp(decoded.payload,"123")==0);
    const std::string original(encoded);
    assert(decodePacket((original.substr(0, original.size()-1)+"\r\n").c_str(),decoded));
    const Packet beforeInvalid = decoded;
    assert(!decodePacket("@broken",decoded));
    assert(decoded.type==beforeInvalid.type && decoded.session==beforeInvalid.session &&
           decoded.seq==beforeInvalid.seq && strcmp(decoded.payload,beforeInvalid.payload)==0);
    for(size_t n=1;n<original.size()-1;++n) {
        std::string damaged=original;damaged[n]=damaged[n]=='1'?'2':'1';
        assert(!decodePacket(damaged.c_str(),decoded));
    }
    const char* invalidBodies[]={"1|Q|0|1|2","1|Q|1|0|2","1|Q|4294967296|1|2", "1|Q|1|1|0",
        "2|Q|1|1|2", "1|Q|1|1|2|3", "1|QQ|1|1|2", "1|Q|1|1|+2", "1|E|1|1|oops",
        "1|E|1|1|E@RR", "1|E|1|1|BAD REASON", "1|X|1|1|OK", "1|D|1|1|1,2,1"};
    for(const char* body:invalidBodies) assert(!decodePacket(framed(body).c_str(),decoded));

    Scene scene;scene.cameraBoot=3;scene.calibrationId=4;scene.count=2;
    scene.objects[0].classId=1;scene.objects[0].x10=-11;scene.objects[0].y10=2147483647;scene.objects[0].pixels=99;
    scene.objects[1].classId=1;scene.objects[1].x10=INT32_MIN;scene.objects[1].y10=10;scene.objects[1].pixels=88;
    Packet d;d.type='D';d.session=17;d.seq=2;assert(encodeScene(scene,d.payload,sizeof(d.payload)));
    assert(encodePacket(d,encoded,sizeof(encoded)));assert(decodePacket(encoded,decoded));
    Scene output;assert(decodeScene(decoded.payload,output));assert(output.count==2 && output.objects[1].x10==INT32_MIN);
    assert(output.objects[0].classId==output.objects[1].classId); // same-color objects stay independent
    const char* invalidScenes[]={"1,2,0;", "1,2,9", "0,2,0", "1,0,0", "1,2,-1", "1,2,1;1,0,0,0",
       "1,2,1;4,0,0,1", "1,2,1;1,0,0,1;", "1,2,1;1,2147483648,0,1", "1,2,1;1,0,0,1,2",
       "1,2,0;1,0,0,1", "1,2,2;1,0,0,1", "1,2,1;1,0,0,1;2,0,0,1"};
    for(const char* payload:invalidScenes) assert(!decodeScene(payload,output));
    scene.cameraBoot=UINT32_MAX;scene.calibrationId=UINT32_MAX;scene.count=8;
    for(size_t k=0;k<8;++k) {
        scene.objects[k].classId=uint8_t(k%3+1);
        scene.objects[k].x10=INT32_MIN;scene.objects[k].y10=INT32_MAX;
        scene.objects[k].pixels=UINT32_MAX;
    }
    assert(encodeScene(scene,d.payload,sizeof(d.payload)));
    assert(encodePacket(d,encoded,sizeof(encoded)) && decodePacket(encoded,decoded));
    assert(decodeScene(decoded.payload,output) && output.count==8);
    assert(output.objects[7].pixels==UINT32_MAX && output.objects[7].x10==INT32_MIN);
    const Scene beforeBadScene=output;
    assert(!decodeScene("1,2,1;1,0,0,0",output));
    assert(output.count==beforeBadScene.count && output.objects[7].pixels==UINT32_MAX);
    char tiny[8];assert(!encodePacket(q,tiny,sizeof(tiny)));assert(tiny[0]==0);

    LineParser parser;int frames=0;
    std::string stream="garbage\n@broken@"+original.substr(1)+"@"+std::string(600,'Z')+"\n"+original;
    for(char c:stream) if(parser.feed(c,decoded)) ++frames;
    assert(frames==2);assert(parser.errors()>=2);
    parser.reset();frames=0;
    std::string nulStream=original.substr(0,8);nulStream.push_back('\0');
    nulStream+=original.substr(8)+original;
    for(char c:nulStream) if(parser.feed(c,decoded)) ++frames;
    assert(frames==1);
    parser.reset();
    for(size_t cut=0;cut<original.size();++cut) {
        parser.reset();int count=0;
        for(size_t n=0;n<cut;++n) if(parser.feed(original[n],decoded))++count;
        for(size_t n=cut;n<original.size();++n) if(parser.feed(original[n],decoded))++count;
        assert(count==1);
    }
    puts("protocol tests passed");
}
