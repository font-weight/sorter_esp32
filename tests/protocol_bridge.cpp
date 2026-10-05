#include <Protocol.h>
#include <iostream>
#include <string>
int main() {
    std::string line;
    while(std::getline(std::cin,line)) {
        sorter::Packet p;char frame[sorter::kMaxFrame];
        if(line.find('\0')==std::string::npos && sorter::decodePacket(line.c_str(),p) && sorter::encodePacket(p,frame,sizeof(frame))) std::cout<<frame;
        else std::cout<<"INVALID\n";
    }
}
